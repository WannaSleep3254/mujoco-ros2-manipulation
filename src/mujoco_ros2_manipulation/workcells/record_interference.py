"""Optional owned-window recording of the actual coordinated avoidance motion."""
import ctypes
import json
from pathlib import Path
import threading
import time

import cv2
import numpy as np

from mujoco_ros2_manipulation.paths import ROOT
from mujoco_ros2_manipulation.verification.images import (
    find_gui_windows,
    window_image,
    save_window_image,
    XEvent,
    XButtonEvent,
)
from mujoco_ros2_manipulation.workcells.record_contact_demo import show_mujoco
from mujoco_ros2_manipulation.workcells.verify import require
from mujoco_ros2_manipulation.workcells.verify_interference import (
    InterferenceVerification,
)


def zoom_owned_window(window, steps=6):
    x = ctypes.CDLL("libX11.so.6")
    x.XOpenDisplay.argtypes = [ctypes.c_char_p]
    x.XOpenDisplay.restype = ctypes.c_void_p
    x.XDefaultRootWindow.argtypes = [ctypes.c_void_p]
    x.XDefaultRootWindow.restype = ctypes.c_ulong
    x.XSendEvent.argtypes = [
        ctypes.c_void_p,
        ctypes.c_ulong,
        ctypes.c_int,
        ctypes.c_long,
        ctypes.POINTER(XEvent),
    ]
    x.XWarpPointer.argtypes = [
        ctypes.c_void_p,
        ctypes.c_ulong,
        ctypes.c_ulong,
        ctypes.c_int,
        ctypes.c_int,
        ctypes.c_uint,
        ctypes.c_uint,
        ctypes.c_int,
        ctypes.c_int,
    ]
    x.XFlush.argtypes = [ctypes.c_void_p]
    x.XCloseDisplay.argtypes = [ctypes.c_void_p]
    display = x.XOpenDisplay(None)
    require(bool(display), "No X11 display for recording")
    try:
        target = int(window, 16)
        root = x.XDefaultRootWindow(display)
        x.XWarpPointer(display, 0, target, 0, 0, 0, 0, 720, 450)
        for _ in range(steps):
            event = XEvent()
            event.button = XButtonEvent(
                type=4,
                send_event=1,
                display=display,
                window=target,
                root=root,
                x=720,
                y=450,
                button=5,
                same_screen=1,
            )
            x.XSendEvent(display, target, 0, 1 << 2, ctypes.byref(event))
            event.button.type = 5
            x.XSendEvent(display, target, 0, 1 << 3, ctypes.byref(event))
            x.XFlush(display)
            time.sleep(0.03)
    finally:
        x.XCloseDisplay(display)


class RecordingMixin:
    def setup_recording(self, directory, video="coordinated-avoidance.mp4"):
        self.video = video
        self.directory = Path(directory).resolve()
        require(
            self.directory.is_relative_to(ROOT / "runtime"),
            "Recording directory must be under runtime/",
        )
        self.directory.mkdir(parents=True, exist_ok=False)
        self.stop_recording = threading.Event()
        self.thread = None
        self.errors = []
        self.metadata = {}

    def before_avoidance_execution(self):
        ready = "Ready to take commands for planning group both_arms."
        self.until(
            lambda: ready in self.launch_log_path.read_text(),
            25,
            "RViz ready before recording",
        )
        self.windows = find_gui_windows(self.process_group)
        show_mujoco(self.windows)
        zoom_owned_window(self.windows["ros2_control_node"])
        deadline = time.monotonic() + 1
        self.until(lambda: time.monotonic() >= deadline, 3, "camera ready")
        save_window_image(
            self.windows["ros2_control_node"], self.directory / "before.png"
        )
        self.metadata["video_start_clock_sec"] = self.clock_last
        self.thread = threading.Thread(target=self.record, name="avoidance-recording")
        self.thread.start()

    def record(self):
        writer = None
        frames = 0
        captures = 0
        fps = 24
        maximum_gap = 0.0
        started = previous = time.monotonic()
        try:
            while not self.stop_recording.is_set():
                image = window_image(self.windows["ros2_control_node"])
                now = time.monotonic()
                maximum_gap = max(maximum_gap, now - previous)
                previous = now
                captures += 1
                if writer is None:
                    writer = cv2.VideoWriter(
                        str(self.directory / self.video),
                        cv2.VideoWriter_fourcc(*"avc1"),
                        fps,
                        image.size,
                    )
                    require(writer.isOpened(), "H264 encoder unavailable")
                pixels = cv2.cvtColor(np.asarray(image), cv2.COLOR_RGB2BGR)
                target = max(frames + 1, int((now - started) * fps) + 1)
                while frames < target:
                    writer.write(pixels)
                    frames += 1
                self.stop_recording.wait(
                    max(0, started + frames / fps - time.monotonic())
                )
        except Exception as error:
            self.errors.append(str(error))
        finally:
            if writer:
                writer.release()
            self.metadata.update(
                frames=frames,
                captures=captures,
                fps=fps,
                wall_sec=time.monotonic() - started,
                playback_sec=frames / fps,
                maximum_capture_gap_sec=maximum_gap,
                kind="Actual owned MuJoCo X11 pixels; no overlays or speed changes; repeated captured frames preserve wall time",
                video=self.video,
                camera_wheel_down_steps=6,
            )

    def recorded_run(self, execute):
        try:
            result = execute()
            deadline = time.monotonic() + 1
            self.until(lambda: time.monotonic() >= deadline, 3, "show completed motion")
            save_window_image(
                self.windows["ros2_control_node"], self.directory / "after.png"
            )
        finally:
            self.stop_recording.set()
            if self.thread:
                self.thread.join(timeout=15)
                if self.thread.is_alive():
                    self.errors.append("Recorder did not stop")
            self.metadata["errors"] = self.errors
            (self.directory / "recording.json").write_text(
                json.dumps(self.metadata, indent=2) + "\n"
            )
        require(not self.errors, str(self.errors))
        result["recording"] = dict(
            directory=str(self.directory.relative_to(ROOT)), **self.metadata
        )
        return result


class RecordedInterference(RecordingMixin, InterferenceVerification):
    def __init__(self, cell, progress, case, directory):
        super().__init__(cell, progress, case)
        self.setup_recording(directory)

    def run(self):
        return self.recorded_run(super().run)
