"""Optional X11/H.264 recording of the actual GUI during the contact demo."""
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
    XClientMessage,
)
from mujoco_ros2_manipulation.workcells.verify_contact_demo import VisibleContactDemo


def show_mujoco(windows):
    """Resize/activate this launch's simulation and hide only its RViz window."""
    x = ctypes.CDLL("libX11.so.6")
    x.XOpenDisplay.argtypes = [ctypes.c_char_p]
    x.XOpenDisplay.restype = ctypes.c_void_p
    x.XDefaultScreen.argtypes = [ctypes.c_void_p]
    x.XDefaultScreen.restype = ctypes.c_int
    x.XIconifyWindow.argtypes = [ctypes.c_void_p, ctypes.c_ulong, ctypes.c_int]
    x.XMapRaised.argtypes = [ctypes.c_void_p, ctypes.c_ulong]
    x.XMoveResizeWindow.argtypes = [
        ctypes.c_void_p,
        ctypes.c_ulong,
        ctypes.c_int,
        ctypes.c_int,
        ctypes.c_uint,
        ctypes.c_uint,
    ]
    x.XDefaultRootWindow.argtypes = [ctypes.c_void_p]
    x.XDefaultRootWindow.restype = ctypes.c_ulong
    x.XInternAtom.argtypes = [ctypes.c_void_p, ctypes.c_char_p, ctypes.c_int]
    x.XInternAtom.restype = ctypes.c_ulong
    x.XSendEvent.argtypes = [
        ctypes.c_void_p,
        ctypes.c_ulong,
        ctypes.c_int,
        ctypes.c_long,
        ctypes.POINTER(XEvent),
    ]
    x.XFlush.argtypes = [ctypes.c_void_p]
    x.XCloseDisplay.argtypes = [ctypes.c_void_p]
    display = x.XOpenDisplay(None)
    if not display:
        raise RuntimeError("Cannot open desktop X11 display")
    try:
        window = int(windows["ros2_control_node"], 16)
        x.XIconifyWindow(display, int(windows["rviz2"], 16), x.XDefaultScreen(display))
        x.XMoveResizeWindow(display, window, 40, 60, 1440, 900)
        x.XMapRaised(display, window)
        active = XEvent()
        active.client = XClientMessage(
            type=33,
            send_event=1,
            display=display,
            window=window,
            message_type=x.XInternAtom(display, b"_NET_ACTIVE_WINDOW", 0),
            format=32,
        )
        active.client.data.longs[0] = 2
        x.XSendEvent(
            display,
            x.XDefaultRootWindow(display),
            0,
            (1 << 19) | (1 << 20),
            ctypes.byref(active),
        )
        x.XFlush(display)
    finally:
        x.XCloseDisplay(display)


class RecordedContactDemo(VisibleContactDemo):
    def __init__(self, cell, progress, directory, control=False):
        super().__init__(cell, progress, control=control)
        self.directory = Path(directory).resolve()
        if not self.directory.is_relative_to(ROOT / "runtime"):
            raise ValueError(
                "Record into a new directory under runtime/; publish reviewed assets separately"
            )
        self.directory.mkdir(parents=True, exist_ok=False)
        self.stop_recording = threading.Event()
        self.record_errors = []
        self.record_metadata = {}
        self.record_thread = None

    def run(self):
        try:
            result = super().run()
        finally:
            self.stop_recording.set()
            if self.record_thread:
                self.record_thread.join(timeout=15)
                if self.record_thread.is_alive():
                    self.record_errors.append("Recorder thread did not stop")
            self.record_metadata["errors"] = self.record_errors
            (self.directory / "recording.json").write_text(
                json.dumps(self.record_metadata, indent=2) + "\n"
            )
        if self.record_errors:
            raise RuntimeError(str(self.record_errors))
        result["recording"] = dict(
            self.record_metadata, directory=str(self.directory.relative_to(ROOT))
        )
        return result

    def before_motion(self):
        # RViz may activate its window late while initializing the MoveIt panel.
        # Wait for that initialization before hiding it and starting the recording.
        log_path = self.launch_log_path
        ready = f'Ready to take commands for planning group {self.cell.ros["planning_group"]}.'
        self.until(
            lambda: ready in log_path.read_text(),
            25,
            "RViz panel ready before recording",
        )
        self.windows = find_gui_windows(self.process_group)
        show_mujoco(self.windows)
        until = time.monotonic() + 1
        self.until(lambda: time.monotonic() >= until, 3, "camera ready")
        save_window_image(
            self.windows["ros2_control_node"], self.directory / "before.png"
        )
        self.record_thread = threading.Thread(
            target=self.record, name="contact-demo-recording"
        )
        self.record_metadata["video_start_clock_sec"] = self.clock_last
        self.record_thread.start()
        until = time.monotonic() + 1
        self.until(lambda: time.monotonic() >= until, 3, "initial recorded scene")

    def after_stop(self):
        save_window_image(
            self.windows["ros2_control_node"], self.directory / "stopped.png"
        )
        until = time.monotonic() + 2
        self.until(lambda: time.monotonic() >= until, 4, "show stopped scene")

    def record(self):
        writer = None
        frames, fps = 0, 24
        started = time.monotonic()
        maximum_capture_gap = 0.0
        previous_capture = started
        try:
            while not self.stop_recording.is_set():
                image = window_image(self.windows["ros2_control_node"])
                captured = time.monotonic()
                maximum_capture_gap = max(
                    maximum_capture_gap, captured - previous_capture
                )
                previous_capture = captured
                pixels = cv2.cvtColor(np.asarray(image), cv2.COLOR_RGB2BGR)
                if writer is None:
                    writer = cv2.VideoWriter(
                        str(self.directory / "approach-contact-stop.mp4"),
                        cv2.VideoWriter_fourcc(*"avc1"),
                        fps,
                        image.size,
                    )
                    if not writer.isOpened():
                        raise RuntimeError("H.264 video encoder unavailable")
                target = max(frames + 1, int((captured - started) * fps) + 1)
                while frames < target:
                    writer.write(pixels)
                    frames += 1
                self.stop_recording.wait(
                    max(0, started + frames / fps - time.monotonic())
                )
        except Exception as error:
            self.record_errors.append(str(error))
        finally:
            if writer:
                writer.release()
            self.record_metadata.update(
                frames=frames,
                fps=fps,
                playback_duration_sec=frames / fps,
                wall_seconds=time.monotonic() - started,
                maximum_capture_gap_sec=maximum_capture_gap,
                kind="Actual owned MuJoCo X11 client pixels; no overlays or speed changes",
                video="approach-contact-stop.mp4",
            )
