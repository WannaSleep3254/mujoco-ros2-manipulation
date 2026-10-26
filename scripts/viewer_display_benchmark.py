#!/usr/bin/env python3
"""Isolated, read-only moving-recording JPEG/WebGL display comparison.

No ROS imports, physics stepping, robot commands, active-console calls or shared
browser profile. Each trial starts a fresh HTTP server and Chrome process. The
server uses the same frozen positions, MuJoCo FK, resolution and camera for both
modes. Its JPEG branch renders/encodes on request; it deliberately is not the
production asynchronous 5 Hz worker, and must not be reported as that pipeline.
"""
from __future__ import annotations

import argparse
from hashlib import sha256
from http.server import BaseHTTPRequestHandler, HTTPServer
import io
import json
import math
import os
from pathlib import Path
import subprocess
import sys
from urllib.parse import parse_qs, urlsplit

ROOT = Path(__file__).resolve().parents[1]
STATIC = ROOT / "src/mujoco_ros2_manipulation/operator_console/static"
sys.path.insert(0, str(ROOT / "src"))


def canonical(value):
    return json.dumps(value, sort_keys=True, separators=(",", ":"), allow_nan=False).encode()


def load_recording(path):
    """Small explicit, finite, complete named-state recording; no interpolation."""
    raw = Path(path).read_bytes()
    if len(raw) > 32 * 1024 * 1024:
        raise ValueError("Recording exceeds 32 MiB")
    value = json.loads(raw)
    if (not isinstance(value, dict) or set(value) != {"schema_version", "joint_names", "samples"}
            or type(value["schema_version"]) is not int or value["schema_version"] != 1):
        raise ValueError("Expected recording schema 1")
    names, samples = value["joint_names"], value["samples"]
    if (not isinstance(names, list) or not names or len(names) > 512
            or any(not isinstance(n, str) or not n or len(n) > 256 for n in names)
            or len(set(names)) != len(names) or not isinstance(samples, list)
            or not 2 <= len(samples) <= 10000):
        raise ValueError("Invalid recording catalog")
    previous = -1
    for sample in samples:
        if not isinstance(sample, dict) or set(sample) != {"time_sec", "positions"}:
            raise ValueError("Invalid recording sample fields")
        stamp, state = sample["time_sec"], sample["positions"]
        if (type(stamp) not in (int, float) or not 0 <= stamp <= 1e9 or not math.isfinite(stamp) or stamp <= previous
                or not isinstance(state, dict) or set(state) != set(names)
                or any(type(q) not in (int, float) or abs(q) > 1e6 or not math.isfinite(q) for q in state.values())):
            raise ValueError("Recording must contain finite complete, increasing samples")
        previous = stamp
    return value, sha256(raw).hexdigest()


def fixture(directory):
    """Portable two-joint articulated fixture, never described as a real robot."""
    model = directory / "fixture.xml"
    model.write_text('''<mujoco model="display_fixture"><compiler angle="radian"/>
      <visual><global fovy="45" offwidth="640" offheight="480" ipd="0"/></visual>
      <worldbody><light pos="2 -2 4"/><geom type="box" size="1 1 .05" pos="0 0 -.05" rgba=".3 .35 .4 1"/>
      <body pos="0 0 .1"><joint name="base" axis="0 0 1"/>
      <geom type="capsule" size=".07 .3" pos="0 0 .3" rgba=".6 .7 .8 1"/>
      <body pos="0 0 .6"><joint name="elbow" axis="0 1 0"/>
      <geom type="capsule" size=".05 .25" pos="0 0 .25" rgba=".8 .5 .3 1"/></body></body></worldbody></mujoco>''')
    recording = directory / "recording.json"
    recording.write_bytes(canonical(dict(schema_version=1, joint_names=["base", "elbow"], samples=[
        dict(time_sec=i / 20, positions=dict(base=.4 * math.sin(i * math.pi / 40), elbow=.7 * math.sin(i * math.pi / 30)))
        for i in range(161)])))
    return model, recording


def page():
    return '''<!doctype html><meta charset="utf-8"><title>Isolated read-only display benchmark</title>
    <style>html,body{margin:0;background:#17242a}#stage{position:relative}canvas{display:block}.local-scene-status{display:none}</style>
    <script type="importmap">{"imports":{"three":"/static/vendor/three.module.js"}}</script>
    <div id="stage"></div><script type="module" src="/benchmark-page.mjs"></script>'''.encode()


def serve(args):
    os.environ.setdefault("MUJOCO_GL", "egl")
    import mujoco
    import numpy as np
    from PIL import Image
    from mujoco_ros2_manipulation.operator_console.scene_view import CompiledScene

    recording, record_sha = load_recording(args.recording)
    scene = CompiledScene(args.model, recording["joint_names"], recording["samples"][0]["positions"])
    manifest = scene.manifest()
    model = scene._model  # A private owned benchmark scene, never the active application.
    model.vis.global_.offwidth, model.vis.global_.offheight = args.width, args.height
    model.vis.global_.fovy, model.vis.global_.ipd = 45, 0
    data = scene._data(recording["samples"][0]["positions"])
    camera = mujoco.MjvCamera()
    camera.lookat[:] = [0, 0, .75]
    camera.azimuth, camera.elevation, camera.distance = 125, -24, 2.6
    # MjvScene camera extraction is CPU-only. Both branches use its central
    # eye, up, forward and vertical FOV, so projection/pixels are controlled.
    options = mujoco.MjvOption()
    options.geomgroup[:] = np.array([1, 1, 1, 0, 0, 0])
    camera_scene = mujoco.MjvScene(model, maxgeom=max(1, model.ngeom))
    mujoco.mjv_updateScene(model, data, options, None, camera, mujoco.mjtCatBit.mjCAT_ALL, camera_scene)
    eye = camera_scene.camera[0]
    camera_spec = dict(position=list(map(float, eye.pos)), forward=list(map(float, eye.forward)),
        up=list(map(float, eye.up)), fov=45, near=float(eye.frustum_near), far=float(eye.frustum_far),
        target=list(map(float, camera.lookat)))
    renderer = mujoco.Renderer(model, height=args.height, width=args.width) if args.mode == "image" else None
    native_gl = None
    if renderer:
        from OpenGL.GL import GL_RENDERER, GL_VENDOR, GL_VERSION, glGetString
        native_gl = {key: glGetString(token).decode("utf-8", "replace")
            for key, token in (("renderer", GL_RENDERER), ("vendor", GL_VENDOR), ("version", GL_VERSION))}
    binding = dict(operator_session_id="a" * 32, generation=1, mode="live", source_id="benchmark",
        job_id=None, run_id=None, motion_session_id=None, request_id=None)
    binding_id = "b" * 64
    config = dict(mode=args.mode, width=args.width, height=args.height, poll_ms=args.poll_ms,
        camera=camera_spec, binding=binding, binding_id=binding_id,
        model_sha256=manifest["model_sha256"], scene_sha256=manifest["scene_sha256"],
        record_sha256=record_sha, sample_count=len(recording["samples"]),
        sample_times_sec=[sample["time_sec"] for sample in recording["samples"]],
        source_period_sec=recording["samples"][-1]["time_sec"], mujoco_version=mujoco.__version__, native_gl=native_gl)

    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *_):
            pass

        def send(self, payload, content_type, status=200):
            self.send_response(status)
            self.send_header("Content-Type", content_type)
            self.send_header("Content-Length", str(len(payload)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(payload)

        def do_GET(self):
            path = urlsplit(self.path)
            try:
                if path.path == "/":
                    return self.send(page(), "text/html")
                if path.path == "/benchmark-page.mjs":
                    return self.send((ROOT / "scripts/viewer_display_benchmark_page.mjs").read_bytes(), "text/javascript")
                if path.path.startswith("/static/"):
                    file = (STATIC / path.path.removeprefix("/static/")).resolve()
                    if not file.is_relative_to(STATIC.resolve()) or not file.is_file() or file.suffix not in {".js", ".mjs"}:
                        return self.send(b"", "text/plain", 404)
                    return self.send(file.read_bytes(), "text/javascript")
                if path.path == "/config":
                    return self.send(canonical(config), "application/json")
                if path.path == "/manifest":
                    return self.send(canonical(manifest), "application/json")
                if path.path.startswith("/assets/"):
                    digest = path.path.removeprefix("/assets/")
                    if len(digest) != 64 or any(c not in "0123456789abcdef" for c in digest):
                        return self.send(b"", "text/plain", 404)
                    return self.send(scene.asset(digest), "application/octet-stream")
                if path.path not in {"/pose", "/frame"}:
                    return self.send(b"", "text/plain", 404)
                query = parse_qs(path.query)
                index, sequence = int(query.get("index", ["0"])[0]), int(query.get("sequence", ["1"])[0])
                zoom = float(query.get("zoom", ["1"])[0])
                if not 0 <= index < len(recording["samples"]) or not 1 <= sequence <= 2**53 - 1 or not .1 <= zoom <= 10:
                    raise ValueError("Invalid sample")
                sample = recording["samples"][index]
                if path.path == "/pose":
                    value = dict(schema_version=1, binding_id=binding_id, binding=binding,
                        sequence=sequence, simulation_sec=sample["time_sec"], source_state_age_sec=0,
                        pose=scene.pose(sample["positions"]))
                    return self.send(canonical(value), "application/json")
                if renderer is None:
                    return self.send(b"", "text/plain", 404)
                data = scene._data(sample["positions"])
                camera.distance = 2.6 * zoom
                renderer.update_scene(data, camera, scene_option=options)
                pixels = renderer.render()
                image = io.BytesIO()
                Image.fromarray(pixels).save(image, format="JPEG", quality=88)
                self.send(image.getvalue(), "image/jpeg")
            except (ValueError, KeyError, TypeError):
                self.send(b"invalid benchmark request", "text/plain", 400)

    server = HTTPServer(("127.0.0.1", 0), Handler)
    print(json.dumps(dict(url=f"http://127.0.0.1:{server.server_port}", model_sha256=manifest["model_sha256"],
        recording_sha256=record_sha)), flush=True)
    try:
        server.serve_forever()
    finally:
        server.server_close()
        if renderer:
            renderer.close()


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--model", type=Path)
    parser.add_argument("--recording", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--node", default="node")
    parser.add_argument("--playwright-module", default="playwright")
    parser.add_argument("--chrome", default="/usr/bin/google-chrome")
    parser.add_argument("--repeats", type=int, default=3)
    parser.add_argument("--seconds", type=float, default=15)
    parser.add_argument("--soak-seconds", type=float, default=120)
    parser.add_argument("--poll-ms", type=int, default=250)
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--mode", choices=["image", "web3d"])
    parser.add_argument("--resume-trials", action="store_true",
        help="Reuse complete matching trial JSON after an interrupted run; always restart the soak")
    parser.add_argument("--hardware-gl", action="store_true",
        help="Require desktop OpenGL ANGLE; a software WebGL renderer fails the run")
    parser.add_argument("--baseline-only", action="store_true",
        help="Save the independent baseline trials, then exit before the separate soak")
    parser.add_argument("--serve", action="store_true", help=argparse.SUPPRESS)
    args = parser.parse_args(argv)
    if not 1 <= args.repeats <= 20 or not 2 <= args.seconds <= 300 or not 1 <= args.soak_seconds <= 3600:
        parser.error("Invalid bounded repetitions/duration")
    if not 100 <= args.poll_ms <= 2000 or not 128 <= args.width <= 1920 or not 128 <= args.height <= 1080:
        parser.error("Invalid bounded polling/pixels")
    if bool(args.model) != bool(args.recording):
        parser.error("Pass both --model and --recording")
    args.output.mkdir(parents=True, exist_ok=True)
    if args.model is None:
        args.model, args.recording = fixture(args.output)
    if args.serve:
        if args.mode is None:
            parser.error("Internal server requires a mode")
        return serve(args)
    load_recording(args.recording)
    command = [args.node, str(ROOT / "scripts/viewer_display_benchmark.mjs"), "--python", sys.executable,
        "--model", str(args.model.resolve()), "--recording", str(args.recording.resolve()),
        "--output", str(args.output.resolve()), "--playwright-module", args.playwright_module,
        "--chrome", args.chrome, "--repeats", str(args.repeats), "--seconds", str(args.seconds),
        "--soak-seconds", str(args.soak_seconds), "--poll-ms", str(args.poll_ms),
        "--width", str(args.width), "--height", str(args.height)]
    if args.resume_trials:
        command += ["--resume-trials", "true"]
    if args.hardware_gl:
        command += ["--hardware-gl", "true"]
    if args.baseline_only:
        command += ["--baseline-only", "true"]
    return subprocess.run(command, check=True).returncode


if __name__ == "__main__":
    raise SystemExit(main())
