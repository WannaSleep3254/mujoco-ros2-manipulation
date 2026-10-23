"""Render received ROS joint states; never integrate physics or send commands."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import signal
import time


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--workcell", default="fr5_ur5e")
    parser.add_argument("--directory", type=Path, required=True)
    parser.add_argument("--model", type=Path)
    parser.add_argument("--model-sha256")
    parser.add_argument("--job-id")
    parser.add_argument("--run-id")
    args = parser.parse_args()
    os.environ.setdefault("MUJOCO_GL", "egl")
    import mujoco
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from rclpy.signals import SignalHandlerOptions
    from sensor_msgs.msg import JointState
    from PIL import Image
    from ..workcells.config import load_workcell
    from ..workcells.integrity import check_workcell
    from ..verification.reports import atomic_json
    from ..model_build.integrity import file_sha256
    cell = load_workcell(args.workcell)
    directory = args.directory.resolve()
    if not directory.is_relative_to((cell.root/"runtime").resolve()):
        parser.error("Renderer output must be under runtime/")
    stop = [False]
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, lambda *_: stop.__setitem__(0, True))
    node = renderer = None
    try:
        check_workcell(cell)
        model_path = (cell.output/"cell.xml").resolve()
        identity = {}
        if any((args.model, args.model_sha256, args.job_id, args.run_id)):
            if not all((args.model, args.model_sha256, args.job_id, args.run_id)):
                raise ValueError("Complete demo model identity is required")
            if len(args.job_id) != 32 or any(c not in "0123456789abcdef" for c in args.job_id):
                raise ValueError("Invalid demonstration identity")
            run = cell.root/"runtime/validation/runs"/args.run_id
            if run.parent != cell.root/"runtime/validation/runs":
                raise ValueError("Invalid demonstration run")
            model_path = args.model.resolve()
            if model_path not in {(cell.output/"cell.xml").resolve(), (run/"contact_test.xml").resolve()}:
                raise ValueError("Unowned demonstration model")
            if file_sha256(model_path) != args.model_sha256:
                raise ValueError("Demonstration model hash mismatch")
            identity = dict(job_id=args.job_id, run_id=args.run_id, model_sha256=args.model_sha256)
        model = mujoco.MjModel.from_xml_path(str(model_path))
        model.vis.global_.offwidth, model.vis.global_.offheight = 960, 640
        data = mujoco.MjData(model)
        identifiers = {n:mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, n) for n in cell.joints}
        if any(i < 0 for i in identifiers.values()):
            raise RuntimeError("Renderer model is missing expected workcell joints")
        addresses = {n:int(model.jnt_qposadr[i]) for n,i in identifiers.items()}
        rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
        node = Node("operator_joint_state_renderer")
        latest = {}
        def joints(message):
            if len(message.name) == len(set(message.name)) and set(message.name) == set(cell.joints) and len(message.position) == len(cell.joints):
                import math
                if all(math.isfinite(v) for v in message.position):
                    latest.update(state=dict(zip(message.name, message.position)), at=time.monotonic(), simulation_sec=message.header.stamp.sec+message.header.stamp.nanosec/1e9)
        node.create_subscription(JointState, "/joint_states", joints, QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
        renderer = mujoco.Renderer(model, height=640, width=960)
        camera = mujoco.MjvCamera()
        camera.lookat[:] = [0, 0, 0.75]
        presets = dict(overview=(125, -24, 2.6), front=(90, -14, 2.6), top=(90, -85, 2.8))
        sequence = 0
        next_frame = 0
        while not stop[0] and rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.01)
            now = time.monotonic()
            if now < next_frame or not latest or now-latest["at"] > 0.5:
                continue
            next_frame = now+0.2
            for name, address in addresses.items():
                data.qpos[address] = latest["state"][name]
            mujoco.mj_forward(model, data)
            view = "overview"
            if (directory/"camera.json").is_file():
                view = json.loads((directory/"camera.json").read_text())["view"]
            camera.azimuth, camera.elevation, camera.distance = presets[view]
            renderer.update_scene(data, camera)
            pixels = renderer.render()
            temporary = directory/"frame.tmp.jpg"
            Image.fromarray(pixels).save(temporary, quality=88)
            jpeg_sha256 = hashlib.sha256(temporary.read_bytes()).hexdigest()
            temporary.replace(directory/"frame.jpg")
            sequence += 1
            atomic_json(directory/"frame.json", dict(sequence=sequence, jpeg_sha256=jpeg_sha256, rendered_monotonic=time.monotonic(),
                source_state_age_sec=now-latest["at"], simulation_sec=latest["simulation_sec"], view=view,
                renderer="MuJoCo "+mujoco.__version__, **identity, kind="ROS joint-state visualization; not a second physics simulation"))
    except Exception as error:
        atomic_json(directory/"frame.json", dict(error=type(error).__name__+": "+str(error)))
        raise
    finally:
        if renderer is not None:
            renderer.close()
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
