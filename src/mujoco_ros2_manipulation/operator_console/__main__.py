"""Start the local FR5/UR5e simulation operator console."""
import argparse
from datetime import datetime, timezone
import os
from pathlib import Path
import secrets
import signal
import threading
import uuid


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--workcell", default="fr5_ur5e")
    parser.add_argument("--domain", type=int, default=int(os.environ.get("ROS_DOMAIN_ID", "89")))
    parser.add_argument("--port", type=int, default=8765)
    parser.add_argument("--no-render", action="store_true", help="Disable local 3D scene workers and the optional image renderer (headless CI)")
    parser.add_argument("--view-scene-catalog", type=Path, help="Explicit local overlay catalog for read-only model previews; checked once before ROS starts")
    parser.add_argument("--execution-environment", type=Path, help="Explicit immutable execution overlay; paired manifest SHA required")
    parser.add_argument("--execution-manifest-sha256", help="Expected SHA256 of the execution overlay manifest")
    parser.add_argument("--image-topic", help="Opt-in single RGB8 sensor_msgs/Image topic; separate from rendered model images")
    parser.add_argument("--image-clock-domain", choices=("unknown", "ros_simulation"), default="unknown", help="Declared acquisition clock; never inferred from the presence of /clock")
    args = parser.parse_args()
    if not 0 <= args.domain <= 232 or not 1024 <= args.port <= 65535:
        parser.error("ROS domain must be 0..232; port must be 1024..65535")
    if args.image_clock_domain != "unknown" and args.image_topic is None:
        parser.error("--image-clock-domain requires --image-topic")
    if args.no_render and args.view_scene_catalog is not None:
        parser.error("--view-scene-catalog requires scene display; remove --no-render")
    os.environ["ROS_DOMAIN_ID"] = os.environ["MANIPULATION_ROS_DOMAIN_ID"] = str(args.domain)
    from ..workcells.config import load_workcell
    from ..workcells.integrity import check_workcell
    from ..workcells.contracts import describe_workcell
    from .state import ConsoleState
    from .http_server import ConsoleHTTPServer, Handler
    cell = load_workcell(args.workcell)
    metadata = check_workcell(cell)
    from ..workcells.execution_target import load_execution_target
    execution_target = load_execution_target(cell, args.execution_environment, args.execution_manifest_sha256)
    view_sources=()
    if args.view_scene_catalog is not None:
        from .environment_sources import load_scene_catalog
        # All paths and CPU receipts are fixed before ROS imports/entities or
        # application threads. No browser endpoint can replace this registry.
        view_sources=load_scene_catalog(args.view_scene_catalog,cell)
    if args.image_topic is not None:
        # Validation creates no ROS entities, worker or Pillow import.
        from .image_source import ImageSource
        ImageSource(source_id="camera0", topic=args.image_topic,
                    clock_domain=args.image_clock_domain)
    # Reserve the port before creating ROS entities or application workers.
    server = ConsoleHTTPServer(("127.0.0.1", args.port), Handler)
    import rclpy
    from rclpy.signals import SignalHandlerOptions
    from .ros_adapter import ConsoleBridge
    from .application import ConsoleApplication
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    from ..workcells.pose_tools import describe_pose_tools
    configuration = dict(workcell=cell.id, label=cell.label, domain=args.domain, home_rad=dict(zip(cell.joints, cell.home)),
        joint_limits=metadata["joint_limits"], instances=[dict(id=i.id, model=i.profile.id, label=i.profile.label, joints=i.joints) for i in cell.instances],
        targets=dict(home=dict(zip(cell.joints, cell.home)), sample={n:v for i in cell.instances for n,v in zip(i.joints,i.plan_target)}),
        render_enabled=not args.no_render, scene_view_version=1, workcell_contract=describe_workcell(cell, metadata),
        pose_tools=describe_pose_tools(cell))
    if execution_target is not None:
        configuration["execution_target"] = execution_target.description()
    state = ConsoleState(configuration)
    node = ConsoleBridge(state, cell, execution_target=execution_target, image_topic=args.image_topic, image_clock_domain=args.image_clock_domain)
    folder = cell.root/"runtime/operator"/(datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")+"-"+uuid.uuid4().hex[:8])
    app = ConsoleApplication(cell, state, node, folder, render=not args.no_render,view_sources=view_sources, execution_target=execution_target)
    server.application, server.token = app, secrets.token_urlsafe(32)
    stopped = threading.Event()
    ros_stopped = threading.Event()
    def spin():
        try:
            while rclpy.ok() and not ros_stopped.is_set():
                rclpy.spin_once(node, timeout_sec=0.05)
        except Exception as error:
            app.error = "ROS 연결 오류: "+str(error)
            stopped.set()
    ros_thread = threading.Thread(target=spin, daemon=True)
    http_thread = threading.Thread(target=server.serve_forever, daemon=True)
    previous = {}
    for sig in (signal.SIGINT, signal.SIGTERM):
        previous[sig] = signal.signal(sig, lambda *_: stopped.set())
    ros_thread.start()
    http_thread.start()
    print(f"OPERATOR_READY http://127.0.0.1:{args.port}/ domain={args.domain} session={folder}", flush=True)
    try:
        while not stopped.wait(0.2):
            pass
    finally:
        server.shutdown()
        server.server_close()
        app.close()
        ros_stopped.set()
        ros_thread.join(timeout=3)
        try:
            node.destroy_node()
        finally:
            if rclpy.ok():
                rclpy.shutdown()
            for sig, handler in previous.items():
                signal.signal(sig, handler)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
