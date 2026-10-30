"""Start the local FR5/UR5e simulation operator console."""
import argparse
from datetime import datetime, timezone
import os
from pathlib import Path
import secrets
import signal
import threading
import uuid
from urllib.parse import urlsplit


def local_grasp_viewer_url(value):
    """An explicit navigation destination; never discovers or controls a peer."""
    try:
        parsed = urlsplit(value)
        if (parsed.scheme != "http" or parsed.hostname != "127.0.0.1"
                or parsed.username is not None or parsed.password is not None
                or parsed.port is None or not 1024 <= parsed.port <= 65535
                or parsed.path not in ("", "/") or parsed.query or parsed.fragment):
            raise ValueError()
        return "http://127.0.0.1:"+str(parsed.port)+"/"
    except (TypeError, ValueError):
        raise argparse.ArgumentTypeError("ROS grasp viewer must be http://127.0.0.1:<port>/") from None


def validate_workspace_options(args):
    """Startup-only task registration; a browser cannot choose files or domains."""
    selected = (args.ros_grasp_environment, args.ros_grasp_manifest_sha256, args.ros_grasp_domain)
    if any(value is not None for value in selected):
        if any(value is None for value in selected):
            raise ValueError("집기 모델 경로·manifest SHA256·전용 ROS 도메인을 함께 지정하세요.")
        if not 0 <= args.ros_grasp_domain <= 232 or args.ros_grasp_domain == args.domain:
            raise ValueError("일반 조작과 다른 빈 집기 ROS 도메인(0..232)을 지정하세요.")
        if args.no_render:
            raise ValueError("통합 집기 작업은 같은 3D 화면을 사용하므로 --no-render와 함께 실행할 수 없습니다.")
        if args.execution_environment is not None:
            raise ValueError("첫 통합 집기는 기본 작업 환경과 명시한 전용 집기 모델을 사용합니다.")
        if args.ros_grasp_viewer is not None:
            raise ValueError("통합 집기와 별도 집기 화면 링크 중 하나를 명시하세요.")
    if len(args.saved_recording) > 8:
        raise ValueError("시작할 때 등록할 보존 기록은 최대 8개입니다.")


def shutdown_console(server, app, ros_stopped, ros_thread, node, rclpy, previous):
    """Always release ROS resources, including when a recording cannot finish."""
    clean = True
    def attempt(stage, operation):
        nonlocal clean
        try:
            return operation()
        except Exception as problem:
            clean = False
            print(f"OPERATOR_CLEANUP_FAILED stage={stage} error_type={type(problem).__name__}", flush=True)
            return None
    try:
        attempt("http_shutdown", server.shutdown)
        attempt("http_close", server.server_close)
        if attempt("application", app.close) is False:
            clean = False
    finally:
        ros_stopped.set()
        attempt("ros_thread", lambda:ros_thread.join(timeout=3))
        if attempt("ros_thread_status", ros_thread.is_alive):
            clean = False
        try:
            attempt("ros_node", node.destroy_node)
        finally:
            if attempt("ros_context_status", rclpy.ok):
                attempt("ros_context", rclpy.shutdown)
            for sig, handler in previous.items():
                attempt("signal_restore", lambda sig=sig, handler=handler:signal.signal(sig, handler))
    return clean


def main():
    # Console validators inspect manufacturer Git state; optional index refresh
    # must not mutate .git metadata while the native owner pins file identities.
    os.environ["GIT_OPTIONAL_LOCKS"] = "0"
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
    parser.add_argument("--ros-grasp-viewer", type=local_grasp_viewer_url,
                        help="Explicit local ROS grasp viewer URL, shown as a separate-task link")
    parser.add_argument("--ros-grasp-environment", type=Path,
                        help="Explicit immutable native ROS grasp target for the shared workspace UI")
    parser.add_argument("--ros-grasp-manifest-sha256", help="Expected SHA256 of the registered grasp.json")
    parser.add_argument("--ros-grasp-domain", type=int, help="Unused fixed ROS domain distinct from the normal task")
    parser.add_argument("--saved-recording", type=Path, action="append", default=[],
                        help="Explicit local synchronized recording to copy into read-only workspace analysis (max 8)")
    args = parser.parse_args()
    if not 0 <= args.domain <= 232 or not 1024 <= args.port <= 65535:
        parser.error("ROS domain must be 0..232; port must be 1024..65535")
    if args.image_clock_domain != "unknown" and args.image_topic is None:
        parser.error("--image-clock-domain requires --image-topic")
    if args.no_render and args.view_scene_catalog is not None:
        parser.error("--view-scene-catalog requires scene display; remove --no-render")
    try:
        validate_workspace_options(args)
    except ValueError as error:
        parser.error(str(error))
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
    if (execution_target is not None
            and execution_target.description().get("artifact_kind") == "grasp-execution-target-v1"):
        parser.error("이 집기 모델은 manipulation-grasp-ros-viewer로 실행하세요. 일반 조작 화면에서는 --ros-grasp-viewer로 연결합니다.")
    from ..grasping.ros_target import load_ros_grasp_target
    grasp_target = load_ros_grasp_target(cell, args.ros_grasp_environment, args.ros_grasp_manifest_sha256)
    if grasp_target is not None:
        from ..grasping.ros_runtime import check_grasp_runtime
        check_grasp_runtime(cell.root)
    from .synchronized_capture import load_saved_recordings
    saved_recordings = load_saved_recordings(args.saved_recording)
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
        pose_tools=describe_pose_tools(cell), ros_grasp_viewer_url=args.ros_grasp_viewer)
    if execution_target is not None:
        configuration["execution_target"] = execution_target.description()
    state = ConsoleState(configuration)
    node = ConsoleBridge(state, cell, execution_target=execution_target, image_topic=args.image_topic, image_clock_domain=args.image_clock_domain)
    folder = cell.root/"runtime/operator"/(datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")+"-"+uuid.uuid4().hex[:8])
    grasp_application = None
    app = None
    try:
        if grasp_target is not None:
            from ..grasping.ros_demo_server import RosDemoApplication
            grasp_application = RosDemoApplication(grasp_target, folder.with_name(folder.name+"-grasp"),
                                                   domain=args.ros_grasp_domain)
        app = ConsoleApplication(cell, state, node, folder, render=not args.no_render,view_sources=view_sources,
                                 execution_target=execution_target, grasp_application=grasp_application)
        for recording in saved_recordings:
            app.sync_recordings.add_saved(recording)
    except Exception:
        try:
            if app is not None:
                app.close()
            elif grasp_application is not None:
                grasp_application.close()
        finally:
            server.server_close()
            node.destroy_node()
            rclpy.shutdown()
        raise
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
        clean = shutdown_console(server, app, ros_stopped, ros_thread, node, rclpy, previous)
    return 0 if clean else 1


if __name__ == "__main__":
    raise SystemExit(main())
