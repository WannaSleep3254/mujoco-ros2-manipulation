"""Open an explicitly selected immutable sensor/3D record without ROS.

python -m mujoco_ros2_manipulation.operator_console.synchronized_server \
    --recording runtime/operator/<session>/synchronized/<id>/recording.json --port 8929
"""
import argparse
from pathlib import Path
import re
import secrets
from types import SimpleNamespace
from urllib.parse import urlsplit

from .http_server import ConsoleHTTPServer, Handler
from .synchronized_recording import SyncRecording, SynchronizedSceneHost


class Library:
    def __init__(self, recording):
        self.identifier = recording.sha256[:32]
        self.host = SynchronizedSceneHost(recording)

    def get(self, identifier):
        if identifier != self.identifier:
            raise ValueError("시작할 때 지정한 기록만 열 수 있습니다.")
        return self.host

    def snapshot(self):
        recording = self.host.recording
        return dict(capture=dict(status="unavailable", id=None, can_start=False,
            reason="읽기 전용 기록 서버입니다.", pose_count=0,image_count=0,duration_sec=0.),
            recordings=[dict(id=self.identifier,label=recording.label,
                duration_sec=recording.duration,ended_reason=recording.ended_reason)])


class ReadonlyHandler(Handler):
    def do_GET(self):
        path = urlsplit(self.path).path
        if path == "/":
            if not self.host_valid():return self.respond(dict(error="허용되지 않은 주소입니다."),403)
            self.send_response(303)
            self.send_header("Location","/synchronized?id="+self.server.application.sync_recordings.identifier)
            self.send_header("Content-Length","0")
            self.end_headers()
            return
        elif (not path.startswith("/api/sync-recordings") and path not in (
                "/synchronized", "/synchronized.css", "/synchronized.js", "/viewer.js", "/viewer_contract.mjs",
                "/vendor/three.module.js", "/vendor/three.core.js", "/vendor/OrbitControls.js")):
            return self.respond(dict(error="이 서버는 선택한 기록 재생만 지원합니다."),404)
        return super().do_GET()

    def do_POST(self):
        path = urlsplit(self.path).path
        if not re.fullmatch(r"/api/sync-recordings/[0-9a-f]{32}/control",path):
            return self.respond(dict(error="이 서버는 재생 조작만 지원합니다."),404)
        return super().do_POST()


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--recording",type=Path,required=True)
    parser.add_argument("--port",type=int,default=8929)
    args=parser.parse_args()
    if not 1024<=args.port<=65535:
        parser.error("Port must be 1024..65535")
    library=Library(SyncRecording.load(args.recording))
    server=ConsoleHTTPServer(("127.0.0.1",args.port),ReadonlyHandler)
    server.application=SimpleNamespace(sync_recordings=library,readonly_recording=True)
    server.token=secrets.token_urlsafe(32)
    print(f"SYNCHRONIZED_RECORDING_READY http://127.0.0.1:{args.port}/synchronized?id={library.identifier}",flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()
        library.host.close()
    return 0


if __name__=="__main__":raise SystemExit(main())
