"""Run the existing Three.js renderer from a recording, without ROS or MuJoCo.

python -m mujoco_ros2_manipulation.operator_console.recorded_server --demo
Only fixed local assets and the explicitly selected recording are served.
"""
import argparse
import hmac
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from importlib.resources import files
import json
import re
import secrets
from urllib.parse import parse_qs, urlsplit

from .recording import Recording, RecordedSceneHost, demo_recording


class RecordingHandler(BaseHTTPRequestHandler):
    def log_message(self, *_):
        pass

    def respond(self, value, status=200, mime="application/json; charset=utf-8"):
        raw = value if isinstance(value, bytes) else (json.dumps(value, ensure_ascii=False, allow_nan=False) + "\n").encode()
        self.send_response(status)
        self.send_header("Content-Type", mime)
        self.send_header("Content-Length", str(len(raw)))
        self.send_header("Cache-Control", "no-store")
        self.send_header("X-Content-Type-Options", "nosniff")
        self.send_header("X-Frame-Options", "DENY")
        self.send_header("Referrer-Policy", "no-referrer")
        self.send_header("Content-Security-Policy", "default-src 'self'; script-src 'self'; style-src 'self'; frame-ancestors 'none'; base-uri 'none'")
        self.end_headers()
        try:
            self.wfile.write(raw)
        except (BrokenPipeError, ConnectionResetError):
            pass

    def valid_host(self):
        port = self.server.server_address[1]
        return self.headers.get("Host") in (f"127.0.0.1:{port}", f"localhost:{port}")

    def do_GET(self):
        if not self.valid_host():
            return self.respond(dict(error="허용되지 않은 주소입니다."), 403)
        host, parsed = self.server.source, urlsplit(self.path)
        try:
            if parsed.path == "/api/recording":
                return self.respond(dict(view=host.snapshot(), token=self.server.token))
            if parsed.path in ("/api/view/manifest", "/api/view/pose"):
                query = parse_qs(parsed.query, keep_blank_values=True)
                if set(query) != {"binding_id"} or len(query["binding_id"]) != 1:
                    raise ValueError("현재 장면 식별자가 필요합니다.")
                identifier = query["binding_id"][0]
                value = host.manifest_bytes(identifier) if parsed.path.endswith("manifest") else host.pose_envelope(identifier)
                return self.respond(value)
            if parsed.path.startswith("/api/view/assets/"):
                digest = parsed.path.removeprefix("/api/view/assets/")
                if not re.fullmatch("[0-9a-f]{64}", digest):
                    raise ValueError("잘못된 자산 식별자입니다.")
                return self.respond(host.asset_bytes(digest), mime="application/octet-stream")
            assets = {"/": "recording.html", "/recording.js": "recording.js", "/style.css": "style.css", "/recording.css": "recording.css",
                "/viewer.js": "viewer.js", "/viewer_contract.mjs": "viewer_contract.mjs",
                "/vendor/three.module.js": "vendor/three.module.js", "/vendor/three.core.js": "vendor/three.core.js",
                "/vendor/OrbitControls.js": "vendor/OrbitControls.js"}
            if parsed.path in assets:
                name = assets[parsed.path]
                raw = files(__package__).joinpath("static", name).read_bytes()
                # Vendored imports use the same local Three.js module, without
                # inline import maps or a CDN requirement in this standalone page.
                if name == "viewer.js":
                    raw = raw.replace(b"from 'three'", b"from './vendor/three.module.js'")
                elif name == "vendor/OrbitControls.js":
                    raw = raw.replace(b"from 'three'", b"from './three.module.js'")
                mime = "text/javascript" if name.endswith((".js", ".mjs")) else "text/css" if name.endswith(".css") else "text/html"
                return self.respond(raw, mime=mime + "; charset=utf-8")
            return self.respond(dict(error="자료를 찾을 수 없습니다."), 404)
        except (ValueError, KeyError, OSError) as error:
            return self.respond(dict(error=str(error)), 409)

    def do_POST(self):
        port = self.server.server_address[1]
        if (not self.valid_host() or self.headers.get("Origin") not in (f"http://127.0.0.1:{port}", f"http://localhost:{port}")
                or not hmac.compare_digest(self.headers.get("X-Operator-Token", ""), self.server.token)):
            return self.respond(dict(error="이 화면의 재생 요청만 허용합니다."), 403)
        if urlsplit(self.path).path != "/api/recording/control":
            return self.respond(dict(error="기록 뷰어는 로봇 제어 요청을 지원하지 않습니다."), 404)
        try:
            length = int(self.headers.get("Content-Length", "0"))
            if not 0 < length <= 1024 or self.headers.get_content_type() != "application/json" or self.headers.get("Transfer-Encoding"):
                raise ValueError("작은 JSON 재생 요청이 필요합니다.")
            from .scene_host import _json
            value = _json(self.rfile.read(length))
            if type(value) is not dict or set(value) not in ({"action"}, {"action", "time_sec"}):
                raise ValueError("재생 동작과 선택한 시각만 받습니다.")
            return self.respond(self.server.source.control(value["action"], value.get("time_sec")))
        except (ValueError, TypeError, KeyError, UnicodeError) as error:
            return self.respond(dict(error=str(error)), 400)


def serve(source, port):
    server = ThreadingHTTPServer(("127.0.0.1", port), RecordingHandler)
    server.daemon_threads = True
    server.source, server.token = source, secrets.token_urlsafe(32)
    return server


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument("--recording", help="Explicit local display-recording-v1 JSON and adjacent assets/")
    group.add_argument("--demo", action="store_true", help="Analytic two-link fixture; no manufacturer/physics model")
    parser.add_argument("--port", type=int, default=8896)
    args = parser.parse_args()
    if not 1024 <= args.port <= 65535:
        parser.error("Port must be 1024..65535")
    source = RecordedSceneHost(demo_recording() if args.demo else Recording.load(args.recording))
    server = serve(source, args.port)
    print(f"READONLY_RECORDING_READY http://127.0.0.1:{args.port}/", flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()
        source.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
