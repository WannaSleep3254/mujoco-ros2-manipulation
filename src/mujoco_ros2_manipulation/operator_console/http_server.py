"""Loopback HTTP API with bounded JSON, exact origins and fixed file routes."""
import hmac
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from importlib.resources import files
import json
import mimetypes
from pathlib import Path
from urllib.parse import unquote, urlsplit

from ..workcells.motion_request import decode_message, command, validate_command


class ConsoleHTTPServer(ThreadingHTTPServer):
    daemon_threads = True
    allow_reuse_address = False


class Handler(BaseHTTPRequestHandler):
    server_version = "MuJoCoOperator/1"

    def log_message(self, *_):
        pass

    def respond(self, data, status=200, mime="application/json; charset=utf-8"):
        if not isinstance(data, bytes):
            data = (json.dumps(data, ensure_ascii=False, allow_nan=False)+"\n").encode()
        self.send_response(status)
        self.send_header("Content-Type", mime)
        self.send_header("Content-Length", str(len(data)))
        self.send_header("Cache-Control", "no-store")
        self.send_header("X-Content-Type-Options", "nosniff")
        self.send_header("Referrer-Policy", "no-referrer")
        self.send_header("X-Frame-Options", "DENY")
        if not self.path.startswith("/reports/"):
            self.send_header("Content-Security-Policy", "default-src 'self'; img-src 'self' data:; style-src 'self'; script-src 'self'; connect-src 'self'; frame-ancestors 'none'; base-uri 'none'")
        self.end_headers()
        try:
            self.wfile.write(data)
        except (BrokenPipeError, ConnectionResetError):
            pass

    def host_valid(self):
        port = self.server.server_address[1]
        return self.headers.get("Host") in (f"127.0.0.1:{port}", f"localhost:{port}")

    def do_GET(self):
        if not self.host_valid():
            return self.respond(dict(error="허용되지 않은 주소입니다."), 403)
        path = urlsplit(self.path).path
        app = self.server.application
        try:
            if path == "/api/bootstrap":
                catalog = json.loads((app.cell.root/"docs/reports/catalog.json").read_text())
                return self.respond(dict(config=app.state.config, reports=catalog["reports"], token=self.server.token))
            if path == "/api/state":
                return self.respond(app.snapshot())
            if path.startswith("/api/requests/"):
                return self.respond(app.request_details(path.removeprefix("/api/requests/")))
            if path == "/api/frame":
                image = app.directory/"frame.jpg"
                return self.respond(image.read_bytes(), mime="image/jpeg") if image.is_file() else self.respond(b"", 204)
            if path == "/api/logs":
                logs = []
                for stream in app.files[-4:]:
                    source = Path(stream.name)
                    with source.open("rb") as reader:
                        reader.seek(max(0, source.stat().st_size-12000))
                        logs.append(dict(name=source.name, text=reader.read().decode(errors="replace")))
                return self.respond(dict(logs=logs))
            assets = {"/":"index.html", "/index.html":"index.html", "/style.css":"style.css", "/app.js":"app.js"}
            if path in assets:
                name = assets[path]
                return self.respond(files(__package__).joinpath("static", name).read_bytes(), mime=mimetypes.guess_type(name)[0]+"; charset=utf-8")
            if path.startswith("/reports/"):
                relative = unquote(path.removeprefix("/reports/")) or "index.html"
                if relative.endswith("/"):
                    relative += "index.html"
                folder = (app.cell.root/"docs/reports").resolve()
                allowlist = set(json.loads((folder/"site-manifest.json").read_text())["files"]) | {"index.html", "catalog.json", "site-manifest.json"}
                source = (folder/relative).resolve()
                if relative in allowlist and source.is_relative_to(folder) and source.is_file():
                    return self.respond(source.read_bytes(), mime=mimetypes.guess_type(source.name)[0] or "application/octet-stream")
            return self.respond(dict(error="자료를 찾을 수 없습니다."), 404)
        except (ValueError, OSError, KeyError) as error:
            return self.respond(dict(error=str(error)), 400)

    def do_POST(self):
        port = self.server.server_address[1]
        if (not self.host_valid() or self.headers.get("Origin") not in (f"http://127.0.0.1:{port}", f"http://localhost:{port}")
                or not hmac.compare_digest(self.headers.get("X-Operator-Token", ""), self.server.token)):
            return self.respond(dict(error="이 조작 화면에서 보낸 요청만 허용합니다."), 403)
        try:
            if self.headers.get_content_type() != "application/json" or self.headers.get("Transfer-Encoding"):
                raise ValueError("JSON 요청이 필요합니다.")
            length = int(self.headers.get("Content-Length", "0"))
            if not 0 < length <= 65536:
                return self.respond(dict(error="요청은 64KiB 이하여야 합니다."), 413)
            payload = decode_message(self.rfile.read(length).decode("utf-8"))
            path = urlsplit(self.path).path
            app = self.server.application
            if path == "/api/start":
                result = app.start(payload)
            elif path == "/api/attach":
                if payload:
                    raise ValueError("추가 인자가 필요하지 않습니다.")
                result = app.start(dict(native_gui=False, planning_case="none"), attach=True)
            elif path == "/api/stop":
                if payload:
                    raise ValueError("추가 인자가 필요하지 않습니다.")
                result = app.stop_async()
            elif path == "/api/command":
                result = app.submit(payload)
            elif path == "/api/target" and set(payload) == {"text"} and isinstance(payload["text"], str):
                target = decode_message(payload["text"])
                validate_command(command(target), app.state.config["joint_limits"])
                result = dict(target_rad=target)
            elif path == "/api/cancel" and set(payload) == {"request_id"}:
                result = app.cancel(payload["request_id"])
            elif path == "/api/camera" and set(payload) == {"view"}:
                result = app.camera(payload["view"])
            else:
                return self.respond(dict(error="지원하지 않는 요청입니다."), 404)
            self.respond(result, 202)
        except (ValueError, UnicodeError, TypeError, KeyError) as error:
            self.respond(dict(error=str(error)), 400)
        except RuntimeError as error:
            self.respond(dict(error=str(error)), 409)
