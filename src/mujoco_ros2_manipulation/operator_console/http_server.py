"""Loopback HTTP API with bounded JSON, exact origins and fixed file routes."""
import hmac
import base64
import re
import hashlib
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from importlib.resources import files
import json
import mimetypes
from pathlib import Path
from urllib.parse import parse_qs, unquote, urlsplit

from ..reporting.data import ReportError, safe_asset
from .catalog import describe_scenarios
from ..workcells.motion_request import decode_message, command, validate_command


def _report_asset(root, relative):
    """Map a reviewed publication URL to its source without broadening access."""
    docs = Path(root) / "docs"
    reports = docs / "reports"
    if docs.is_symlink() or reports.is_symlink():
        return None
    try:
        manifest = safe_asset(reports, "site-manifest.json")
        # Preserve local preview behavior: the manifest selects paths. Build and
        # deployment verification, rather than each GET, check asset digests.
        allowlist = set(json.loads(manifest.read_text())["files"]) | {"index.html", "catalog.json", "site-manifest.json"}
        if relative not in allowlist:
            return None
        # Keep measurement report IDs (including "operator") in their original
        # directory. The publisher rejects overlapping measurement/demo URLs.
        candidates = [(reports, relative)]
        if relative.startswith("operator/"):
            candidates.append((docs / "operator", relative.removeprefix("operator/")))
        for directory, name in candidates:
            if directory.is_symlink():
                continue
            try:
                return safe_asset(directory, name)
            except ReportError:
                continue
        return None
    except ReportError:
        return None


LOCAL_IMPORT_MAP = '{"imports":{"three":"/vendor/three.module.js"}}'
IMPORT_MAP_CSP_HASH = base64.b64encode(hashlib.sha256(LOCAL_IMPORT_MAP.encode()).digest()).decode('ascii')


def _view_binding_query(path):
    query = parse_qs(urlsplit(path).query, keep_blank_values=True)
    if set(query) != {'binding_id'} or len(query['binding_id']) != 1 or not re.fullmatch('[0-9a-f]{64}',query['binding_id'][0]):
        raise ValueError('현재 장면 식별자가 필요합니다.')
    return query['binding_id'][0]


class ConsoleHTTPServer(ThreadingHTTPServer):
    daemon_threads = True
    allow_reuse_address = True


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
            self.send_header("Content-Security-Policy", "default-src 'self'; img-src 'self' data:; style-src 'self'; script-src 'self' 'sha256-"+IMPORT_MAP_CSP_HASH+"'; connect-src 'self'; frame-ancestors 'none'; base-uri 'none'")
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
                return self.respond(dict(config=app.state.config, reports=catalog["reports"], demos=describe_scenarios(), token=self.server.token))
            if path == "/api/state":
                return self.respond(app.snapshot())
            if path == "/api/view/manifest":
                value = app.views.host.manifest_bytes(_view_binding_query(self.path)) if app.views.host else None
                return self.respond(b"",204) if value is None else self.respond(value,mime="application/json; charset=utf-8")
            if path == "/api/view/pose":
                value = app.views.host.pose_envelope(_view_binding_query(self.path)) if app.views.host else None
                return self.respond(b"",204) if value is None else self.respond(value)
            if path.startswith("/api/view/assets/"):
                digest = path.removeprefix("/api/view/assets/")
                if not re.fullmatch('[0-9a-f]{64}',digest):
                    raise ValueError('올바른 장면 자산 식별자가 필요합니다.')
                value = app.views.host.asset_bytes(digest) if app.views.host else None
                return self.respond(dict(error="현재 장면의 자산이 아닙니다."),404) if value is None else self.respond(value,mime="application/octet-stream")
            if path.startswith("/api/requests/"):
                return self.respond(app.request_details(path.removeprefix("/api/requests/")))
            if path == "/api/frame":
                # JPEG and metadata are separate atomic files. Require their digest
                # to agree and never answer an old scene identity with a new image.
                metadata = app.directory/"frame.json"
                image = app.directory/"frame.jpg"
                if not metadata.is_file() or not image.is_file():
                    return self.respond(b"", 204)
                before = metadata.read_bytes()
                frame = json.loads(before)
                query = parse_qs(urlsplit(self.path).query, keep_blank_values=True)
                for name in ("job_id", "run_id", "model_sha256", "sequence", "generation", "operator_session_id"):
                    if name in query and query[name] != [str(frame.get(name, ""))]:
                        return self.respond(b"", 204)
                pixels = image.read_bytes()
                if (metadata.read_bytes() != before or not frame.get("jpeg_sha256")
                        or hashlib.sha256(pixels).hexdigest() != frame["jpeg_sha256"]):
                    return self.respond(b"", 204)
                return self.respond(pixels, mime="image/jpeg")
            if path == "/api/logs":
                logs = []
                for stream in app.files[-4:]:
                    source = Path(stream.name)
                    with source.open("rb") as reader:
                        reader.seek(max(0, source.stat().st_size-12000))
                        logs.append(dict(name=source.name, text=reader.read().decode(errors="replace")))
                return self.respond(dict(logs=logs))
            assets = {"/":"index.html", "/index.html":"index.html", "/style.css":"style.css", "/app.js":"app.js",
                "/viewer.js":"viewer.js", "/viewer_contract.mjs":"viewer_contract.mjs", "/scene_app.js":"scene_app.js",
                "/vendor/three.module.js":"vendor/three.module.js", "/vendor/three.core.js":"vendor/three.core.js",
                "/vendor/OrbitControls.js":"vendor/OrbitControls.js", "/vendor/LICENSE":"vendor/LICENSE",
                "/vendor/vendor-manifest.json":"vendor/vendor-manifest.json"}
            if path in assets:
                name = assets[path]
                return self.respond(files(__package__).joinpath("static", name).read_bytes(), mime=("text/javascript" if name.endswith((".js",".mjs")) else mimetypes.guess_type(name)[0] or "text/plain")+"; charset=utf-8")
            if path.startswith("/reports/"):
                relative = unquote(path.removeprefix("/reports/")) or "index.html"
                if relative.endswith("/"):
                    relative += "index.html"
                source = _report_asset(app.cell.root, relative)
                if source is not None:
                    return self.respond(source.read_bytes(), mime=mimetypes.guess_type(source.name)[0] or "application/octet-stream")
            return self.respond(dict(error="자료를 찾을 수 없습니다."), 404)
        except RuntimeError as error:
            return self.respond(dict(error=str(error)), 409)
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
            if path == "/api/view/select":
                result = app.views.select(payload)
            elif path == "/api/view/plan" and set(payload) == {"request_id"}:
                result = app.views.select_plan(payload["request_id"])
            elif path == "/api/view/review" and set(payload) == {"request_id"}:
                result = app.views.select_review_plan(payload["request_id"])
            elif path == "/api/view/seek" and set(payload) == {"time_sec","binding_id"}:
                result = app.views.seek(payload["time_sec"],binding_id=payload["binding_id"])
            elif path == "/api/view/display" and set(payload) == {"mode"}:
                result = app.set_display(payload["mode"])
            elif path == "/api/demos/start":
                result = app.demos.start(payload)
            elif path == "/api/demos/stop" and set(payload) == {"job_id"}:
                result = app.demos.request_stop(payload["job_id"])
            elif path == "/api/start":
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
            elif path == "/api/plan":
                result = app.create_plan(payload)
            elif path == "/api/plan/execute":
                result = app.execute_plan(payload)
            elif path == "/api/plan/discard" and set(payload) == {"request_id"}:
                result = app.discard_plan(payload["request_id"])
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
