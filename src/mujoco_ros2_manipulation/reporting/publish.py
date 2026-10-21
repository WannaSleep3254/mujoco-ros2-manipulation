"""Deterministic generated files and an allowlisted, fresh-directory site export."""
import hashlib
from html.parser import HTMLParser
from importlib.resources import files
import os
from pathlib import Path
import posixpath
import shutil
import tempfile
from urllib.parse import unquote, urlsplit

from . import GENERATOR_VERSION
from .data import ReportError, json_text, load_report
from .render import catalog_entry, catalog_html, metrics_csv, report_html

GENERATOR_FILES = ("__init__.py", "data.py", "render.py", "publish.py", "report.schema.json",
                   "templates/page.html", "templates/style.css")


class _Links(HTMLParser):
    def __init__(self):
        super().__init__()
        self.ids, self.links = set(), []

    def handle_starttag(self, tag, attrs):
        attrs = dict(attrs)
        if "id" in attrs:
            if attrs["id"] in self.ids:
                raise ReportError("Duplicate HTML anchor")
            self.ids.add(attrs["id"])
        for key in ("href", "src", "poster"):
            if key in attrs:
                self.links.append((key, attrs[key]))


def audit_site(site):
    """Resolve local links against the complete exported set, including legacy HTML."""
    pages = {}
    for name, content in site.items():
        if name.endswith(".html"):
            page = _Links()
            page.feed(content.decode("utf-8"))
            pages[name] = page
    for name, page in pages.items():
        for attribute, url in page.links:
            parsed = urlsplit(url)
            if parsed.scheme == "data":
                continue
            if parsed.scheme == "https" and attribute == "href":
                continue
            if parsed.scheme or parsed.netloc or parsed.path.startswith("/") or "\\" in parsed.path:
                raise ReportError(f"Nonportable HTML resource: {name}: {url}")
            target = posixpath.normpath(posixpath.join(posixpath.dirname(name), unquote(parsed.path))) if parsed.path else name
            if target not in site:
                raise ReportError(f"Broken HTML link: {name}: {url}")
            if parsed.fragment and target in pages and unquote(parsed.fragment) not in pages[target].ids:
                raise ReportError(f"Broken HTML anchor: {name}: {url}")


def _metadata(content):
    return {"bytes": len(content), "sha256": hashlib.sha256(content).hexdigest()}


def generator_metadata():
    resources = files(__package__)
    inputs = {name: _metadata(resources.joinpath(name).read_bytes()) for name in GENERATOR_FILES}
    return {"version": GENERATOR_VERSION,
            "fingerprint": hashlib.sha256(json_text(inputs).encode()).hexdigest(), "files": inputs}


def collect(root):
    root = Path(root)
    if root.is_symlink():
        raise ReportError("Reports directory cannot be a symlink")
    paths = sorted(root.glob("*/report.json"))
    if not paths:
        raise ReportError("No */report.json sources found")
    reports = [(path, load_report(path)) for path in paths]
    by_id = {data["report_id"]: data for _, data in reports}
    if len(by_id) != len(reports):
        raise ReportError("Duplicate report IDs")
    for data in by_id.values():
        seen = {data["report_id"]}
        current = data
        while "supersedes" in current:
            previous = current["supersedes"]
            if previous not in by_id or previous in seen:
                raise ReportError("Missing or cyclic supersedes reference")
            seen.add(previous)
            current = by_id[previous]
    generator = generator_metadata()
    generated, site = {}, {}
    for path, data in reports:
        prefix = data["report_id"] + "/"
        content = {"report.json": path.read_bytes(),
                   "index.html": report_html(data).encode(),
                   "metrics.csv": metrics_csv(data).encode()}
        for asset in data["assets"]:
            raw = (path.parent / asset["path"]).read_bytes()
            if _metadata(raw)["sha256"] != asset["sha256"]:
                raise ReportError("Evidence changed during publication")
            content[asset["path"]] = raw
        manifest = {"schema_version": 1, "report_id": data["report_id"],
                    "source_commit": data["provenance"]["commit"], "generator": generator,
                    "files": {name: _metadata(raw) for name, raw in sorted(content.items())}}
        content["report-manifest.json"] = json_text(manifest).encode()
        site.update({prefix + name: raw for name, raw in content.items()})
        generated.update({prefix + name: content[name] for name in ("index.html", "metrics.csv", "report-manifest.json")})
    entries = [catalog_entry(data) for _, data in reports]
    entries.sort(key=lambda entry: (entry["published_date"], entry["report_id"]), reverse=True)
    generated["catalog.json"] = json_text({"schema_version": 1, "reports": entries}).encode()
    generated["index.html"] = catalog_html(entries).encode()
    site.update({name: generated[name] for name in ("catalog.json", "index.html")})
    generated["site-manifest.json"] = json_text({
        "schema_version": 1, "generator": generator,
        "files": {name: _metadata(raw) for name, raw in sorted(site.items())},
    }).encode()
    site["site-manifest.json"] = generated["site-manifest.json"]
    audit_site(site)
    return generated, site


def build(root, *, check=False):
    root = Path(root)
    generated, _ = collect(root)
    stale = []
    for name, raw in generated.items():
        path = root / name
        if path.is_symlink():
            raise ReportError(f"Generated output cannot be a symlink: {name}")
        if not path.exists() or path.read_bytes() != raw:
            stale.append(name)
    if check and stale:
        raise ReportError("Generated report files are stale; run build:\n" + "\n".join(stale))
    if not check:
        for name in stale:
            path = root / name
            with tempfile.NamedTemporaryFile(dir=path.parent, delete=False) as stream:
                temporary = Path(stream.name)
                stream.write(generated[name])
            try:
                os.replace(temporary, path)
            finally:
                temporary.unlink(missing_ok=True)
    return len(generated)


def export_site(root, output):
    root, output = Path(root), Path(output)
    build(root, check=True)
    if output.exists() or output.is_symlink():
        raise ReportError("Site output must be a new directory; existing content is never deleted")
    source, destination = root.resolve(), output.resolve()
    if destination.is_relative_to(source) or source.is_relative_to(destination):
        raise ReportError("Site output must be separate from reports sources")
    _, site = collect(root)
    output.parent.mkdir(parents=True, exist_ok=True)
    staging = Path(tempfile.mkdtemp(prefix=".report-site-", dir=output.parent))
    try:
        for name, raw in site.items():
            path = staging / name
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_bytes(raw)
        os.rename(staging, output)
    finally:
        if staging.exists():
            shutil.rmtree(staging)
    return len(site)
