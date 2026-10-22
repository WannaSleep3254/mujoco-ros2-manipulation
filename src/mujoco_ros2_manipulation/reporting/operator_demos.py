"""Publish explicitly registered functional demos separately from measured reports."""
import hashlib
import json
from importlib.resources import files
from pathlib import Path
import re

from .data import ReportError, read_json, safe_asset


REGISTRY = "operator-demos.json"


def collect_demos(root):
    """Return catalog entries and reviewed bytes; never discover arbitrary folders."""
    root = Path(root)
    registry = root / REGISTRY
    if registry.is_symlink():
        raise ReportError("Demo registry cannot be a symlink")
    if not registry.exists():
        return [], {}
    raw_registry = registry.read_bytes()
    data = read_json(registry)
    from jsonschema import Draft202012Validator, FormatChecker
    definition = json.loads(files(__package__).joinpath("operator-demos.schema.json").read_text(encoding="utf-8"))
    Draft202012Validator.check_schema(definition)
    errors = list(Draft202012Validator(definition, format_checker=FormatChecker()).iter_errors(data))
    if errors:
        raise ReportError(f"Demo registry schema: {errors[0].message}")
    if registry.read_bytes() != raw_registry:
        raise ReportError("Demo registry changed during publication")
    entries, site, seen = [], {REGISTRY: raw_registry}, set()
    operator = root.parent / "operator"
    if operator.is_symlink():
        raise ReportError("Demo parent directory cannot be a symlink")
    for entry in data["demos"]:
        identifier = entry["id"]
        if not re.fullmatch(r"[a-z0-9]+(?:-[a-z0-9]+)*", identifier):
            raise ReportError("Demo registry schema: invalid demo ID")
        if identifier in seen:
            raise ReportError("Duplicate demo IDs")
        seen.add(identifier)
        directory = operator / identifier
        if directory.is_symlink():
            raise ReportError("Demo directory cannot be a symlink")
        manifest_path = safe_asset(directory, "manifest.json")
        raw_manifest = manifest_path.read_bytes()
        if hashlib.sha256(raw_manifest).hexdigest() != entry["manifest_sha256"]:
            raise ReportError(f"Demo manifest hash mismatch: {identifier}")
        manifest = read_json(manifest_path)
        if manifest_path.read_bytes() != raw_manifest:
            raise ReportError("Demo manifest changed during publication")
        if not isinstance(manifest, dict) or type(manifest.get("schema_version")) is not int or manifest["schema_version"] != 1:
            raise ReportError("Unsupported demo manifest version")
        assets = manifest.get("assets")
        if not isinstance(assets, dict) or "index.html" not in assets or "manifest.json" in assets:
            raise ReportError("Demo manifest needs index.html and cannot claim manifest.json")
        prefix = "operator/" + identifier + "/"
        site[prefix + "manifest.json"] = raw_manifest
        for name, digest in assets.items():
            if not isinstance(digest, str) or not re.fullmatch(r"[a-f0-9]{64}", digest):
                raise ReportError(f"Invalid demo asset digest: {name}")
            content = safe_asset(directory, name).read_bytes()
            if hashlib.sha256(content).hexdigest() != digest:
                raise ReportError(f"Demo asset hash mismatch: {identifier}/{name}")
            site[prefix + name] = content
        entries.append({key: entry[key] for key in ("id", "title", "published_date", "overview")}
                       | {"url": prefix + "index.html"})
    entries.sort(key=lambda entry: (entry["published_date"], entry["id"]), reverse=True)
    return entries, site

