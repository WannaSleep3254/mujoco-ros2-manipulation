"""Validate the exchange schema and the publication's local evidence."""
import hashlib
from importlib.resources import files
import json
import math
from pathlib import Path, PurePosixPath

from . import FORMAT_VERSION


class ReportError(ValueError):
    pass


def sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def json_text(value):
    return json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False) + "\n"


def _pairs(pairs):
    value = {}
    for key, item in pairs:
        if key in value:
            raise ReportError(f"Duplicate JSON key: {key}")
        value[key] = item
    return value


def read_json(path):
    def constant(value):
        raise ReportError(f"Non-finite JSON number: {value}")
    try:
        return json.loads(Path(path).read_text(encoding="utf-8"),
                          object_pairs_hook=_pairs, parse_constant=constant)
    except (OSError, UnicodeError, json.JSONDecodeError) as error:
        raise ReportError(f"Cannot read {path}: {error}") from error


def schema():
    return json.loads(files(__package__).joinpath("report.schema.json").read_text(encoding="utf-8"))


def safe_asset(directory, name):
    parts = PurePosixPath(name)
    if (parts.is_absolute() or ".." in parts.parts or "\\" in name
            or parts.as_posix() != name or not parts.parts
            or any(p.startswith(".") for p in parts.parts)
            or any(c in name for c in "?#:%")):
        raise ReportError(f"Asset must be a plain relative publication path: {name}")
    current = Path(directory)
    for part in parts.parts:
        current = current / part
        if current.is_symlink():
            raise ReportError(f"Symlink assets are not published: {name}")
    if not current.is_file():
        raise ReportError(f"Missing asset: {name}")
    return current


def _unique(items, field, description):
    keys = [item[field] for item in items]
    if len(keys) != len(set(keys)):
        raise ReportError(f"Duplicate {description}")


def validate(data, directory, *, check_assets=True):
    try:
        from jsonschema import Draft202012Validator, FormatChecker
    except ImportError as error:
        raise ReportError("Install reporting dependencies: pip install '.[reports]' (jsonschema >=4.18)") from error
    definition = schema()
    Draft202012Validator.check_schema(definition)
    errors = sorted(Draft202012Validator(definition, format_checker=FormatChecker()).iter_errors(data),
                    key=lambda error: str(list(error.absolute_path)))
    if errors:
        error = errors[0]
        raise ReportError(f"Schema {list(error.absolute_path)}: {error.message}")
    if data["schema_version"] != FORMAT_VERSION:
        raise ReportError("Unsupported report version")
    for key in ("sample_groups", "stages", "metrics"):
        _unique(data[key], "id", key + " ID")
    _unique(data["assets"], "path", "asset path")
    groups = {g["id"]: g for g in data["sample_groups"]}
    for group in groups.values():
        n = group["observations"]
        if group["unique_cases"] > n:
            raise ReportError("Unique cases exceed observations")
        success, failure = group["successes"], group["failures"]
        if (success is None) != (failure is None):
            raise ReportError("Success and failure counts must both be known or null")
        if success is not None and success + failure != n:
            raise ReportError("Success + failure counts must include every observation")
        if group["kind"] == "unmeasured" and n != 0:
            raise ReportError("Unmeasured groups cannot contain observations")
    for metric in data["metrics"]:
        _unique(metric["values"], "group_id", "metric condition")
        is_time = metric["unit"] in ("s", "ms", "us")
        if is_time == (metric["clock"] == "not_applicable"):
            raise ReportError("Time units require wall/simulation; other units require not_applicable")
        for item in metric["values"]:
            if item["group_id"] not in groups:
                raise ReportError("Metric refers to an unknown sample group")
            group = groups[item["group_id"]]
            value, n, statistic = item["value"], item["sample_count"], item["statistic"]
            if n > group["observations"]:
                raise ReportError("Metric sample count exceeds its group observations")
            if value is None:
                if n != 0 or statistic != "not_measured":
                    raise ReportError("Missing measurements require null / n=0 / not_measured")
            elif not math.isfinite(value) or n < 1 or statistic == "not_measured":
                raise ReportError("Measured values require a finite number and samples")
            elif statistic == "single" and n != 1:
                raise ReportError("Single observations require n=1")
            elif statistic in ("p95", "p99") and n < 20:
                raise ReportError("Tail quantiles require at least 20 observations; this is no confidence guarantee")
    reserved = {"report.json", "index.html", "metrics.csv", "report-manifest.json"}
    for asset in data["assets"]:
        if asset["path"].split("/")[0] in reserved:
            raise ReportError("Asset conflicts with generated publication files")
        if check_assets:
            path = safe_asset(directory, asset["path"])
            if sha256(path) != asset["sha256"]:
                raise ReportError(f"Asset hash mismatch: {asset['path']}")
            if asset["role"] == "figure" and path.suffix.lower() not in (".png", ".jpg", ".jpeg", ".webp"):
                raise ReportError("Figure assets must use a raster image format")
    return data


def load_report(path):
    path = Path(path)
    if path.is_symlink() or path.parent.is_symlink():
        raise ReportError("Report sources cannot be symlinks")
    data = validate(read_json(path), path.parent)
    if path.parent.name != data["report_id"]:
        raise ReportError("Report directory must match report_id")
    return data
