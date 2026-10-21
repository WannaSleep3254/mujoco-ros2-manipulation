"""Verify the deployed static site against the checked-in publication manifest."""
import argparse
import hashlib
import json
from pathlib import Path, PurePosixPath
import re
import time
from urllib.error import URLError
from urllib.parse import quote, urljoin, urlsplit
from urllib.request import Request, urlopen


class PublishedReportError(ValueError):
    pass


def fetch_bytes(url, limit):
    request = Request(url, headers={"Accept-Encoding": "identity", "Cache-Control": "no-cache"})
    try:
        with urlopen(request, timeout=15) as response:
            return response.read(limit)
    except (OSError, URLError) as error:
        raise PublishedReportError(f"Cannot read {url}: {error}") from error


def expected_files(manifest):
    raw = Path(manifest).read_bytes()
    data = json.loads(raw)
    if not isinstance(data, dict) or data.get("schema_version") != 1 or not isinstance(data.get("files"), dict) or not data["files"]:
        raise PublishedReportError("Unsupported or empty site manifest")
    records = {"site-manifest.json": {"bytes": len(raw), "sha256": hashlib.sha256(raw).hexdigest()}}
    for name, metadata in data["files"].items():
        path = PurePosixPath(name)
        if (not path.parts or path.is_absolute() or ".." in path.parts or path.as_posix() != name
                or any(p.startswith(".") for p in path.parts) or any(c in name for c in "\\?#:%")
                or name == "site-manifest.json"):
            raise PublishedReportError(f"Invalid publication path: {name}")
        if (not isinstance(metadata, dict) or type(metadata.get("bytes")) is not int
                or metadata["bytes"] < 0 or not re.fullmatch(r"[0-9a-f]{64}", str(metadata.get("sha256")))):
            raise PublishedReportError(f"Invalid file metadata: {name}")
        records[name] = metadata
    return records


def verify_site(base_url, manifest, *, fetch=None):
    parsed = urlsplit(base_url)
    if (parsed.scheme not in ("https", "http") or not parsed.hostname or parsed.username
            or parsed.password or parsed.query or parsed.fragment):
        raise PublishedReportError("Supply an HTTP(S) site root without credentials, query or fragment")
    root = base_url.rstrip("/") + "/"
    records = expected_files(manifest)
    fetch = fetch or fetch_bytes
    total = 0
    for name, metadata in records.items():
        content = fetch(urljoin(root, quote(name, safe="/")), metadata["bytes"] + 1)
        if len(content) != metadata["bytes"] or hashlib.sha256(content).hexdigest() != metadata["sha256"]:
            raise PublishedReportError(f"Published content does not match this commit: {name}")
        total += len(content)
    return dict(url=root, files=len(records), bytes=total,
                manifest_sha256=records["site-manifest.json"]["sha256"])


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--url", required=True)
    parser.add_argument("--manifest", type=Path,
        default=Path(__file__).resolve().parents[1] / "docs/reports/site-manifest.json")
    parser.add_argument("--attempts", type=int, default=1)
    parser.add_argument("--delay", type=float, default=5)
    args = parser.parse_args(argv)
    if not 1 <= args.attempts <= 30 or not 0 <= args.delay <= 10:
        parser.error("attempts must be 1..30; delay must be 0..10 seconds")
    for attempt in range(1, args.attempts + 1):
        try:
            result = verify_site(args.url, args.manifest)
            print(json.dumps(dict(**result, attempt=attempt), ensure_ascii=False))
            return 0
        except (PublishedReportError, OSError, json.JSONDecodeError) as error:
            if attempt == args.attempts:
                parser.exit(1, f"Published report verification failed: {error}\n")
            print(f"Publication not ready ({attempt}/{args.attempts}): {error}", flush=True)
            time.sleep(args.delay)


if __name__ == "__main__":
    raise SystemExit(main())
