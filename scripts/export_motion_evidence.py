"""Project one local JSON/gzip record into a new privacy-filtered evidence file."""
import argparse
import getpass
import gzip
from pathlib import Path
import socket
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
from mujoco_ros2_manipulation.workcells.evidence_export import encoded_evidence, export_evidence


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", type=Path, required=True)
    parser.add_argument("--kind", choices=("session", "trace", "setup", "result", "physics", "verification"), required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if args.output.exists():
        parser.error("Output already exists; original evidence must not be overwritten")
    raw = args.source.read_bytes()
    if args.source.suffix == ".gz":
        raw = gzip.decompress(raw)
    exported = export_evidence(raw, args.kind, forbidden_tokens=(getpass.getuser(), socket.gethostname()))
    output = encoded_evidence(exported)
    if args.output.suffix == ".gz":
        output = gzip.compress(output, mtime=0)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open("xb") as stream:
        stream.write(output)
    print("Wrote one privacy-filtered extract; complete source left unchanged")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
