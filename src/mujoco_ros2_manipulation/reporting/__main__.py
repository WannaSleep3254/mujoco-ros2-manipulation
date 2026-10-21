"""manipulation-report validate | build [--check] | site --output PATH."""
import argparse
from pathlib import Path

from .data import ReportError
from .publish import build, collect, export_site


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--reports-dir", type=Path, help="Default: workspace/docs/reports")
    commands = parser.add_subparsers(dest="command", required=True)
    commands.add_parser("validate", help="Validate schema, sample semantics and evidence hashes")
    command = commands.add_parser("build", help="Generate report HTML/CSV/manifests and catalog")
    command.add_argument("--check", action="store_true", help="Fail on stale files without writing")
    command = commands.add_parser("site", help="Export only validated publication files")
    command.add_argument("--output", type=Path, required=True, help="New output directory")
    args = parser.parse_args(argv)
    try:
        root = args.reports_dir
        if root is None:
            from ..paths import workspace_root
            root = workspace_root() / "docs/reports"
        if args.command == "validate":
            _, site = collect(root)
            print(f"Report schema, samples and evidence hashes passed ({len(site)} publication files)")
        elif args.command == "build":
            count = build(root, check=args.check)
            print(f"{count} generated report files {'match sources' if args.check else 'built'}")
        else:
            count = export_site(root, args.output)
            print(f"Site exported: {args.output} ({count} allowlisted files)")
    except (ReportError, OSError) as error:
        parser.exit(1, f"Report error: {error}\n")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
