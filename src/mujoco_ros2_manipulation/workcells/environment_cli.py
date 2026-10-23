"""Compose a new local environment overlay without rebuilding the active base."""
import argparse
import json
from pathlib import Path

from .config import load_workcell
from .environment_assets import require, sha
from .environment_overlay import _json, _read, build_overlay


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--workcell', default='fr5_ur5e', help='Existing checked base workcell')
    parser.add_argument('--assets', type=Path, required=True, help='Explicit local installation plan JSON')
    parser.add_argument('--output', type=Path, required=True, help='New immutable private output directory')
    parser.add_argument('--overlay-id', required=True, help='New ID, distinct from the active base')
    args = parser.parse_args()
    plan_path = args.assets.resolve()
    plan = _json(_read(plan_path, 64 * 1024))
    require(type(plan) is dict and set(plan) == {'schema_version', 'assets'}
        and type(plan['schema_version']) is int and plan['schema_version'] == 1,
        'Expected installation plan schema version 1')
    require(type(plan['assets']) is list and 1 <= len(plan['assets']) <= 32,
        'Expected between 1 and 32 explicit installed assets')
    assets = []
    for item in plan['assets']:
        require(type(item) is dict and set(item) == {'bundle', 'manifest_sha256', 'installation'}
            and isinstance(item['bundle'], str) and 0 < len(item['bundle']) <= 4096,
            'Invalid installation plan asset')
        path = Path(item['bundle'])
        path = path if path.is_absolute() else plan_path.parent / path
        assets.append(dict(item, bundle=path.resolve()))
    cell = load_workcell(args.workcell)
    result = build_overlay(cell, assets, args.output, overlay_id=args.overlay_id)
    output = args.output.resolve()
    print(json.dumps(dict(directory=str(output), manifest_sha256=sha((output / 'overlay.json').read_bytes()),
        id=result['id'], geometry_check=result['geometry_check'],
        use='Read-only model preview; ROS execution is not enabled'), allow_nan=False))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
