"""Check tracked Python, shell syntax, configuration and local Markdown links."""
import ast
from pathlib import Path
import re
import subprocess
import sys
from urllib.parse import unquote

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src'))
from mujoco_ros2_manipulation.build import build_plan, load_manifest
from mujoco_ros2_manipulation.profiles import list_profiles
from mujoco_ros2_manipulation.scenes import load_scene


def main():
    files = [ROOT / name for name in subprocess.check_output(
        ['git', '-C', str(ROOT), 'ls-files', '--cached', '--others', '--exclude-standard'], text=True).splitlines()]
    # Removed files remain in the index until git add.
    files = [path for path in files if path.is_file()]
    failures = []
    for path in files:
        if path.suffix == '.py':
            ast.parse(path.read_text(), filename=str(path))
        elif path.suffix == '.sh':
            subprocess.run(['bash', '-n', str(path)], check=True)
        elif path.suffix == '.md':
            for target in re.findall(r'\]\(([^\n)]+)\)', path.read_text()):
                target = target.strip('<>').split('#', 1)[0]
                if not target or re.match('[a-zA-Z][a-zA-Z0-9+.-]*:', target):
                    continue
                if not (path.parent / unquote(target)).exists():
                    failures.append(f'{path.relative_to(ROOT)}: missing link {target}')
    load_manifest()
    build_plan()
    from mujoco_ros2_manipulation.workcells.config import load_workcell
    for path in (ROOT / 'config/workcells').glob('*.yaml'):
        load_workcell(path.stem)
    from mujoco_ros2_manipulation.workcells.interference import load_case
    import yaml
    for path in (ROOT / 'config/scenarios').glob('*.yaml'):
        case = yaml.safe_load(path.read_text())
        load_case(load_workcell(case['workcell']), path.stem)
    profiles = list_profiles()
    for path in (ROOT / 'config/scenes').glob('*.yaml'):
        load_scene(path.stem, ROOT)
    if failures:
        raise SystemExit('\n'.join(failures))
    print(f'Syntax, local links, build manifest, {len(profiles)} robot profiles and scenes passed')


if __name__ == '__main__':
    main()
