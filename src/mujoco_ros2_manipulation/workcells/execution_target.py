"""Startup-only, immutable selection of a checked environment execution target.

The base Workcell remains the robot/controller/policy contract. An overlay never
pretends to be a Workcell ``source.json`` build. Selecting a display source does
not construct this object, and no command or browser payload supplies its paths.
"""
from dataclasses import dataclass, field
import json
from pathlib import Path
import re

from .environment_assets import canonical, sha
from .environment_overlay import check_overlay_identity


def _preflight(directory, base, manifest_sha256, report):
    # Reuse the existing owned, bounded CPU worker/nonce/receipt protocol. This
    # import is startup-only; neither this module nor the checker imports ROS.
    from ..operator_console.environment_sources import _full_check
    python = base.root / ".venv/bin/python"
    if not python.is_file():
        raise ValueError("The local environment checking Python is unavailable")
    _full_check(directory, base, manifest_sha256, report, python, 30.0)


@dataclass(frozen=True)
class ExecutionTarget:
    base: object = field(repr=False, compare=False)
    output: Path
    manifest_sha256: str
    _report_json: bytes = field(repr=False)
    _preflighted: bool = field(default=False, init=False, repr=False, compare=False)

    @property
    def id(self):
        return json.loads(self._report_json)["id"]

    @property
    def fingerprint(self):
        return json.loads(self._report_json)["fingerprint"]

    @property
    def model_path(self):
        return self.output / "cell.xml"

    # These narrow structural properties let the existing native evaluator use
    # the selected model while policy and legacy build validation use .base.
    @property
    def root(self):
        return self.base.root

    @property
    def joints(self):
        return self.base.joints

    @property
    def instances(self):
        return self.base.instances

    @property
    def home(self):
        return self.base.home

    def description(self):
        """Detached identity only; no local paths or raw design geometry."""
        report = json.loads(self._report_json)
        return dict(schema_version=1, artifact_kind="environment-execution-target-v1",
            environment_artifact_kind=report["artifact_kind"], environment_id=report["id"],
            manifest_sha256=self.manifest_sha256, fingerprint=report["fingerprint"],
            base_workcell_id=report["base_workcell_id"],
            base_workcell_sha256=report["inputs"]["base_workcell_sha256"],
            base_manifest_sha256=report["inputs"]["base_manifest_sha256"],
            outputs_sha256={name: report["outputs"][name] for name in (
                "cell.xml", "cell.urdf", "cell.srdf", "controllers.yaml", "moveit.yaml", "moveit.rviz")},
            dependencies_sha256=sha(canonical(report["dependencies"])),
            report_sha256=sha(self._report_json),
            selection="explicit_local_startup_only",
            collision_geometry="Conservative imported-shape bounding boxes; sampled native clearance and the contact interlock remain required")

    def validate(self):
        """Rehash bounded files/dependencies without importing an engine/ROS."""
        if not self._preflighted:
            raise ValueError("Execution target requires its owned startup CPU preflight")
        report = check_overlay_identity(self.output, self.base,
            expected_sha256=self.manifest_sha256)
        if canonical(report) != self._report_json:
            raise ValueError("Execution environment changed; restart with a newly checked target")
        return self.description()


def load_execution_target(base_cell, directory=None, manifest_sha256=None):
    """Resolve an explicit CLI pair, CPU-preflight once, then freeze its identity.

    No pair returns None and retains the existing base execution path. The
    expected manifest hash is mandatory; an artifact's own claim is insufficient.
    Failed checks never return a partial target or publish/modify an artifact.
    """
    if directory is None and manifest_sha256 is None:
        return None
    if (directory is None or not isinstance(manifest_sha256, str)
            or re.fullmatch("[0-9a-f]{64}", manifest_sha256) is None):
        raise ValueError("Execution environment requires a directory and lowercase SHA256 pair")
    candidate = Path(directory).absolute()
    if not candidate.is_dir() or candidate.is_symlink():
        raise ValueError("Execution environment must be an existing unlinked directory")
    directory = candidate.resolve(strict=True)
    report = check_overlay_identity(directory, base_cell, expected_sha256=manifest_sha256)
    target = ExecutionTarget(base_cell, directory, manifest_sha256, canonical(report))
    _preflight(directory, base_cell, manifest_sha256, report)
    object.__setattr__(target, "_preflighted", True)
    target.validate()
    return target
