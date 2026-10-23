"""Run one owned simulation comparison; never accept a user-supplied command.

The console owns this worker and its renderer. Each verifier owns a fresh launch
through run_verification. Status identifies every nested launch before ROS init,
so the console can clean up that exact process group after a worker failure.
"""
import argparse
import json
import math
import os
from pathlib import Path
import re
import signal
import time
from types import SimpleNamespace

from ..model_build.integrity import file_sha256
from ..paths import ROOT
from ..verification.reports import atomic_json
from ..verification.runner import group_gone
from .catalog import describe_scenarios


class DemoCancelled(KeyboardInterrupt):
    """User interruption must stop the pair without starting another launch."""


def read_process_start_ticks(pid, *, proc=Path("/proc")):
    """Linux process identity, separate from its reusable PID."""
    return int((Path(proc) / str(pid) / "stat").read_text().rsplit(")", 1)[1].split()[19])


def frame_matches(frame, job_id, run_id, model_sha256, *, now=None):
    """Accept only a fresh rendered observation for this exact scene/run/job."""
    now = time.monotonic() if now is None else now
    if not isinstance(frame, dict) or not model_sha256:
        return False
    rendered, age = frame.get("rendered_monotonic"), frame.get("source_state_age_sec")
    finite_number = lambda v: type(v) in (int, float) and math.isfinite(v)
    return (frame.get("job_id") == job_id and frame.get("run_id") == run_id
            and frame.get("model_sha256") == model_sha256
            and type(frame.get("sequence")) is int and frame["sequence"] > 0
            and finite_number(rendered) and 0 <= now - rendered < 1.0
            and finite_number(age) and 0 <= age < 0.5 and not frame.get("error"))


def _json(path, limit=64 * 1024 * 1024):
    path = Path(path)
    if path.stat().st_size > limit:
        raise ValueError("Owned result exceeds the bounded JSON read limit")
    return json.loads(path.read_text())


def summarize_run(report, variant, label, path):
    """Keep UI metrics compact; the unchanged full report remains on disk."""
    physics = report.get("physics") or {}
    planned, observed = report.get("planned") or {}, report.get("observed") or {}
    execution = report.get("coordinated_execution") or {}
    return dict(
        variant=variant, label=label, status=report["status"], run_id=report["run_id"],
        contact_count=physics.get("contacts"), stopped=physics.get("stop_latched"),
        duration_sec=report.get("motion_duration_sec", report.get("commanded_duration_sec", execution.get("duration_sec"))),
        planned_min_distance_m=(planned.get("minimum") or {}).get("distance_m"),
        observed_min_distance_m=(observed.get("minimum") or {}).get("distance_m"),
        joint_travel_rad=planned.get("joint_total_variation_rad"),
        goal_terminal_statuses=report.get("goal_terminal_statuses", execution.get("goal_terminal_statuses")),
        report_path=str(path), report_sha256=file_sha256(path),
        shutdown_clean=report.get("shutdown_clean"), process_group_gone=report.get("process_group_gone"),
        error=report.get("error"), failure_stage=report.get("failure_stage"),
        contact_motion_elapsed_sec=report.get("native_contact_motion_elapsed_sec"),
        departure_delay_sec=report.get("departure_delay_sec"),
    )


class DemoJob:
    def __init__(self, directory, scenario, domain, console_directory, *, render=True, native_gui=False, root=ROOT):
        self.root = Path(root).resolve()
        self.directory, self.console_directory = Path(directory).resolve(), Path(console_directory).resolve()
        operator = (self.root / "runtime/operator").resolve()
        if (not self.console_directory.is_relative_to(operator)
                or not self.directory.is_relative_to(self.console_directory)
                or self.directory == self.console_directory
                or not re.fullmatch("[0-9a-f]{32}", self.directory.name)):
            raise ValueError("Use a new hex job directory inside this console's runtime/operator session")
        self.spec = next((item for item in describe_scenarios() if item["id"] == scenario), None)
        if self.spec is None or type(domain) is not int or not 0 <= domain <= 232:
            raise ValueError("Unknown fixed scenario or invalid ROS domain")
        if type(render) is not bool or type(native_gui) is not bool:
            raise ValueError("Render options must be booleans")
        self.directory.mkdir(parents=True, exist_ok=True)
        self.path = self.directory / "status.json"
        if self.path.exists():
            initial = _json(self.path, 65536)
            if (initial.get("schema_version") != 1 or initial.get("job_id") != self.directory.name
                    or initial.get("scenario") != scenario or initial.get("status") != "starting"
                    or initial.get("active_launch") is not None or initial.get("results") != []):
                raise ValueError("Only this job's unstarted console status may be initialized")
        self.domain, self.render, self.native_gui = domain, render, native_gui
        self.cancel_requested = False
        self.active_report = None
        self.data = dict(schema_version=1, job_id=self.directory.name, scenario=scenario,
                         status="running", stage="preparing", variant=None, run_id=None,
                         active_launch=None, model_sha256=None, results=[], comparison=None, error=None)
        self.save()

    def save(self):
        atomic_json(self.path, self.data)

    def stage(self, name):
        self.check_cancel()
        self.data["stage"] = name
        self.save()

    def check_cancel(self):
        if self.cancel_requested:
            raise DemoCancelled("The user stopped this simulation comparison")

    def interrupt(self, *_):
        if not self.cancel_requested:
            self.cancel_requested = True
            raise DemoCancelled("The user stopped this simulation comparison")

    def _on_launch(self, report, cell, args, process):
        self.check_cancel()
        self.active_report = report
        model = (report.directory / "contact_test.xml"
                 if self.data["variant"] in ("contact", "control") else cell.output / "cell.xml")
        self.data.update(run_id=report.run_id, active_launch=dict(
            pid=process.pid, start_ticks=read_process_start_ticks(process.pid),
            run_id=report.run_id, model_path=str(model.resolve()), model_sha256=None,
            report_path=str(report.path.resolve()),
        ))
        self.stage("launching")

    def _frame_barrier(self, node):
        self.check_cancel()
        launch = self.data["active_launch"]
        if not launch or self.active_report is None:
            raise RuntimeError("Missing owned launch identity")
        model = Path(launch["model_path"])
        node.until(lambda: model.is_file(), 15, "owned renderer model")
        resolved = model.resolve()
        expected = (self.active_report.directory / "contact_test.xml"
                    if self.data["variant"] in ("contact", "control")
                    else self.root / "runtime/generated/workcells/fr5_ur5e/cell.xml")
        if resolved != expected.resolve() or not resolved.is_relative_to(self.root / "runtime"):
            raise ValueError("Renderer model escaped the owned verification scene")
        launch.update(model_path=str(resolved), model_sha256=file_sha256(resolved))
        self.data["model_sha256"] = launch["model_sha256"]
        self.stage("waiting_for_frame" if self.render else "scene_ready")
        if not self.render:
            return
        def ready():
            self.check_cancel()
            try:
                frame = _json(self.console_directory / "frame.json", 65536)
            except (OSError, ValueError):
                return False
            return frame_matches(frame, self.data["job_id"], launch["run_id"], launch["model_sha256"])
        node.until(ready, 30, "fresh console frame for owned job/run/model")
        self.stage("frame_ready")

    def _args(self, *, contact=False):
        return SimpleNamespace(robot="fr5_ur5e", scene="default", domain=self.domain,
                               gui=self.native_gui, stop_mode="interrupt",
                               require_gpu=False, screenshot_dir=None, contact_stop_test=contact)

    def _run_verifier(self, variant, cell, factory, validator, command, finalize, prefix, *, contact=False):
        from rclpy.signals import SignalHandlerOptions
        import rclpy
        from std_srvs.srv import Trigger
        from ..verification.diagnostics import diagnostic_environment
        from ..verification.runner import run_verification

        self.check_cancel()
        self.active_report = None
        self.data.update(variant=variant, run_id=None, active_launch=None, model_sha256=None)
        self.stage("preflight")
        args = self._args(contact=contact)
        def make_node(profile, progress):
            def observed_progress(name):
                self.check_cancel()
                progress(name)
                self.stage(name)
            return factory(profile, observed_progress)
        # Preserve our SIGINT -> no-next-variant rule across every ROS context.
        ros = SimpleNamespace(
            init=lambda: rclpy.init(signal_handler_options=SignalHandlerOptions.NO),
            ok=rclpy.ok, shutdown=rclpy.shutdown,
        )
        code = run_verification(
            args, self.root, ros, make_node, Trigger,
            profile_loader=lambda *a, **kw: cell,
            validator=validator, command_builder=lambda *a: command,
            report_prefix=prefix, launch_environment=diagnostic_environment,
            finalize=finalize, on_launch=self._on_launch,
        )
        report = self.active_report
        value = _json(report.path) if report is not None else None
        if value is not None:
            label = next(item["label"] for item in self.spec["variants"] if item["id"] == variant)
            self.data["results"].append(summarize_run(value, variant, label, report.path))
        launch = self.data["active_launch"]
        if launch and group_gone(launch["pid"]):
            self.data["active_launch"] = None
        self.save()
        if code == 130 or self.cancel_requested:
            raise DemoCancelled("The user stopped this simulation comparison")
        if (code != 0 or value is None or value.get("status") != "passed"
                or value.get("shutdown_clean") is not True
                or value.get("process_group_gone") is not True):
            cause = (value or {}).get("error") or dict(
                type="VerificationFailure",
                message="Verification failed before launch; the worker log preserves the original report path")
            raise RuntimeError(cause.get("message", str(cause)))
        self.check_cancel()
        return report.path, value

    def _command(self, cell, *, contact=False, control=False, case=False):
        options = [f"workcell:={cell.id}", f"gui:={str(self.native_gui).lower()}",
                   f"rviz:={str(self.native_gui).lower()}"]
        if contact:
            options += ["contact_demo:=true", f"contact_demo_control:={str(control).lower()}"]
        if case:
            options += ["planning_case:=dual_arm_crossing"]
        return ["bash", str(self.root / "run_scene.sh"), *options]

    def _contact_pair(self, cell):
        from ..workcells.verify_contact_demo import VisibleContactDemo, finalize_demo
        from ..workcells.contact_comparison import compare_runs
        from ..workcells.integrity import check_workcell
        job = self
        class LiveContact(VisibleContactDemo):
            def before_motion(self):
                job._frame_barrier(self)
        reports = []
        for variant, control in (("contact", False), ("control", True)):
            def factory(profile, progress, control=control):
                return LiveContact(profile, progress, control=control)
            def finalize(report, profile, args, control=control):
                args.control = control
                return finalize_demo(report, profile, args)
            _, report = self._run_verifier(
                variant, cell, factory, check_workcell,
                self._command(cell, contact=True, control=control), finalize,
                "control_contact_" if control else "visible_contact_", contact=not control,
            )
            reports.append(report)
        self.check_cancel()
        self.stage("comparing")
        return compare_runs(*reports)

    def _timing_pair(self, cell):
        from ..workcells.verify_interference import InterferenceVerification, preflight
        from ..workcells.verify_timing import TimingVerification
        from ..workcells.interference_comparison import freeze_bundle, load_bundle
        from ..workcells.distance import DistanceEvaluator
        from ..workcells.timing import prepare_timing, load_timing
        from ..workcells.timing_comparison import compare_timing
        from ..verification.diagnostics import finalize_physics
        job = self
        class LiveInterference(InterferenceVerification):
            def discover(self):
                super().discover()
                job._frame_barrier(self)
            def before_avoidance_execution(self):
                job._frame_barrier(self)
        source, _ = self._run_verifier(
            "prepare", cell, lambda profile, progress: LiveInterference(profile, progress, "dual_arm_crossing"),
            lambda profile: preflight(profile, "dual_arm_crossing"),
            self._command(cell, case=True), finalize_physics, "interference_dual_arm_crossing_",
        )
        self.stage("freezing_bundle")
        bundle, plan = self.directory / "bundle.json", self.directory / "timing-plan.json"
        freeze_bundle(source, bundle, cell)
        self.stage("searching_departure_delay")
        with DistanceEvaluator(cell) as evaluator:
            prepare_timing(bundle, plan, cell, evaluator)
        self.check_cancel()
        class LiveTiming(TimingVerification):
            def discover(self):
                super().discover()
                job._frame_barrier(self)
            def before_avoidance_execution(self):
                job._frame_barrier(self)
        def validator(profile):
            value, _ = load_bundle(bundle, profile)
            preflight(profile, value["case_id"])
            with DistanceEvaluator(profile) as evaluator:
                load_timing(plan, bundle, profile, evaluator)
        reports = []
        for variant in ("baseline", "timing"):
            path, _ = self._run_verifier(
                variant, cell,
                lambda profile, progress, variant=variant: LiveTiming(profile, progress, bundle, plan, variant),
                validator, self._command(cell, case=True), finalize_physics, "timing_" + variant + "_",
            )
            reports.append(path)
        self.check_cancel()
        self.stage("comparing")
        return compare_timing(reports, plan)

    def run(self):
        previous = signal.signal(signal.SIGINT, self.interrupt)
        try:
            from ..workcells.config import load_workcell
            os.environ["ROS_DOMAIN_ID"] = os.environ["MANIPULATION_ROS_DOMAIN_ID"] = str(self.domain)
            cell = load_workcell("fr5_ur5e", root=self.root)
            comparison = self._contact_pair(cell) if self.data["scenario"] == "contact_comparison" else self._timing_pair(cell)
            self.check_cancel()
            atomic_json(self.directory / "comparison.json", comparison)
            self.data.update(status="succeeded", stage="complete", comparison=comparison)
            self.save()
            return 0
        except (DemoCancelled, KeyboardInterrupt) as error:
            self.cancel_requested = True
            self.data.update(status="cancelled", stage="cancelled", comparison=None,
                             error=dict(type="Cancelled", message=str(error) or "The user stopped the comparison"))
            self.save()
            return 130
        except Exception as error:
            self.data.update(status="failed", stage="failed", comparison=None,
                             error=dict(type=type(error).__name__, message=str(error)))
            self.save()
            return 1
        finally:
            signal.signal(signal.SIGINT, previous)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--directory", type=Path, required=True)
    parser.add_argument("--scenario", choices=[item["id"] for item in describe_scenarios()], required=True)
    parser.add_argument("--domain", type=int, required=True)
    parser.add_argument("--console-directory", type=Path, required=True)
    parser.add_argument("--no-render", action="store_true")
    parser.add_argument("--native-gui", action="store_true")
    args = parser.parse_args()
    try:
        job = DemoJob(args.directory, args.scenario, args.domain, args.console_directory,
                      render=not args.no_render, native_gui=args.native_gui)
    except ValueError as error:
        parser.error(str(error))
    return job.run()


if __name__ == "__main__":
    raise SystemExit(main())
