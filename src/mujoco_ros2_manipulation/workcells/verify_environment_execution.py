"""Owned-domain environment execution checks; never attach an existing server.

Cases run separately so an intentionally changed target cannot be reused after
its server is blocked. Only the changed-target case mutates files, and only in
its newly composed private derivative artifact. The supplied target stays intact.
"""
import argparse
from copy import deepcopy
import json
from pathlib import Path
import uuid

import rclpy
from std_msgs.msg import String
from std_srvs.srv import Trigger

from mujoco_ros2_manipulation.paths import ROOT
from mujoco_ros2_manipulation.model_build.integrity import file_sha256
from mujoco_ros2_manipulation.verification.diagnostics import diagnostic_environment, finalize_physics
from mujoco_ros2_manipulation.verification.reports import VerificationReport, atomic_json, utc_now
from mujoco_ros2_manipulation.verification.runner import run_verification
from .config import load_workcell
from .distance import DistanceEvaluator
from .environment_clearance import environment_policy_snapshot, required_environment_distance
from .environment_native import check_native_overlay, _owned as run_owned_cpu
from .environment_overlay import check_overlay_identity
from .execution_target import load_execution_target
from .integrity import check_workcell
from .motion_request import COMMAND_TOPIC, PLAN_TOPIC, APPROVE_TOPIC, CANCEL_TOPIC, command
from .motion_server import MotionCommandServer, MotionEnvironmentChanged
from .verify import require
from .verify_motion_reproduction import require_unused_domain
from .verify_motion_review import MotionReviewVerification


class EnvironmentExecutionVerification(MotionReviewVerification):
    def __init__(self, cell, progress, output, target, case):
        MotionCommandServer.__init__(self, cell, output, progress,
            trace_root=ROOT/'runtime', execution_target=target)
        self.case = case
        self.senders = {name:self.create_publisher(String, topic, 10) for name,topic in (
            ('command', COMMAND_TOPIC), ('plan', PLAN_TOPIC),
            ('approve', APPROVE_TOPIC), ('cancel', CANCEL_TOPIC))}
        self.review_check = dict(schema_version=1, status='running', case=case,
            session_id=self.session_id, execution_target=target.description(), checks=[],
            implementation_sha256={name:file_sha256(Path(__file__).with_name(name)) for name in (
                'verify_environment_execution.py', 'execution_target.py', 'motion_server.py',
                'execution_adapter.py', 'launch.py', 'environment_cli.py', 'environment_native.py')},
            note='Owned simulation only. Action terminal, joint settling, native contacts and process cleanup have separate evidence; no real-robot safety or performance claim.')
        self.save_review_checks()

    def _request(self, kind, payload, expected):
        self.send_review_message(kind, payload)
        record = self.read_review_trace(payload['request_id'])
        require(record['status'] == expected, 'Unexpected environment request outcome: '+str(record.get('error')))
        self.record_check(kind+'_'+expected, request_id=payload['request_id'],
            dispatched_goals=record['dispatched_goals'], action_results=record['action_results'],
            planned_clearance=record.get('planned_clearance'), observed_clearance=record.get('observed_clearance'),
            execution_transport=record.get('execution_transport'))
        return record

    def _normal(self):
        for publisher in self.senders.values():
            self.until(lambda:publisher.get_subscription_count() >= 1, 5, 'environment topic subscription')
        initial = self.state()
        target = dict(initial)
        for instance in self.cell.instances:
            target[instance.joints[0]] += .04
        plan, approval = self.plan_for_review(target)
        self.record_check('environment_plan_only_zero_dispatch', request_id=plan['request_id'],
            dispatched_goals=plan['dispatched_goals'], planned_clearance=plan['planned_clearance'])
        executed = self._request('approve', approval, 'succeeded')
        require(executed['dispatched_goals'] == len(self.cell.instances)
            and executed['trajectories'] == plan['trajectories'], 'Approval did not execute the exact reviewed environment path')
        require(all(item['terminal_seen'] and item['confirmed_goal_uuid'] for item in
            executed['execution_transport']['attempts']), 'Execution lacks confirmed UUID/terminal ownership')
        self._request('command', command(initial), 'succeeded')

        cancelled_target = dict(initial)
        for instance in self.cell.instances:
            cancelled_target[instance.joints[0]] += .12
        payload = command(cancelled_target)
        sent = [False]
        def cancel_moving():
            if self.active and self.first_by_arm and not sent[0]:
                self.senders['cancel'].publish(String(data=json.dumps(dict(request_id=payload['request_id']))))
                sent[0] = True
        timer = self.create_timer(.02, cancel_moving)
        try:
            cancelled = self._request('command', payload, 'cancelled')
        finally:
            self.destroy_timer(timer)
        require(sent[0] and cancelled['metrics']['received_to_first_motion_wall_sec'] is not None,
                'Cancellation did not occur after observed robot motion')
        require(cancelled['dispatched_goals'] == len(self.cell.instances)
            and cancelled['execution_transport']['cleanup_uncertain'] is False
            and any(item['status'] == 5 for item in cancelled['action_results']),
            'Moving cancellation lacks owned terminal confirmation')
        require(not self.blocked and not self.contact_guard['latched'],
                'Normal environment cancellation blocked or latched the server')
        self._request('command', command(initial), 'succeeded')
        # Planning's .003rad goal tolerance is distinct from this profile's
        # existing tracking/settling tolerance. Reuse the production consecutive-
        # observation gate against the requested home, then record its bound.
        self.settle([initial[name] for name in self.joint_names])
        home_error = max(abs(self.state()[name]-initial[name]) for name in self.joint_names)
        self.record_check('home_return_settled', maximum_joint_error_rad=home_error,
            tracking_tolerance_rad=self.settings['tracking_tolerance_rad'])
        self.execution_target.validate()

    def _changed_after_review(self):
        for publisher in self.senders.values():
            self.until(lambda:publisher.get_subscription_count() >= 1, 5, 'environment topic subscription')
        target = self.state()
        target[self.joint_names[0]] += .04
        plan, approval = self.plan_for_review(target)
        path = self.execution_target.model_path
        before = file_sha256(path)
        # This target is a newly composed derivative owned by this verification;
        # the input artifact is never modified or restored in place.
        with path.open('ab') as stream:
            stream.write(b'\n<!-- owned W06 stale-target negative case -->\n')
        after = file_sha256(path)
        require(after != before, 'Owned negative target mutation did not occur')
        result = self.send_review_message('approve', approval)
        record = self.read_review_trace(approval['request_id'])
        require(result['status'] in ('rejected', 'failed') and record['dispatched_goals'] == 0
            and self.dispatched == 0 and self.blocked, 'Changed environment approval dispatched or did not block')
        require((self.review_store.summary() or {}).get('state') == 'invalidated',
                'Changed environment did not invalidate its awaiting review')
        self.record_check('changed_target_after_review_zero_dispatch',
            plan_request_id=plan['request_id'], request_id=approval['request_id'],
            dispatched_goals=0, model_sha256_before=before, model_sha256_after=after,
            error=record['error'], review_state=self.review_store.summary()['state'])

    def run(self):
        try:
            if self.case == 'mismatched_runtime':
                try:
                    self.prepare()
                except MotionEnvironmentChanged as error:
                    require(self.dispatched == 0 and self.blocked, 'Mismatched environment may have dispatched')
                    self.record_check('base_runtime_vs_overlay_target_zero_dispatch', dispatched_goals=0,
                        error=dict(type=type(error).__name__, message=str(error)))
                else:
                    raise RuntimeError('Mismatched base runtime was accepted as the environment target')
            else:
                self.prepare()
                if self.case == 'normal':
                    self._normal()
                elif self.case == 'changed_after_review':
                    self._changed_after_review()
                else:
                    raise ValueError('Unknown environment execution case')
            self.review_check['status'] = 'completed'
        except BaseException as error:
            self.review_check.update(status='aborted', error=dict(type=type(error).__name__, message=str(error)))
            raise
        finally:
            self.save_review_checks()
        return dict(environment_execution_verification_path=str(self.directory/'review-verification.json'),
            environment_execution_case=self.case, execution_target=self.execution_target.description(),
            dispatched_goals=self.dispatched, checks=len(self.review_check['checks']),
            contact_guard=deepcopy(self.contact_guard), expected_rejection=self.case != 'normal')


def verify_bad_sha(args):
    """Expected pre-launch refusal; no ROS context, simulation or client exists."""
    report = VerificationReport(ROOT, 'environment_execution_bad_sha', 'preflight', args.domain)
    try:
        cell = load_workcell(args.workcell, root=args.model_workspace.resolve())
        check_workcell(cell)
        check_overlay_identity(args.execution_environment.resolve(), cell,
            expected_sha256=args.execution_manifest_sha256)
        wrong = '0'*64 if args.execution_manifest_sha256 != '0'*64 else '1'*64
        try:
            load_execution_target(cell, args.execution_environment, wrong)
        except ValueError as error:
            require('manifest SHA mismatch' in str(error), 'Wrong SHA failed for an unrelated reason')
            report.update(status='passed', completed_utc=utc_now(), stage='complete', expected_rejection=True,
                error=None, rejection=dict(type=type(error).__name__, message=str(error)),
                supplied_negative_manifest_sha256=wrong, source_manifest_sha256=args.execution_manifest_sha256,
                launch_started=False, ros_entities_created=False, dispatched_goals=0,
                physics_steps=0, shutdown_clean=True, process_group_gone=True)
            print('PASSED: '+str(report.path), flush=True)
            return 0
        raise RuntimeError('Wrong environment manifest SHA was accepted')
    except Exception as error:
        report.update(status='failed', completed_utc=utc_now(), stage='complete',
            error=dict(type=type(error).__name__, message=str(error)), launch_started=False,
            ros_entities_created=False, dispatched_goals=0)
        print('FAILED: '+str(report.path), flush=True)
        return 1


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--domain', type=int, required=True)
    parser.add_argument('--workcell', default='fr5_ur5e')
    parser.add_argument('--execution-environment', type=Path, required=True)
    parser.add_argument('--execution-manifest-sha256', required=True)
    parser.add_argument('--case', choices=('normal', 'bad_sha', 'changed_after_review', 'mismatched_runtime'), default='normal')
    parser.add_argument('--output', type=Path, help='New command session directory under this workspace runtime/')
    parser.add_argument('--model-workspace', type=Path, default=ROOT)
    args = parser.parse_args(argv)
    require(0 <= args.domain <= 232, 'ROS domain must be 0..232')
    if args.case == 'bad_sha':
        return verify_bad_sha(args)
    # Refuse an occupied domain; this probe owns/reaps only its own ROS context.
    require_unused_domain(args.domain)
    args.robot, args.scene, args.gui, args.stop_mode = args.workcell, 'default', False, 'interrupt'
    args.require_gpu, args.screenshot_dir = False, None
    output = (args.output or ROOT/'runtime/validation/environment-sessions'/uuid.uuid4().hex).resolve()
    require(output.is_relative_to((ROOT/'runtime').resolve()) and not output.exists(),
            'Environment verification requires a new owned runtime session directory')
    state = {}

    def profile_loader(*_, **__):
        cell = load_workcell(args.workcell, root=args.model_workspace.resolve())
        check_workcell(cell)
        original = load_execution_target(cell, args.execution_environment, args.execution_manifest_sha256)
        state['source_target'] = original
        target = original
        if args.case == 'changed_after_review':
            report = check_overlay_identity(original.output, cell, expected_sha256=original.manifest_sha256)
            assets = [dict(bundle=original.output/entry['bundle_path'],
                manifest_sha256=entry['manifest_sha256'], installation=entry['installation'])
                for entry in report['installations']]
            directory = output.with_name(output.name+'-owned-negative-target')
            # ROS Python deliberately has no CPU MuJoCo engine. Compose only in
            # a separately owned .venv child using the existing artifact CLI.
            work = output.with_name(output.name+'-negative-fixture')
            work.mkdir(parents=True, mode=0o700, exist_ok=False)
            plan = work/'installations.json'
            atomic_json(plan, dict(schema_version=1, assets=[dict(item, bundle=str(item['bundle'])) for item in assets]))
            argv = ['/usr/bin/env', 'MUJOCO_ROS2_WORKSPACE='+str(cell.root),
                str(cell.root/'.venv/bin/python'), '-m',
                'mujoco_ros2_manipulation.workcells.environment_cli', '--workcell', cell.id,
                '--assets', str(plan), '--output', str(directory),
                '--overlay-id', 'w06_changed_'+uuid.uuid4().hex[:12]]
            with (work/'compose.log').open('wb') as log:
                run_owned_cpu(argv, timeout=60, stdout=log, stderr=log)
            state['negative_artifact_creation'] = dict(owned_child=True,
                implementation_sha256=file_sha256(Path(__file__).with_name('environment_cli.py')),
                installation_plan_sha256=file_sha256(plan), compose_log=str(work/'compose.log'))
            target = load_execution_target(cell, directory, file_sha256(directory/'overlay.json'))
        state['target'] = target
        native_work = output.with_name(output.name+'-native-geometry')
        native_work.mkdir(parents=True, mode=0o700, exist_ok=False)
        report = check_overlay_identity(target.output, cell, expected_sha256=target.manifest_sha256)
        state['native_geometry'] = check_native_overlay(target.output, cell, report, native_work)
        with DistanceEvaluator(target, environment=True) as evaluator:
            state['home_clearance'] = evaluator.at(dict(zip(cell.joints, cell.home)))
            state['native_evaluator'] = evaluator.metadata()
        _, policy, _ = environment_policy_snapshot(cell)
        require(state['home_clearance']['environment']['distance_m'] >= required_environment_distance(policy),
                'Environment home is below the planned clearance policy')
        return cell

    def command_builder(cell, _):
        result = ['bash', str(cell.root/'run_scene.sh'), 'workcell:='+cell.id, 'gui:=false', 'rviz:=false']
        if args.case != 'mismatched_runtime':
            target = state['target']
            result += ['execution_environment:='+str(target.output),
                       'execution_manifest_sha256:='+target.manifest_sha256]
        return result

    def finalize(report, cell, arguments):
        finalize_physics(report, cell, arguments)
        # The supplied source stays unchanged, even when the separately owned
        # derivative was deliberately made stale by the negative test.
        state['source_target'].validate()
        report.update(source_execution_target=state['source_target'].description(),
            execution_target=state['target'].description(), native_installed_geometry=state['native_geometry'],
            native_evaluator=state['native_evaluator'], home_clearance=state['home_clearance'],
            source_artifact_unchanged=True)
        if 'negative_artifact_creation' in state:
            report.update(negative_artifact_creation=state['negative_artifact_creation'])

    return run_verification(args, ROOT, rclpy,
        lambda cell,progress:EnvironmentExecutionVerification(cell, progress, output, state['target'], args.case),
        Trigger, profile_loader=profile_loader, validator=check_workcell,
        command_builder=command_builder, report_prefix='environment_execution_'+args.case+'_',
        launch_environment=diagnostic_environment, finalize=finalize)


if __name__ == '__main__':
    raise SystemExit(main())
