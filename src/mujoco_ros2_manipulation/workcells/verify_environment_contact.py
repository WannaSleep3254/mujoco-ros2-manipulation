"""Explicit simulation-only contact diagnostic against one imported fixed box.

This CLI deliberately bypasses motion planning for a bounded raw FJT test. It
adds no production command, browser control, planner exception or weaker gate.
The native contact interlock and controller guard remain enabled throughout.
"""
import argparse
from copy import deepcopy
import json
import math
from pathlib import Path
import uuid

import rclpy
from builtin_interfaces.msg import Time
from control_msgs.action import FollowJointTrajectory
from std_srvs.srv import Trigger
from std_msgs.msg import String

from mujoco_ros2_manipulation.paths import ROOT
from mujoco_ros2_manipulation.model_build.integrity import file_sha256
from mujoco_ros2_manipulation.verification.diagnostics import diagnostic_environment, check_stop_physics
from mujoco_ros2_manipulation.verification.reports import atomic_json
from mujoco_ros2_manipulation.verification.runner import run_verification
from .config import load_workcell
from .distance import DistanceEvaluator
from .environment_assets import validate_bundle
from .environment_native import check_native_overlay
from .environment_overlay import check_overlay_identity
from .execution_adapter import ExecutionLedger, ExecutionAttempt
from .execution_target import load_execution_target
from .integrity import check_workcell
from .motion_server import MotionCommandServer
from .motion_request import COMMAND_TOPIC, PLAN_TOPIC, APPROVE_TOPIC, CANCEL_TOPIC, command
from .verify import require
from .verify_motion_reproduction import require_unused_domain
from .verify_motion_review import MotionReviewVerification
from .verify_environment_execution import EnvironmentExecutionVerification


OFFSETS_RAD = (.6, .08)
DURATION_SEC = 4
CONTACT_BODY = 'environment_contact_box'
SAMPLES = 1001


def contact_path_gate(home_environment_m, minimum_inter_arm_m, minimum_environment, *, sample_count):
    """Require a clear start, isolated inter-arm space and the intended overlap."""
    require(type(sample_count) is int and sample_count == SAMPLES, 'Incomplete contact diagnostic sample coverage')
    require(all(type(value) in (int, float) and math.isfinite(value) for value in (
        home_environment_m, minimum_inter_arm_m, minimum_environment['distance_m'])),
        'Contact diagnostic distances must be finite')
    require(home_environment_m >= .015, 'Contact diagnostic start is below 15mm environment clearance')
    require(minimum_inter_arm_m >= .03, 'Contact diagnostic inter-arm clearance is inadequate')
    bodies = minimum_environment['bodies']
    require(type(bodies) is list and len(bodies) == 2 and CONTACT_BODY in bodies
        and any(type(name) is str and name.startswith('left_') for name in bodies)
        and minimum_environment['distance_m'] < 0,
            'Diagnostic path does not penetrate the selected imported collider')


def raw_goal_contract(trajectory, names, initial, goal):
    """Bind serialized FJT order/endpoints to the exact nominal FK input."""
    require(list(trajectory.joint_names) == list(names), 'Raw goal joint order differs from the FK contract')
    require(len(trajectory.points) == 2, 'Raw diagnostic requires exactly two linear endpoints')
    first, last = trajectory.points
    require(list(first.positions) == [initial[name] for name in names], 'Raw goal initial point differs from the sampled FK anchor')
    require(list(last.positions) == [goal[name] for name in names], 'Raw goal target differs from the sampled FK target')
    require(first.time_from_start.sec == first.time_from_start.nanosec == 0
        and last.time_from_start.sec == DURATION_SEC and last.time_from_start.nanosec == 0
        and all(not point.velocities and not point.accelerations for point in trajectory.points),
        'Raw diagnostic requires its fixed position-only linear time contract')
    return dict(joint_names=list(names), fk_initial_rad=[initial[name] for name in names],
        first_point_positions_rad=list(first.positions), final_point_positions_rad=list(last.positions),
        nominal_endpoint_times_sec=[0, DURATION_SEC], exact_fk_goal_anchor_match=True)


class EnvironmentContactVerification(MotionReviewVerification):
    _request = EnvironmentExecutionVerification._request

    def __init__(self, cell, progress, output, target):
        MotionCommandServer.__init__(self, cell, output, progress, trace_root=ROOT/'runtime', execution_target=target)
        self.senders = {name:self.create_publisher(String, topic, 10) for name,topic in (
            ('command', COMMAND_TOPIC), ('plan', PLAN_TOPIC), ('approve', APPROVE_TOPIC), ('cancel', CANCEL_TOPIC))}
        self.review_check = dict(schema_version=1, status='running',
            case='same_target_safe_cycle_before_contact', session_id=self.session_id,
            execution_target=target.description(), checks=[])
        self.receipt = dict(schema_version=1, status='running', simulation_only=True,
            diagnostic_planning_bypass=True, unsafe_motion_public_api=False,
            production_planning_gates_changed=False, contact_injection_requests=0,
            fixed_imported_collider=True, execution_target=target.description(),
            verifier_cancel_requests_scope='raw_contact_phase_only',
            commanded_first_joint_offsets_rad=list(OFFSETS_RAD), duration_sec=DURATION_SEC,
            implementation_sha256={name:file_sha256(Path(__file__).with_name(name)) for name in (
                'verify_environment_contact.py', 'verify_environment_execution.py', 'verify_motion_review.py',
                'motion_server.py', 'execution_adapter.py', 'execution_target.py')})
        self.ledger = ExecutionLedger()
        self.transport = self._trajectory_executor()
        self.save_contact()

    def safe_production_cycle(self):
        """Use every unchanged production gate in this very same installed model."""
        for publisher in self.senders.values():
            self.until(lambda:publisher.get_subscription_count() >= 1, 5, 'safe production topic subscription')
        initial = self.state()
        goal = dict(initial)
        for instance in self.cell.instances:
            goal[instance.joints[0]] -= .04
        plan, approval = self.plan_for_review(goal)
        self.record_check('same_target_plan_only_zero_dispatch', dispatched_goals=0,
            request_id=plan['request_id'], planned_clearance=plan['planned_clearance'])
        executed = self._request('approve', approval, 'succeeded')
        require(executed['trajectories'] == plan['trajectories'], 'Safe approved cycle changed its reviewed path')
        self._request('command', command(initial), 'succeeded')
        cancel_target = dict(initial)
        for instance in self.cell.instances:
            cancel_target[instance.joints[0]] -= .12
        payload = command(cancel_target)
        published = [False]
        def cancel_moving():
            if self.active and self.first_by_arm and not published[0]:
                self.senders['cancel'].publish(String(data=json.dumps(dict(request_id=payload['request_id']))))
                published[0] = True
        timer = self.create_timer(.02, cancel_moving)
        try:
            cancelled = self._request('command', payload, 'cancelled')
        finally:
            self.destroy_timer(timer)
        require(published[0] and cancelled['metrics']['received_to_first_motion_wall_sec'] is not None
            and cancelled['execution_transport']['cleanup_uncertain'] is False,
            'Safe production cancellation lacks motion/owned cleanup evidence')
        self._request('command', command(initial), 'succeeded')
        self.settle([initial[name] for name in self.joint_names])
        require(not self.blocked and not self.contact_guard['latched'], 'Safe production cycle contacted or blocked')
        self._recheck_execution_target()
        self.review_check['status'] = 'completed'
        self.save_review_checks()
        self.receipt.update(same_target_safe_production_cycle=True,
            production_dispatched_goals=self.dispatched, production_cancel_topic_publications=1,
            safe_cycle_verification_path=str(self.directory/'review-verification.json'))

    def save_contact(self):
        atomic_json(self.directory/'environment-contact.json', self.receipt)

    def run(self):
        primary = None
        try:
            self.prepare()
            self.safe_production_cycle()
            self.ledger = ExecutionLedger(total_dispatched=self.dispatched)
            initial = self.state()
            # Diagnostic only: define one raw curve from this observed snapshot.
            # Do not inherit the preceding MoveIt endpoint's tolerated offset.
            self.commanded = dict(initial)
            goal = dict(initial)
            for instance, offset in zip(self.cell.instances, OFFSETS_RAD):
                goal[instance.joints[0]] += offset
                for name in instance.joints:
                    bounds = self.metadata['joint_limits'][name]
                    require(bounds['lower'] <= goal[name] <= bounds['upper'], 'Diagnostic target exceeds joint limits')
            home = self.evaluator.at(initial)
            minimum_inter = float('inf')
            minimum_env = None
            for index in range(SAMPLES):
                fraction = index/(SAMPLES-1)
                state = {name:initial[name]+fraction*(goal[name]-initial[name]) for name in initial}
                row = self.evaluator.at(state)
                minimum_inter = min(minimum_inter, row['distance_m'])
                env = row['environment']
                if minimum_env is None or env['distance_m'] < minimum_env['distance_m']:
                    minimum_env = dict(fraction=fraction, **env)
            contact_path_gate(home['environment']['distance_m'], minimum_inter, minimum_env, sample_count=SAMPLES)
            self.receipt.update(initial_state_rad=initial, target_rad=goal,
                actual_anchor_home_environment=home['environment'], sample_count=SAMPLES,
                nominal_inter_arm_minimum_m=minimum_inter, nominal_environment_minimum=minimum_env,
                sampling_scope='Native FK of bounded position-only linear FJT; not a physical rollout or continuous collision guarantee',
                sampling_clock='Nominal trajectory time; no measured latency or exact ROS/native step alignment')
            self.save_contact()
            self.execution_target.validate()
            self.fresh()
            start = self.clock_last+1.
            self.receipt['scheduled_start_simulation_sec'] = start
            self.progress('diagnostic_raw_goals_without_planning')
            commands, contracts = [], []
            for instance in self.cell.instances:
                trajectory = self.trajectory(instance, [goal[name] for name in instance.joints])
                trajectory.points[-1].time_from_start.sec = DURATION_SEC
                trajectory.header.stamp = Time(sec=int(start), nanosec=int((start%1)*1e9))
                contracts.append(dict(arm=instance.id, **raw_goal_contract(trajectory, instance.joints, initial, goal)))
                commands.append((instance, trajectory))
            self.receipt['raw_goal_contracts'] = contracts
            self.save_contact()
            for instance, trajectory in commands:
                attempt = ExecutionAttempt(instance.id, instance.controller)
                with self.ledger._lock:
                    self.ledger.attempts.append(attempt)
                self.check_abort()
                with self.ledger._lock:
                    attempt.submission_started = attempt.submission_uncertain = True
                future = self.arm_clients[instance.id].send_goal_async(FollowJointTrajectory.Goal(trajectory=trajectory))
                require(future is not None, 'Diagnostic raw submission has no acceptance future')
                with self.ledger._lock:
                    attempt.send_future = future
                    attempt.submission_uncertain = False
                    self.ledger.total_dispatched += 1
                # The normal adapter cannot drive this scenario: its guard abort
                # hook would request cancellation before autonomous guard proof.
                # Reuse its tested ownership/late-acceptance helpers only; no
                # planner or admission behavior is changed by this diagnostic.
                self.transport._register_acceptance_callback(self.ledger, attempt)
            for attempt in self.ledger.attempts:
                handle = self.result(attempt.send_future, 5, 'diagnostic raw goal acceptance')
                self.transport._claim(self.ledger, attempt, handle)
                require(attempt.accepted and attempt.goal_uuid, 'Diagnostic goal rejected or UUID unavailable')
            require(start-self.clock_last > .1, 'Diagnostic goals accepted too close to their start')
            self.progress('wait_for_natural_imported_box_contact')
            # No verifier cancellation/dispatch abort hook is used here. The
            # native latch and separate ROS guard must end the owned actions.
            self.until(lambda:self.contact_guard and self.contact_guard['latched'],
                DURATION_SEC+5, 'natural imported-box contact latch')
            self.until(lambda:all(attempt.result_future.done() for attempt in self.ledger.attempts),
                5, 'autonomous contact goal termination')
            statuses = [attempt.result_future.result().status for attempt in self.ledger.attempts]
            require(all(status in (5, 6) for status in statuses), 'Contact goals were not canceled/aborted')
            self.until(lambda:self.contact_guard['phase'] == 'stopped'
                and self.contact_guard.get('reason') == 'contact'
                and all(self.contact_guard['controllers'].get(instance.controller) == 'inactive'
                    for instance in self.cell.instances), 5, 'contact guard controllers inactive')
            stopped = self.state()
            stopped_clock, count = self.clock_last, self.joint_count
            self.until(lambda:self.joint_count >= count+20 and self.clock_last >= stopped_clock+.6,
                3, 'frozen state with advancing ROS clock')
            drift = max(abs(self.state()[name]-stopped[name]) for name in self.joint_names)
            require(drift < 1e-12 and all(value == 0 for value in self.latest_velocities.values()),
                    'State moved after native contact latch')
            require(not self.clock_backwards, 'ROS clock moved backwards after native contact')
            self.receipt.update(status='completed', goal_terminal_statuses=statuses,
                contact_guard=deepcopy(self.contact_guard), post_stop_drift_rad=drift,
                clock_advance_after_stop_sec=self.clock_last-stopped_clock,
                stationary_velocity_rad_sec=dict(self.latest_velocities))
        except BaseException as error:
            primary = error
            self.receipt.update(status='aborted', error=dict(type=type(error).__name__, message=str(error)))
            raise
        finally:
            try:
                self.transport.cancel_actions(self.ledger)
            except Exception as error:
                self.receipt['owned_goal_cleanup_error'] = dict(type=type(error).__name__, message=str(error))
                if primary is None:
                    self.receipt['status'] = 'aborted'
                    raise
            finally:
                self.receipt['execution_transport'] = self.ledger.snapshot()
                self.receipt['dispatched_goals'] = self.ledger.total_dispatched
                self.receipt['raw_diagnostic_goals'] = sum(attempt.send_future is not None for attempt in self.ledger.attempts)
                self.receipt['verifier_cancel_requests'] = sum(attempt.cancel_requested for attempt in self.ledger.attempts)
                self.save_contact()
        require(self.receipt['verifier_cancel_requests'] == 0,
                'Verifier cancellation would obscure autonomous guard evidence')
        # The common runner owns its report schema/status/error envelope. Keep
        # the diagnostic receipt's independent v1 envelope in its own file.
        payload = {name:deepcopy(value) for name,value in self.receipt.items()
                   if name not in ('schema_version', 'status', 'error')}
        payload['environment_contact_verification_path'] = str(self.directory/'environment-contact.json')
        return payload


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--domain', type=int, required=True)
    parser.add_argument('--execution-environment', type=Path, required=True)
    parser.add_argument('--execution-manifest-sha256', required=True)
    parser.add_argument('--allow-simulated-contact', action='store_true',
        help='Explicitly authorize this fixed bounded raw-FJT contact diagnostic, bypassing planning only in this CLI')
    parser.add_argument('--output', type=Path)
    args = parser.parse_args(argv)
    if not args.allow_simulated_contact:
        parser.error('--allow-simulated-contact is required for the diagnostic planning bypass')
    require(0 <= args.domain <= 232, 'ROS domain must be 0..232')
    require_unused_domain(args.domain)
    args.robot, args.scene, args.gui, args.stop_mode = 'fr5_ur5e', 'default', False, 'interrupt'
    args.require_gpu, args.screenshot_dir = False, None
    output = (args.output or ROOT/'runtime/validation/environment-contact'/uuid.uuid4().hex).resolve()
    require(output.is_relative_to((ROOT/'runtime').resolve()) and not output.exists(),
            'Contact diagnostic requires a new owned runtime directory')
    state = {}

    def profile_loader(*_, **__):
        cell = load_workcell()
        target = load_execution_target(cell, args.execution_environment, args.execution_manifest_sha256)
        report = check_overlay_identity(target.output, cell, expected_sha256=target.manifest_sha256)
        require(len(report['installations']) == 1, 'Contact diagnostic requires one imported box')
        entry = report['installations'][0]
        bundle = validate_bundle(target.output/entry['bundle_path'], expected_sha256=entry['manifest_sha256'])
        require('environment_'+entry['installation']['name'] == CONTACT_BODY
            and bundle['id'] == 'synthetic_negative_box' and bundle['source']['format'] == 'step',
            'Contact diagnostic supports only its explicitly generated STEP box fixture')
        state['target'] = target
        native_work = output.with_name(output.name+'-native-geometry')
        native_work.mkdir(parents=True, mode=0o700, exist_ok=False)
        state['native_geometry'] = check_native_overlay(target.output, cell, report, native_work)
        return cell

    def finalize(report, cell, _):
        path = report.directory/'physics.json'
        data = json.loads(path.read_text())
        report.update(physics=data, physics_report_sha256=file_sha256(path), physics_checks_passed=False)
        check_stop_physics(data, run_id=report.run_id,
            source_sha256=file_sha256(cell.root/'native/step_monitor.hpp'),
            timestep=cell.data['physics']['timestep'])
        require(data['pairs'] and all(CONTACT_BODY in pair['bodies'] for pair in data['pairs']),
                'Native contact was not exclusively with the selected imported box')
        state['target'].validate()
        report.update(physics_checks_passed=True, physics_verification=dict(
            native_checks_passed=True, observed_condition='contact_stop',
            scope='Native latch/hold coverage and actual imported-box contact; explicit simulation-only planning bypass'),
            native_installed_geometry=state['native_geometry'], source_artifact_unchanged=True)

    return run_verification(args, ROOT, rclpy,
        lambda cell,progress:EnvironmentContactVerification(cell, progress, output, state['target']),
        Trigger, profile_loader=profile_loader, validator=check_workcell,
        command_builder=lambda cell,args:['bash', str(cell.root/'run_scene.sh'), 'workcell:='+cell.id,
            'gui:=false', 'rviz:=false', 'execution_environment:='+str(state['target'].output),
            'execution_manifest_sha256:='+state['target'].manifest_sha256],
        report_prefix='environment_contact_', launch_environment=diagnostic_environment, finalize=finalize)


if __name__ == '__main__':
    raise SystemExit(main())
