"""Actual ROS regression of bounded candidate selection; injections are test-only."""
import argparse
from copy import deepcopy
import hashlib
import json
from pathlib import Path
import uuid

import rclpy
from moveit_msgs.msg import MoveItErrorCodes
from std_msgs.msg import String
from std_srvs.srv import Trigger

from mujoco_ros2_manipulation.paths import ROOT
from mujoco_ros2_manipulation.model_build.integrity import file_sha256
from mujoco_ros2_manipulation.verification.diagnostics import diagnostic_environment, finalize_physics
from mujoco_ros2_manipulation.verification.reports import atomic_json
from mujoco_ros2_manipulation.verification.runner import run_verification
from .config import load_workcell
from .integrity import check_workcell
from .interference import Spline
from .motion_request import COMMAND_TOPIC, CANCEL_TOPIC, command
from .motion_server import MotionCommandServer, MotionPlanningFailed
from .path_quality import evaluate_path, sample_schedule
from .verify import require
from .verify_motion_reproduction import require_unused_domain


class ReplanningVerification(MotionCommandServer):
    """Exercise the production loop with explicit, request-scoped test proposals.

    The withheld candidate is real MoveIt output for this exact initial state and
    goal, checked against the current native and MoveIt scene before reuse. Its
    first reported failure is deliberately injected; it is not a measured solver
    failure or a performance result. Every later production execution gate runs.
    """
    def __init__(self, cell, progress, output=None):
        super().__init__(cell, output or ROOT/'runtime/validation/replanning-sessions'/uuid.uuid4().hex,
                         progress, compute_budget_sec=30.0, max_plan_candidates=12)
        self.command_publisher = self.create_publisher(String, COMMAND_TOPIC, 10)
        self.cancel_publisher = self.create_publisher(String, CANCEL_TOPIC, 10)
        self.proposal_mode = None
        self.proposal_calls = 0
        self.saved_proposal = None
        self.result_record = dict(
            schema_version=1, status='running', session_id=self.session_id,
            source_sha256=file_sha256(Path(__file__)), checks=[], maintenance=[],
            note='Simulation-only functional regression. Explicit failed proposals and same-request saved successful MoveIt output are test-only; not solver success-rate or speed measurements.')
        self.save_checks()

    def save_checks(self):
        atomic_json(self.directory/'replanning.json', self.result_record)

    def _save_real_proposal(self, group, target, initial):
        trajectory = super().plan(group, target, initial)
        curve = Spline.from_ros(trajectory)
        threshold = self.policy['min_inter_arm_distance_m'] + self.policy['tracking_reserve_m']
        with self.active.span('verification_source_validation'):
            assessment = evaluate_path([curve], initial, self.cell, self.evaluator,
                dt=self.policy['sample_dt_sec'], reject_below=threshold, checkpoint=self.compute_checkpoint)
            require(assessment['complete'], 'The real source proposal failed current native clearance; no substitution is allowed')
            samples = 0
            for _, state in sample_schedule([curve], initial, self.policy['sample_dt_sec']):
                self.compute_checkpoint()
                require(self.validity_of(state).valid, 'The real source proposal failed current MoveIt validity')
                samples += 1
            self.compute_checkpoint()
        encoded = json.dumps(curve.record(), sort_keys=True, separators=(',', ':'), allow_nan=False).encode()
        self.saved_proposal = dict(request_id=self.active.data['request_id'], group=group,
            target=deepcopy(target), initial=deepcopy(initial), trajectory=deepcopy(trajectory))
        self.active.data['verification_injection'].update(
            source='Successful MoveIt proposal withheld once to exercise candidate retry',
            source_request_id=self.active.data['request_id'], initial_state_rad=deepcopy(initial),
            target_rad=deepcopy(target), source_curve=curve.record(),
            source_curve_sha256=hashlib.sha256(encoded).hexdigest(),
            source_native=assessment, source_moveit_samples=samples)

    def plan(self, group, target, start_state=None):
        if self.proposal_mode is None:
            return super().plan(group, target, start_state)
        self.compute_checkpoint()
        self.proposal_calls += 1
        if self.proposal_mode == 'withhold_one_success':
            if self.proposal_calls == 1:
                self._save_real_proposal(group, target, start_state)
                self.event('verification_injected_planning_failure', error_code=MoveItErrorCodes.PLANNING_FAILED)
                raise MotionPlanningFailed(MoveItErrorCodes.PLANNING_FAILED)
            saved = self.saved_proposal
            require(saved is not None and saved['request_id'] == self.active.data['request_id']
                    and saved['group'] == group and saved['target'] == target and saved['initial'] == start_state,
                    'Saved proposal belongs to a different request, start or goal')
            require(self.proposal_calls == 2, 'A saved proposal may be reused only once; current checks refused the source')
            self.active.data.setdefault('plans', []).append(dict(
                group=group, source='verification_same_request_saved_proposal', wall_sec=None,
                server_planning_sec=None, requested_planner_id=None,
                note='No second planning RPC. The saved real path still passes all production candidate/final checks.'))
            self.event('verification_reused_same_request_proposal')
            self.compute_checkpoint()
            return deepcopy(saved['trajectory'])
        if self.proposal_mode == 'force_waypoint':
            injection = self.active.data['verification_injection']
            initial = self.active.data['initial_state_rad']
            complete_target = self.active.data['command']['target_rad']
            injection.update(
                source='First four full-group proposals explicitly fail; ordered arm proposals use actual MoveIt RPCs',
                baseline_initial_state_rad=deepcopy(initial), baseline_target_rad=deepcopy(complete_target))
            failures = injection.setdefault('injected_full_group_failures', [])
            if group == 'both_arms' and len(failures) < 4:
                require(start_state == initial and target == complete_target,
                        'Full-group proposal changed the fixed request input')
                failures.append(dict(group=group, error_code=MoveItErrorCodes.PLANNING_FAILED))
                self.event('verification_injected_planning_failure', error_code=MoveItErrorCodes.PLANNING_FAILED)
                raise MotionPlanningFailed(MoveItErrorCodes.PLANNING_FAILED)
            if group in {instance.group for instance in self.cell.instances}:
                instance = next(item for item in self.cell.instances if item.group == group)
                require(set(start_state) == set(initial)
                        and target == {name:complete_target[name] for name in instance.joints},
                        'Arm proposal lost the complete virtual start or declared goal')
                calls = injection.setdefault('actual_arm_calls', [])
                expected_start = dict(initial)
                for previous in calls:
                    expected_start.update(previous['actual_endpoint_rad'])
                require(start_state == expected_start, 'Arm proposal did not start from the preceding actual segment endpoint')
                trajectory = super().plan(group, target, start_state)
                curve = Spline.from_ros(trajectory)
                encoded = json.dumps(curve.record(), sort_keys=True, separators=(',', ':'), allow_nan=False).encode()
                calls.append(dict(group=group, initial_state_rad=deepcopy(start_state), target_rad=deepcopy(target),
                    source_curve_sha256=hashlib.sha256(encoded).hexdigest(),
                    actual_endpoint_rad=dict(zip(curve.names, curve.points[-1]['p']))))
                return trajectory
            # If the actual waypoint candidate fails, later genuine RRT output is
            # retained and the selected-attempt assertion below fails honestly.
            return super().plan(group, target, start_state)
        if self.proposal_mode == 'cancel_after_failed_proposal':
            if self.proposal_calls == 1:
                self.event('verification_injected_planning_failure', error_code=MoveItErrorCodes.PLANNING_FAILED)
                raise MotionPlanningFailed(MoveItErrorCodes.PLANNING_FAILED)
            require(self.proposal_calls == 2, 'Cancellation did not interrupt the candidate loop')
            self.event('verification_candidate_cancel_published')
            self.cancel_publisher.publish(String(data=json.dumps(dict(request_id=self.active.data['request_id']))))
            self.until(lambda:self.cancel_requested, 5, 'candidate-loop cancellation topic receipt')
            self.compute_checkpoint()
            raise RuntimeError('Cancellation checkpoint unexpectedly allowed another proposal')
        raise RuntimeError('Unknown verification-only proposal mode')

    def submit(self, target, *, case, expected, mode=None, limit=12, budget=30.0, maintenance=False):
        payload = command(target, 'coordinated')
        self.proposal_mode, self.proposal_calls, self.saved_proposal = mode, 0, None
        self.max_plan_candidates, self.compute_budget_sec = limit, budget
        try:
            self.command_publisher.publish(String(data=json.dumps(payload)))
            self.until(lambda:self.pending is not None or payload['request_id'] in self.replies, 5, 'replanning command topic receipt')
            require(self.pending is not None and self.pending.data['request_id'] == payload['request_id'],
                    'Verification command was rejected or another request owns the server')
            if mode:
                self.pending.data['verification_injection'] = dict(
                    mode=mode, scope='Verification-only proposal injection; no production command field changed')
            self.process_next()
        finally:
            self.proposal_mode, self.saved_proposal = None, None
            self.max_plan_candidates, self.compute_budget_sec = 12, 30.0
        path = self.directory/(payload['request_id']+'.json')
        record = json.loads(path.read_text())
        row = dict(case=case, expected=expected, status=record['status'], request_id=payload['request_id'],
            trace_file=path.name, trace_sha256=file_sha256(path), dispatched_goals=record['dispatched_goals'],
            failure_kind=record.get('failure_kind'), failure_stage=record.get('failure_stage'),
            plan_selection=record.get('plan_selection'), verification_injection=record.get('verification_injection'),
            compute_budget=record.get('compute_budget'), error=record['error'])
        self.result_record['maintenance' if maintenance else 'checks'].append(row)
        self.save_checks()
        require(record['status'] == expected, f'{case}: unexpected {record["status"]}: {record["error"]}')
        require(not self.blocked and not self.contact_guard['latched'], 'Execution/interlock failure; stop the regression')
        stamps = [event['wall_sec'] for event in record['events']]
        require(stamps == sorted(stamps), 'Request timeline is not monotonic')
        if expected == 'succeeded':
            require(record['dispatched_goals'] == 2 and len(record['action_results']) == 2
                    and all(result['status'] == 4 and result['error_code'] == 0 for result in record['action_results']),
                    'Successful replan did not complete both physical actions')
            require(record['metrics']['received_to_first_motion_wall_sec'] is not None,
                    'Successful replan has no observed physical motion')
            require(record['planned_clearance']['complete'] and not record['compute_budget']['exceeded'],
                    'Successful request has unfinished validation or an exhausted budget')
        else:
            require(record['dispatched_goals'] == 0
                    and record['metrics']['received_to_first_motion_wall_sec'] is None
                    and not record['action_results'], 'Failed/cancelled planning dispatched or moved')
        return record

    def check_waypoint_execution(self, record):
        selection = record['plan_selection']
        require(selection['attempted'] == 5 and selection['selected_attempt'] == 5,
                'The forced waypoint regression did not select candidate five')
        candidates = selection['candidates']
        require(len(candidates) == 5 and all(not item['accepted']
                and item['moveit_error_code'] == MoveItErrorCodes.PLANNING_FAILED for item in candidates[:4]),
                'The four full-group failures were not explicit verification proposals')
        chosen = candidates[4]
        require(chosen['accepted'] and chosen['proposal_kind'] == 'arm_order_waypoint'
                and chosen['arm_order'] == [instance.id for instance in self.cell.instances]
                and len(chosen['segment_trajectories']) == 2, 'Missing ordered two-segment waypoint evidence')
        injection = record['verification_injection']
        require(len(injection['injected_full_group_failures']) == 4
                and [call['group'] for call in injection['actual_arm_calls']] == [instance.group for instance in self.cell.instances],
                'Waypoint regression did not use two actual arm planning RPCs')
        first, second = [Spline(**item) for item in chosen['segment_trajectories']]
        curve = Spline(**chosen['curve'])
        require(abs(curve.duration - first.duration - second.duration) <= 1e-8,
                'Composed waypoint duration differs from its two serial segments')
        join = first.duration
        initial, waypoint = record['initial_state_rad'], chosen['waypoint_state_rad']
        first_arm, second_arm = self.cell.instances
        require(set(first.names) == set(first_arm.joints) and set(second.names) == set(second_arm.joints),
                'Waypoint segments changed arm ownership')
        def state_at(stamp):
            return dict(zip(curve.names, curve.position(stamp)))
        require(max(abs(state_at(join)[name]-position) for name, position in waypoint.items()) <= 1e-9,
                'Composed spline does not pass through the recorded full waypoint')
        for point in curve.points:
            positions = dict(zip(curve.names, point['p']))
            held = second_arm.joints if point['t'] < join else first_arm.joints
            reference = initial if point['t'] < join else waypoint
            require(max(abs(positions[name]-reference[name]) for name in held) <= 1e-9,
                    'The inactive arm is not held at its full-state waypoint')
        boundary = next((point for point in curve.points if abs(point['t']-join) <= 1e-9), None)
        require(boundary is not None and max(abs(value) for field in ('v','a') for value in boundary[field]) <= 1e-10,
                'The composed path does not stop at the intermediate waypoint')
        require(max(abs(state_at(join/2)[name]-initial[name]) for name in second_arm.joints) <= 1e-9
                and max(abs(state_at(join+second.duration/2)[name]-waypoint[name]) for name in first_arm.joints) <= 1e-9,
                'Interpolated inactive arm positions do not hold between knots')
        start = record['scheduled_start_simulation_sec']
        observed = [[], []]
        for sample in record['observations']:
            elapsed = sample['simulation_sec']-start
            if not 0 <= elapsed <= curve.duration:
                continue
            leg = 0 if elapsed < join else 1
            held = second_arm.joints if leg == 0 else first_arm.joints
            reference = initial if leg == 0 else waypoint
            observed[leg].append(max(abs(sample['state_rad'][name]-reference[name]) for name in held))
        require(all(len(values) >= 2 for values in observed), 'Missing physical observations of one serial leg')
        maximums = [max(values) for values in observed]
        require(max(maximums) < .01, 'The physically inactive arm drifted beyond the existing isolated-motion tolerance')
        evidence = dict(join_simulation_offset_sec=join, planned_hold_checked=True,
                        observed_hold_tolerance_rad=.01, observed_hold_samples=[len(values) for values in observed],
                        observed_hold_max_drift_rad=maximums)
        self.result_record['checks'][-1]['waypoint_hold_verification'] = evidence
        self.save_checks()

    def run(self):
        try:
            self.prepare()
            self.until(lambda:self.command_publisher.get_subscription_count() >= 1
                       and self.cancel_publisher.get_subscription_count() >= 1, 5, 'command/cancel subscriptions')
            original = self.state()
            target = dict(original)
            for instance in self.cell.instances:
                target[instance.joints[0]] += .12
            self.result_record.update(initial_state_rad=original, target_rad=target)
            self.save_checks()

            def reset():
                self.progress('replanning_unscored_reset')
                self.submit(original, case='reset', expected='succeeded', maintenance=True)
                self.quiet()

            self.progress('replanning_normal_actual_moveit')
            self.submit(target, case='normal_actual_moveit', expected='succeeded')
            reset()
            self.progress('replanning_retry_after_explicit_failed_proposal')
            retry = self.submit(target, case='retry_after_explicit_failed_proposal', expected='succeeded', mode='withhold_one_success')
            selection = retry['plan_selection']
            require(selection['attempted'] == 2 and selection['selected_attempt'] == 2
                    and selection['candidates'][0]['moveit_error_code'] == MoveItErrorCodes.PLANNING_FAILED
                    and selection['candidates'][1]['accepted'], 'The failed proposal did not yield to the same-request real path')
            reset()
            self.progress('replanning_candidate_cap_one')
            limited = self.submit(target, case='candidate_cap_one', expected='failed', mode='withhold_one_success', limit=1)
            require(limited.get('failure_kind') == 'candidate_exhausted'
                    and limited['plan_selection']['attempted'] == 1
                    and limited['plan_selection']['selected_attempt'] is None, 'Candidate cap was not enforced')
            self.progress('replanning_candidate_loop_topic_cancellation')
            cancelled = self.submit(target, case='candidate_loop_topic_cancellation', expected='cancelled', mode='cancel_after_failed_proposal')
            require(cancelled.get('failure_kind') == 'cancelled'
                    and any(event['name'] == 'cancel_received' for event in cancelled['events'])
                    and cancelled['plan_selection']['attempted'] == 2
                    and cancelled['plan_selection']['selected_attempt'] is None, 'Candidate cancellation was not actually received before dispatch')
            self.progress('replanning_wall_budget_exhaustion')
            exhausted = self.submit(target, case='wall_budget_exhaustion', expected='failed', budget=.01)
            require(exhausted.get('failure_kind') == 'budget_exhausted'
                    and exhausted['compute_budget']['exceeded'], 'Tiny real computation budget did not fail closed')
            self.progress('replanning_actual_arm_order_waypoint')
            waypoint = self.submit(target, case='actual_arm_order_waypoint', expected='succeeded', mode='force_waypoint')
            self.check_waypoint_execution(waypoint)
            reset()
            self.progress('replanning_normal_after_failures')
            self.submit(target, case='normal_after_failures', expected='succeeded')
            reset()
            self.result_record['status'] = 'completed'
        except BaseException as error:
            self.result_record.update(status='aborted', error=dict(type=type(error).__name__, message=str(error)))
            raise
        finally:
            self.save_checks()
        return dict(replanning_verification_path=str(self.directory/'replanning.json'),
            replanning_verification_sha256=file_sha256(self.directory/'replanning.json'),
            command_trace_directory=str(self.directory), replanning_checks=self.result_record['checks'],
            verification_injections_explicit=True, contact_guard=deepcopy(self.contact_guard))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--domain', type=int, default=159)
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    if not 0 <= args.domain <= 232:
        parser.error('ROS domain must be 0..232')
    if args.output and (args.output.exists() or not args.output.resolve().is_relative_to((ROOT/'runtime').resolve())):
        parser.error('Output must be a new directory under runtime/')
    cell = load_workcell()
    check_workcell(cell)
    require_unused_domain(args.domain)
    args.robot, args.scene, args.gui, args.stop_mode = cell.id, 'default', False, 'interrupt'
    args.require_gpu, args.screenshot_dir = False, None
    return run_verification(args, ROOT, rclpy, lambda cell, progress:ReplanningVerification(cell, progress, args.output), Trigger,
        profile_loader=lambda *a, **kw:load_workcell(root=kw['root']), validator=check_workcell,
        command_builder=lambda cell, args:['bash',str(ROOT/'run_scene.sh'),f'workcell:={cell.id}',
                                           'gui:=false','rviz:=false','planning_case:=dual_arm_crossing'],
        report_prefix='motion_replanning_', launch_environment=diagnostic_environment, finalize=finalize_physics)


if __name__ == '__main__':
    raise SystemExit(main())
