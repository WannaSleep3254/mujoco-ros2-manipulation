"""Read-only replay of an observed request's saved, fully checked trajectory."""
from copy import deepcopy
import hashlib
import json
import math
import re
from ..workcells.interference import Spline


def _require(condition, message):
    if not condition:
        raise ValueError(message)


def _finite(value):
    try:
        return type(value) in (int, float) and math.isfinite(value)
    except OverflowError:
        return False


def _load(raw, maximum):
    _require(type(raw) is bytes and len(raw) <= maximum, 'Saved plan source exceeds its limit')
    def pairs(items):
        value = {}
        for key, item in items:
            _require(key not in value, 'Duplicate saved plan field')
            value[key] = item
        return value
    def number(text):
        value = float(text)
        _require(_finite(value), 'Nonfinite saved plan value')
        return value
    value = json.loads(raw, object_pairs_hook=pairs,
        parse_float=number,
        parse_constant=lambda _: (_ for _ in ()).throw(ValueError('Nonfinite saved plan value')))
    _require(type(value) is dict, 'Saved plan source must be an object')
    return value


class SavedPlan:
    """A detached visual replay; no ROS entities, publication, or physics steps."""
    def __init__(self, trace_raw, session_raw, *, request_id, motion_session_id,
                 model_manifest, joint_names, joint_limits, _pending_review=None):
        trace, session = _load(trace_raw, 32*1024*1024), _load(session_raw, 4*1024*1024)
        _require(all(isinstance(value, str) and re.fullmatch('[0-9a-f]{32}', value)
                     for value in (request_id, motion_session_id)), 'Invalid replay request/session identity')
        _require(type(trace.get('schema_version')) is int and trace['schema_version'] == 1
                 and type(session.get('schema_version')) is int and session['schema_version'] == 1,
                 'Unsupported saved plan source schema')
        _require(trace.get('request_id') == request_id and session.get('session_id') == motion_session_id,
            'Saved plan belongs to another observed request/session')
        _require(type(model_manifest) is dict and session.get('model_manifest') == model_manifest,
                 'Saved plan uses different model assets')
        if _pending_review is not None:
            review = trace.get('review')
            _require(type(review) is dict and trace.get('mode') == 'plan_only'
                     and review.get('state') == 'awaiting_approval' and trace.get('dispatched_goals', 0) == 0
                     and review.get('plan_request_id') == request_id and review.get('session_id') == motion_session_id
                     and review.get('plan_sha256') == _pending_review.get('plan_sha256')
                     and _pending_review.get('state') == 'awaiting_approval'
                     and _finite(_pending_review.get('remaining_sec')) and _pending_review['remaining_sec'] > 0,
                     'Pending plan identity changed or approval expired')
            payload = trace.get('review_payload')
            _require(type(payload) is dict and set(payload) == {'schema_version','plan_request_id','session_id',
                'identity','initial_state_rad','simulation_sec','command','trajectories','no_motion_required'},
                'Missing exact pending plan payload')
            digest = hashlib.sha256(json.dumps(payload, sort_keys=True, allow_nan=False,
                separators=(',', ':')).encode()).hexdigest()
            _require(digest == _pending_review['plan_sha256']
                     and payload['schema_version'] == 1 and payload['plan_request_id'] == request_id
                     and payload['session_id'] == motion_session_id and payload['command'] == trace.get('command')
                     and payload['initial_state_rad'] == trace.get('initial_state_rad')
                     and payload['trajectories'] == trace.get('trajectories')
                     and payload['no_motion_required'] is False,
                     'Displayed path differs from the exact pending plan digest')
        else:
            _require(trace.get('mode') != 'plan_only', 'Use pending-plan preview for an unexecuted plan')
        events = trace.get('events')
        _require(trace.get('status') == 'succeeded' and trace.get('error') is None
                 and type(events) is list and events and type(events[-1]) is dict
                 and events[-1].get('name') == 'terminal'
                 and _finite(events[-1].get('wall_sec')) and events[-1]['wall_sec'] >= 0,
                 'Only completed successful requests can be replayed')
        command = trace.get('command')
        _require(type(command) is dict and set(command) == {'schema_version','request_id','strategy','target_rad'}
                 and type(command.get('schema_version')) is int and command['schema_version'] == 1
                 and command.get('request_id') == request_id
                 and command.get('strategy') in ('coordinated','departure_timing'),
                 'Saved plan command belongs to another request')
        quality = trace.get('planned_clearance') or {}
        policy = session.get('policy') or {}
        _require(type(quality) is dict and type(quality.get('minimum')) is dict
                 and type(policy) is dict and type(policy.get('schema_version')) is int
                 and policy['schema_version'] == 1, 'Invalid saved path check/policy contract')
        minimum = (quality.get('minimum') or {}).get('distance_m')
        distance, reserve = policy.get('min_inter_arm_distance_m'), policy.get('tracking_reserve_m')
        _require(_finite(distance) and distance > 0 and _finite(reserve) and reserve >= 0,
                 'Incomplete or invalid saved clearance policy')
        threshold = distance + reserve
        _require(quality.get('complete') is True and _finite(minimum)
            and math.isfinite(threshold) and minimum >= threshold,
            'Saved plan lacks a complete accepted path check')
        _require(type(joint_names) in (list,tuple), 'Invalid replay joint contract')
        names = tuple(joint_names)
        _require(names and all(isinstance(name, str) and name for name in names)
                 and len(names) == len(set(names)), 'Invalid replay joint contract')
        _require(type(joint_limits) is dict and set(joint_limits) == set(names)
                 and model_manifest.get('joints') == list(names)
                 and model_manifest.get('joint_limits') == joint_limits,
                 'Replay joints/limits differ from the exact model manifest')
        for limit in joint_limits.values():
            _require(type(limit) is dict and _finite(limit.get('lower')) and _finite(limit.get('upper'))
                     and limit['lower'] < limit['upper'], 'Invalid finite replay joint limits')
        initial = trace.get('initial_state_rad')
        _require(type(initial) is dict and set(initial) == set(names), 'Incomplete saved initial state')
        _require(type(command.get('target_rad')) is dict and set(command['target_rad']) == set(names),
                 'Incomplete saved command target')
        self.initial = deepcopy(initial)
        self.limits = deepcopy(joint_limits)
        self._check(self.initial)
        self._check(command['target_rad'])
        records = trace.get('trajectories')
        _require(type(records) is list and 1 <= len(records) <= 2, 'Missing accepted execution trajectory')
        _require(trace.get('planned_trajectories') == records,
                 'Execution trajectory differs from the fully checked path')
        self.curves = []
        covered = set()
        for item in records:
            _require(type(item) is dict and set(item) == {'names','points'}, 'Invalid accepted trajectory fields')
            _require(type(item['names']) is list and all(isinstance(name,str) for name in item['names'])
                     and type(item['points']) is list, 'Invalid accepted trajectory arrays')
            for point in item['points']:
                _require(type(point) is dict and set(point) == {'t','p','v','a'}
                         and _finite(point.get('t'))
                         and all(type(point[key]) is list and all(_finite(value) for value in point[key])
                                 for key in ('p','v','a')),
                         'Invalid accepted trajectory point')
            curve = Spline(**item)
            _require(not covered.intersection(curve.names) and set(curve.names).issubset(names), 'Overlapping or unknown replay joints')
            _require(all(abs(curve.points[0]['p'][index]-self.initial[name]) <= .01
                         for index,name in enumerate(curve.names)), 'Saved trajectory start differs from initial state')
            for point in curve.points:
                state = dict(self.initial)
                state.update(zip(curve.names, point['p']))
                self._check(state)
            covered.update(curve.names)
            self.curves.append(curve)
        _require(covered == set(names), 'Replay does not cover every model joint')
        self.duration = max(curve.duration for curve in self.curves)
        _require(_finite(quality.get('duration_sec')) and abs(quality['duration_sec']-self.duration) <= 1e-8
                 and type(quality.get('samples')) is int and quality['samples'] >= 2,
                 'Saved full-path check has a different duration or no samples')
        self.positions(0)
        self.positions(self.duration)

    def _check(self, values):
        _require(set(values) == set(self.initial), 'Replay joint contract changed')
        for name, value in values.items():
            _require(_finite(value), 'Nonfinite replay joint')
            limit = self.limits[name]
            _require(limit['lower']-1e-8 <= value <= limit['upper']+1e-8, 'Replay joint exceeds model limits')

    def positions(self, stamp):
        """Hold shorter curves at their endpoint and reject interpolated overshoot.

        This per-query limit guard does not certify continuous-time geometry or
        perform a new physical/MoveIt validation of the saved plan.
        """
        _require(_finite(stamp) and 0 <= stamp <= self.duration,
            'Replay time is outside the accepted trajectory')
        result = dict(self.initial)
        for curve in self.curves:
            result.update(zip(curve.names, curve.position(min(stamp, curve.duration))))
        self._check(result)
        return result


class ReviewPlan(SavedPlan):
    """Visualize a current checked plan before dispatch; never approve or execute."""
    def __init__(self, trace_raw, session_raw, *, review, **options):
        super().__init__(trace_raw, session_raw, _pending_review=review, **options)
