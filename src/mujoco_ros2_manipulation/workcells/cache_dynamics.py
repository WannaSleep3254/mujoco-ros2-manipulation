"""Continuous first-segment gates for a reanchored cache candidate only.

Reanchoring a first position changes a JTC spline's velocity and acceleration,
even when sampled positions remain collision-free. These checks use polynomial
extrema between both knots, rather than endpoint values or geometry samples.
They do not validate the remainder of the path, contacts, freshness or execution
authority. Existing fresh-planning behavior is deliberately outside this module.
"""
import math
import re
import time

from .interference import Spline


MIN_SEGMENT_SEC = 1e-6
ROOT_REAL_TOL = 1e-9
ROOT_AMBIGUOUS_TOL = 1e-5
ROOT_BOUND_TOL = 1e-8
LIMIT_TOL = 1e-9


class CacheDynamicsRejected(ValueError):
    """A cache candidate is unsuitable; the owner may freshly plan within budget."""
    def __init__(self, message, diagnostic):
        super().__init__(message)
        self.diagnostic = diagnostic


def _finite(value):
    try:
        return type(value) in (int, float) and math.isfinite(value)
    except OverflowError:
        return False


def _limits(joint_limits, max_acceleration):
    # Bad *configuration* is not a candidate failure and must not quietly turn
    # into a cache miss/fresh fallback in the caller.
    if (type(joint_limits) is not dict or not 1 <= len(joint_limits) <= 64
            or not _finite(max_acceleration) or not 0 < max_acceleration <= 1e6):
        raise ValueError("Expected bounded rotational joint limits and positive acceleration in rad/s^2")
    checked = {}
    for name, limits in joint_limits.items():
        if (type(name) is not str or re.fullmatch(r"[A-Za-z][A-Za-z0-9_]{0,127}", name) is None
                or type(limits) is not dict or not {"lower", "upper", "velocity"}.issubset(limits)
                or not set(limits).issubset({"lower", "upper", "velocity", "effort"})
                or any(not _finite(value) or abs(value) > 1e6 for value in limits.values())
                or not limits["lower"] < limits["upper"] or limits["velocity"] <= 0
                or ("effort" in limits and limits["effort"] <= 0)):
            raise ValueError("Joint limits require finite lower/upper radians and positive rad/s velocity")
        checked[name] = {key: float(limits[key]) for key in ("lower", "upper", "velocity")}
    return checked, float(max_acceleration)


def _reject(diagnostic, kind, message):
    diagnostic.update(status="rejected", failure_kind=kind)
    raise CacheDynamicsRejected(message, diagnostic)


def _curves(records, limits, diagnostic):
    if type(records) is not list or not 1 <= len(records) <= 8:
        _reject(diagnostic, "invalid_spline", "Expected bounded cached spline records")
    curves, names = [], []
    for record in records:
        if (type(record) is not dict or set(record) != {"names", "points"}
                or type(record["names"]) is not list or not 1 <= len(record["names"]) <= 64
                or any(type(name) is not str or name not in limits for name in record["names"])
                or type(record["points"]) is not list or not 2 <= len(record["points"]) <= 2000):
            _reject(diagnostic, "invalid_spline", "Cached spline names or point count differ from the limits")
        for point in record["points"]:
            if (type(point) is not dict or set(point) != {"t", "p", "v", "a"}
                    or not _finite(point["t"])):
                _reject(diagnostic, "invalid_spline", "Expected exact finite cached spline point fields")
            for key in ("p", "v", "a"):
                values = point[key]
                if (type(values) is not list or len(values) not in
                        ((len(record["names"]),) if key == "p" else (0, len(record["names"])))
                        or any(not _finite(value) or abs(value) > 1e6 for value in values)):
                    _reject(diagnostic, "invalid_spline", "Cached spline vectors are partial, nonfinite or unbounded")
        try:
            curves.append(Spline(**record))
        except (ValueError, TypeError, KeyError, OverflowError) as error:
            _reject(diagnostic, "invalid_spline", "Cached spline contract rejected: " + str(error)[:256])
        names.extend(record["names"])
    if len(names) != len(set(names)) or set(names) != set(limits):
        _reject(diagnostic, "invalid_spline", "Cached curves must cover each configured joint exactly once")
    return curves


def _coefficients(left, right, index, duration, layout):
    """Ascending coefficients in normalized u=(t-t0)/duration, as JTC Spline."""
    p0, p1 = float(left["p"][index]), float(right["p"][index])
    delta = p1 - p0
    if not layout[0]:
        return [p0, delta]
    v0, v1 = float(left["v"][index]) * duration, float(right["v"][index]) * duration
    if not layout[1]:
        return [p0, v0, 3 * delta - 2 * v0 - v1, -2 * delta + v0 + v1]
    a0 = float(left["a"][index]) * duration ** 2
    a1 = float(right["a"][index]) * duration ** 2
    d, v, a = delta - v0 - a0 / 2, v1 - v0 - a0, a1 - a0
    return [p0, v0, a0 / 2, 10 * d - 4 * v + a / 2,
            -15 * d + 7 * v - a, 6 * d - 3 * v + a / 2]


def _extrema(coefficients, np, diagnostic):
    values = np.asarray(coefficients, dtype=np.float64)
    if not np.isfinite(values).all():
        _reject(diagnostic, "nonfinite_polynomial", "Cached first-segment polynomial is nonfinite")
    derivative = np.polynomial.polynomial.polyder(values)
    while len(derivative) > 1 and derivative[-1] == 0:
        derivative = derivative[:-1]
    points, ambiguous, uncertainties = [0., 1.], [], []
    scale = float(np.max(np.abs(derivative)))
    if len(derivative) > 1 and scale:
        normalized = derivative / scale
        try:
            with np.errstate(all="raise"):
                roots = np.roots(normalized[::-1])
        except (ValueError, FloatingPointError, np.linalg.LinAlgError) as error:
            roots = []
            uncertainties.append("root_solver_" + type(error).__name__)
        if len(roots) != len(derivative) - 1 or not np.isfinite(roots).all():
            roots = []
            uncertainties.append("incomplete_or_nonfinite_roots")
        for root in roots:
            real, imag = float(np.real(root)), abs(float(np.imag(root)))
            # A huge irrelevant root can have a large absolute cancellation
            # residual even for a nearly stationary joint. It is outside the
            # closed segment and cannot be an extremum of that segment.
            if not -ROOT_BOUND_TOL <= real <= 1 + ROOT_BOUND_TOL:
                continue
            with np.errstate(all="ignore"):
                residual = abs(np.polynomial.polynomial.polyval(root, normalized))
            if not np.isfinite(residual) or residual > 1e-7:
                uncertainties.append("in_segment_root_residual")
                continue
            if imag <= ROOT_REAL_TOL * max(1., abs(real)):
                points.append(min(1., max(0., real)))
            elif imag <= ROOT_AMBIGUOUS_TOL * max(1., abs(real)):
                # Repeated endpoint roots commonly split numerically. Include
                # their real part, but certify limits using the polynomial's
                # conservative Bernstein convex hull instead of assuming that
                # the computed roots locate its exact extrema.
                value = min(1., max(0., real))
                points.append(value)
                ambiguous.append(value)
                uncertainties.append("near_real_root")
    points = sorted(set(points))
    with np.errstate(all="raise"):
        try:
            samples = np.polynomial.polynomial.polyval(np.asarray(points), values)
        except FloatingPointError:
            _reject(diagnostic, "nonfinite_polynomial", "Cached polynomial extrema overflowed")
    if not np.isfinite(samples).all():
        _reject(diagnostic, "nonfinite_polynomial", "Cached polynomial extrema are nonfinite")
    conservative = None
    if uncertainties:
        degree = len(values) - 1
        bernstein = [sum(np.longdouble(values[k]) * math.comb(j, k) / math.comb(degree, k)
                         for k in range(j + 1)) for j in range(degree + 1)]
        padding = 64 * np.finfo(np.float64).eps * max(1., float(np.sum(np.abs(values))))
        conservative = dict(minimum=float(min(bernstein)) - padding, maximum=float(max(bernstein)) + padding)
        if not all(math.isfinite(value) for value in conservative.values()):
            _reject(diagnostic, "nonfinite_polynomial", "Cached polynomial enclosure is nonfinite")
    sampled_minimum, sampled_maximum = float(np.min(samples)), float(np.max(samples))
    minimum = conservative["minimum"] if conservative else sampled_minimum
    maximum = conservative["maximum"] if conservative else sampled_maximum
    return dict(minimum=minimum, maximum=maximum, maximum_absolute=max(abs(minimum), abs(maximum)),
                sampled_minimum=sampled_minimum, sampled_maximum=sampled_maximum,
                sampled_maximum_absolute=float(np.max(np.abs(samples))), candidates_u=points,
                ambiguous_roots_u=ambiguous, conservative_range=conservative,
                method="bernstein_enclosure" if conservative else "stationary_roots",
                uncertainty_reasons=sorted(set(uncertainties)))


def validate_reanchored_dynamics(curves_records, joint_limits, max_acceleration=1.0):
    """Validate continuous first-segment p/v/a extrema of cached spline copies.

    Limits are rotational: ``lower/upper`` radians, ``velocity`` rad/s, and the
    explicit ``max_acceleration`` rad/s^2. The current workcell's MoveIt generator
    declares a 1.0rad/s^2 maximum; callers must supply a different checked limit
    if that configuration changes. This checks that declared cap, not a claim
    that the planner's velocity/acceleration scaling factors were achieved.

    A candidate failure raises ``CacheDynamicsRejected`` with ``diagnostic``;
    malformed limit/configuration input raises plain ValueError. No transport,
    cancellation or budget exception is translated into a cache miss.
    """
    started = time.monotonic_ns()
    limits, acceleration = _limits(joint_limits, max_acceleration)
    diagnostic = dict(schema_version=1, status="running", failure_kind=None,
        scope="continuous_first_segment_only", interpolation="JTC linear/cubic/quintic Spline",
        max_acceleration_rad_s2=acceleration, minimum_segment_sec=MIN_SEGMENT_SEC,
        units=dict(position="rad", velocity="rad/s", acceleration="rad/s^2", time="s"),
        method="normalized-time polynomial; real stationary roots or conservative whole-segment Bernstein enclosure",
        peak_summary="Upper bounds for processed joints; require summary_complete. Sampled extrema are reported separately",
        summary_complete=False,
        numerical_limit_tolerance=LIMIT_TOL, roots_real_tolerance=ROOT_REAL_TOL,
        roots_ambiguous_tolerance=ROOT_AMBIGUOUS_TOL,
        joints=[], violations=[], numerical_uncertainties=[], maximum_velocity_rad_s=0., maximum_acceleration_rad_s2=0.,
        maximum_sampled_velocity_rad_s=0., maximum_sampled_acceleration_rad_s2=0.,
        maximum_velocity_ratio=0., maximum_acceleration_ratio=0., ratio_overflow=False, elapsed_wall_sec=None,
        full_path_revalidation_required=True)
    try:
        curves = _curves(curves_records, limits, diagnostic)
        import numpy as np
        for curve_index, curve in enumerate(curves):
            left, right = curve.points[:2]
            duration = float(right["t"] - left["t"])
            if not math.isfinite(duration) or duration < MIN_SEGMENT_SEC:
                _reject(diagnostic, "segment_duration_too_small", "Cached first segment is below the 1us numerical bound")
            for index, name in enumerate(curve.names):
                coefficients = _coefficients(left, right, index, duration, curve.layout)
                position = _extrema(coefficients, np, diagnostic)
                velocity_coefficients = np.polynomial.polynomial.polyder(coefficients) / duration
                acceleration_coefficients = np.polynomial.polynomial.polyder(coefficients, 2) / duration ** 2
                velocity = _extrema(velocity_coefficients, np, diagnostic)
                acc = _extrema(acceleration_coefficients, np, diagnostic)
                bound = limits[name]
                row = dict(joint=name, curve_index=curve_index, segment_duration_sec=duration,
                    position_min_rad=position["minimum"], position_max_rad=position["maximum"],
                    velocity_max_abs_rad_s=velocity["maximum_absolute"],
                    acceleration_max_abs_rad_s2=acc["maximum_absolute"],
                    sampled_extrema=dict(position_min_rad=position["sampled_minimum"],
                        position_max_rad=position["sampled_maximum"],
                        velocity_max_abs_rad_s=velocity["sampled_maximum_absolute"],
                        acceleration_max_abs_rad_s2=acc["sampled_maximum_absolute"]),
                    position_candidates_u=position["candidates_u"], velocity_candidates_u=velocity["candidates_u"],
                    acceleration_candidates_u=acc["candidates_u"], limits=dict(bound, acceleration=acceleration),
                    ambiguous_roots_u=dict(position=position["ambiguous_roots_u"], velocity=velocity["ambiguous_roots_u"],
                                           acceleration=acc["ambiguous_roots_u"]),
                    conservative_ranges=dict(position=position["conservative_range"], velocity=velocity["conservative_range"],
                                             acceleration=acc["conservative_range"]),
                    extrema_methods=dict(position=position["method"], velocity=velocity["method"], acceleration=acc["method"]),
                    root_uncertainty_reasons=dict(position=position["uncertainty_reasons"],
                        velocity=velocity["uncertainty_reasons"], acceleration=acc["uncertainty_reasons"]))
                diagnostic["joints"].append(row)
                diagnostic["maximum_velocity_rad_s"] = max(diagnostic["maximum_velocity_rad_s"], velocity["maximum_absolute"])
                diagnostic["maximum_acceleration_rad_s2"] = max(diagnostic["maximum_acceleration_rad_s2"], acc["maximum_absolute"])
                diagnostic["maximum_sampled_velocity_rad_s"] = max(diagnostic["maximum_sampled_velocity_rad_s"], velocity["sampled_maximum_absolute"])
                diagnostic["maximum_sampled_acceleration_rad_s2"] = max(diagnostic["maximum_sampled_acceleration_rad_s2"], acc["sampled_maximum_absolute"])
                for key, numerator, denominator in (("maximum_velocity_ratio", velocity["maximum_absolute"], bound["velocity"]),
                                                    ("maximum_acceleration_ratio", acc["maximum_absolute"], acceleration)):
                    ratio = numerator / denominator
                    if not math.isfinite(ratio):
                        diagnostic[key], diagnostic["ratio_overflow"] = None, True
                    elif diagnostic[key] is not None:
                        diagnostic[key] = max(diagnostic[key], ratio)
                for condition, actual, limit, kind in (
                        (position["sampled_minimum"] < bound["lower"] - LIMIT_TOL, position["sampled_minimum"], bound["lower"], "position_lower"),
                        (position["sampled_maximum"] > bound["upper"] + LIMIT_TOL, position["sampled_maximum"], bound["upper"], "position_upper"),
                        (velocity["sampled_maximum_absolute"] > bound["velocity"] + LIMIT_TOL, velocity["sampled_maximum_absolute"], bound["velocity"], "velocity"),
                        (acc["sampled_maximum_absolute"] > acceleration + LIMIT_TOL, acc["sampled_maximum_absolute"], acceleration, "acceleration")):
                    if condition:
                        diagnostic["violations"].append(dict(joint=name, curve_index=curve_index,
                                                             kind=kind, actual=actual, limit=limit))
                for kind, extrema, lo, hi in (("position", position, bound["lower"], bound["upper"]),
                                              ("velocity", velocity, -bound["velocity"], bound["velocity"]),
                                              ("acceleration", acc, -acceleration, acceleration)):
                    conservative = extrema["conservative_range"]
                    if conservative and (conservative["minimum"] < lo - LIMIT_TOL
                                         or conservative["maximum"] > hi + LIMIT_TOL):
                        diagnostic["numerical_uncertainties"].append(dict(joint=name, kind=kind,
                            conservative_range=conservative, lower=lo, upper=hi))
        diagnostic["summary_complete"] = True
        if diagnostic["violations"]:
            _reject(diagnostic, "continuous_joint_limits", "Reanchored cached first segment exceeds continuous joint limits")
        if diagnostic["numerical_uncertainties"]:
            _reject(diagnostic, "root_solver_uncertain", "Polynomial enclosure cannot certify uncertain extrema within joint limits")
        diagnostic["status"] = "passed"
        return diagnostic
    finally:
        diagnostic["elapsed_wall_sec"] = (time.monotonic_ns() - started) / 1e9
