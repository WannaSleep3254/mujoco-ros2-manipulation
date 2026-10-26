"""Strict simulation command contract, independent of ROS and simulator imports."""
import json
import math
import re
import uuid

COMMAND_TOPIC = "/workcell/motion/command"
CANCEL_TOPIC = "/workcell/motion/cancel"
STATUS_TOPIC = "/workcell/motion/status"
PLAN_TOPIC = "/workcell/motion/plan"
APPROVE_TOPIC = "/workcell/motion/approve"
STRATEGIES = ("coordinated", "departure_timing")
MAX_MESSAGE_BYTES = 65536


def request_id(value):
    if not isinstance(value, str) or not re.fullmatch(r"[0-9a-f]{32}", value):
        raise ValueError("request_id must be 32 lowercase hexadecimal characters")
    return value


def decode_message(text):
    if len(text.encode("utf-8")) > MAX_MESSAGE_BYTES:
        raise ValueError("Command message exceeds 64 KiB")

    def pairs(items):
        result = {}
        for key, value in items:
            if key in result:
                raise ValueError("Duplicate JSON key: " + key)
            result[key] = value
        return result

    def invalid(value):
        raise ValueError("Nonfinite JSON number: " + value)

    def number(value):
        parsed = float(value)
        if not math.isfinite(parsed):
            invalid(value)
        return parsed

    def integer(value):
        if len(value) > 64:
            raise ValueError("Unreasonably large command integer")
        return int(value)

    data = json.loads(text, object_pairs_hook=pairs, parse_constant=invalid,
                      parse_float=number, parse_int=integer)
    if not isinstance(data, dict):
        raise ValueError("Command must be a JSON object")
    return data


def validate_command(data, limits):
    if set(data) != {"schema_version", "request_id", "strategy", "target_rad"} or type(data["schema_version"]) is not int or data["schema_version"] != 1:
        raise ValueError("Expected command schema v1 and its four declared fields")
    request_id(data["request_id"])
    if data["strategy"] not in STRATEGIES:
        raise ValueError("Unknown command strategy")
    target = data["target_rad"]
    if not isinstance(target, dict) or set(target) != set(limits):
        raise ValueError("target_rad must contain every workcell joint exactly once")
    for name, value in target.items():
        if type(value) not in (int, float) or not limits[name]["lower"] <= value <= limits[name]["upper"] or not math.isfinite(value):
            raise ValueError("Nonfinite or out-of-limit target: " + name)
    return data


def command(target, strategy="coordinated", identifier=None):
    return dict(schema_version=1, request_id=identifier or uuid.uuid4().hex,
                strategy=strategy, target_rad=dict(target))


def validate_cancel(data):
    if set(data) != {"request_id"}:
        raise ValueError("Cancellation accepts only request_id")
    return request_id(data["request_id"])


def validate_approval(data):
    """Approve one server-issued plan, using a new execution request identity.

    This additive topic leaves the four-field command v1 contract unchanged.
    The digest identifies the immutable plan; it is not an authentication token.
    """
    fields = {"schema_version", "request_id", "plan_request_id", "plan_sha256", "session_id"}
    if (set(data) != fields or type(data["schema_version"]) is not int
            or data["schema_version"] != 1):
        raise ValueError("Expected approval schema v1 and its five declared fields")
    for key in ("request_id", "plan_request_id", "session_id"):
        request_id(data[key])
    if data["request_id"] == data["plan_request_id"]:
        raise ValueError("Approval requires a new execution request_id")
    if not isinstance(data["plan_sha256"], str) or not re.fullmatch(r"[0-9a-f]{64}", data["plan_sha256"]):
        raise ValueError("plan_sha256 must be a lowercase SHA-256 digest")
    return data
