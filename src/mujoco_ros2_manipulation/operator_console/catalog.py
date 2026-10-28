"""Fixed simulation comparisons exposed by the local operator console."""
from copy import deepcopy

from ..workcells.pose_goal import ToolDefinition, validate_pose_goal


def validate_pose_registration(config, payload):
    """Resolve one explicit coordinate reference; never infer physical tooling.

    The catalog is generated from the checked workcell. Publication also needs
    the command server to advertise this same catalog and its Pose topic.
    """
    rows = config.get("pose_tools", [])
    if type(rows) is not list or not rows:
        raise ValueError("등록된 TCP 기준점이 없습니다.")
    checked = validate_pose_goal(payload, arm_ids=[row.get("arm_id") for row in rows if type(row) is dict])
    matching = [row for row in rows if type(row) is dict and row.get("arm_id") == checked["arm_id"]
                and type(row.get("tool")) is dict and row["tool"].get("id") == checked["tool_id"]]
    if len(matching) != 1:
        raise ValueError("선택한 팔과 TCP 기준점이 등록 정보와 일치하지 않습니다.")
    row, record = matching[0], matching[0]["tool"]
    try:
        tool = ToolDefinition(record["id"], record["parent_tip"],
            record["translation_m"], record["quaternion_xyzw"], record["source"])
        expected = dict(tool.record(), sha256=tool.sha256)
    except (KeyError, TypeError, ValueError) as error:
        raise ValueError("등록한 TCP 기준점의 형식이 올바르지 않습니다.") from error
    if (record != expected or record["sha256"] != checked["tool_sha256"]
            or row.get("scope") != "flange_reference_only" or row.get("frame_id") != "world"
            or row.get("position_unit") != "m" or row.get("orientation_format") != "quaternion_xyzw"
            or tool.translation_m != (0., 0., 0.) or tool.quaternion_xyzw != (0., 0., 0., 1.)):
        raise ValueError("TCP 기준점·해시·좌표 규약이 바뀌었습니다. 등록 정보를 다시 확인하세요.")
    return checked

SCENARIOS = (
    dict(
        id="contact_comparison",
        title="장애물 접촉 정지 비교",
        description="고정 기둥 접촉 후 양팔 정지와, 기둥 없는 동일 왕복 명령의 완료를 새 실행에서 비교합니다.",
        variants=[
            dict(id="contact", label="고정 장애물 접촉 정지"),
            dict(id="control", label="무장애물 왕복"),
        ],
    ),
    dict(
        id="timing_comparison",
        title="공동 경로와 출발 시점 비교",
        description="현재 모델에서 새 경로를 계획하고, 같은 시작·목표로 공동 계획과 출발 시점 조정을 차례로 실행합니다.",
        variants=[
            dict(id="prepare", label="공동 경로 준비"),
            dict(id="baseline", label="공동 계획 경로"),
            dict(id="timing", label="출발 시점 조정"),
        ],
    ),
)


def describe_scenarios():
    """Return descriptions that callers can serialize and mutate independently."""
    return deepcopy(list(SCENARIOS))
