"""Fixed simulation comparisons exposed by the local operator console."""
from copy import deepcopy

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
