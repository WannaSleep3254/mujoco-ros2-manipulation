"""Explicit flange references for the existing geometry; no gripper is implied.

Only zero-offset URDF tip references are currently admitted. A reference frame
does not add physical tool volume, inertia, actuation or grasp contact policy.
"""
from .pose_goal import ToolDefinition, validate_pose_goal


def configured_pose_tools(cell):
    return {instance.id: ToolDefinition(
        instance.id + "_flange_reference",
        instance.prefix + instance.profile.ros["tip_frame"],
        source="Checked robot URDF tip origin; no added gripper geometry")
        for instance in cell.instances}


def describe_pose_tools(cell):
    tools = configured_pose_tools(cell)
    return [dict(arm_id=instance.id, label=instance.profile.label,
        tool=dict(tools[instance.id].record(), sha256=tools[instance.id].sha256),
        frame_id="world", position_unit="m", orientation_format="quaternion_xyzw",
        scope="flange_reference_only") for instance in cell.instances]


def resolve_pose_tool(cell, goal):
    tools = configured_pose_tools(cell)
    goal = validate_pose_goal(goal, arm_ids=tools)
    tool = tools[goal["arm_id"]]
    if goal["tool_id"] != tool.id or goal["tool_sha256"] != tool.sha256:
        raise ValueError("Pose goal does not identify the configured flange reference")
    return goal, tool


def validate_context_identity(context, *, plan_request_id=None):
    if (type(context) is not dict or set(context) != {"kind", "goal", "tool"}
            or context["kind"] != "tcp_pose"):
        raise ValueError("Expected the exact reviewed TCP goal context")
    goal = validate_pose_goal(context["goal"])
    if plan_request_id is not None and goal["request_id"] != plan_request_id:
        raise ValueError("TCP goal belongs to another plan request")
    record = context["tool"]
    if type(record) is not dict or set(record) != {"schema_version", "id", "parent_tip",
            "translation_m", "quaternion_xyzw", "source", "scope", "sha256"}:
        raise ValueError("Expected an exact TCP reference record")
    if type(record["schema_version"]) is not int or record["schema_version"] != 1:
        raise ValueError("Expected TCP reference schema v1")
    tool = ToolDefinition(record["id"], record["parent_tip"], record["translation_m"],
                          record["quaternion_xyzw"], record["source"])
    if (record != dict(tool.record(), sha256=tool.sha256)
            or goal["tool_id"] != tool.id or goal["tool_sha256"] != tool.sha256):
        raise ValueError("TCP goal/reference digest differs")
    return goal, tool


def validate_goal_context(cell, context):
    goal, _ = validate_context_identity(context)
    goal, tool = resolve_pose_tool(cell, goal)
    if context["tool"] != dict(tool.record(), sha256=tool.sha256):
        raise ValueError("Reviewed TCP reference changed; create a new plan")
    return goal, tool
