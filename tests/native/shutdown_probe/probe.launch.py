"""Test-only production-node factory instrumentation; no controller changes."""
import os
from pathlib import Path


def generate_launch_description():
    from launch import LaunchDescription
    from launch.actions import OpaqueFunction
    from mujoco_ros2_manipulation.workcells import launch as production

    shim = Path(os.environ["MANIPULATION_SHUTDOWN_PROBE_LIBRARY"]).resolve(strict=True)
    jtc = Path(os.environ["MANIPULATION_SHUTDOWN_PROBE_JTC_LIBRARY"]).resolve(strict=True)

    def instrumented(context):
        original = production.Node
        count = 0

        def node(*args, **kwargs):
            nonlocal count
            if kwargs.get("package") == "mujoco_ros2_control" and kwargs.get("executable") == "ros2_control_node":
                count += 1
                environment = dict(kwargs.get("additional_env", {}))
                # Production deliberately replaces inherited LD_PRELOAD. Merge
                # here, on this one test-owned native Node, preserving its guard.
                guard = environment["LD_PRELOAD"]
                environment["LD_PRELOAD"] = os.pathsep.join((str(shim), str(jtc), guard))
                kwargs["additional_env"] = environment
            return original(*args, **kwargs)

        production.Node = node
        try:
            result = production.nodes(context)
        finally:
            production.Node = original
        if count != 1:
            raise RuntimeError("Probe requires exactly one owned native simulation node")
        return result

    description = production.generate_launch_description()
    opaque = [item for item in description.entities if isinstance(item, OpaqueFunction)]
    if len(opaque) != 1:
        raise RuntimeError("Production launch shape changed; inspect probe instrumentation")
    return LaunchDescription([
        OpaqueFunction(function=instrumented) if item is opaque[0] else item
        for item in description.entities
    ])
