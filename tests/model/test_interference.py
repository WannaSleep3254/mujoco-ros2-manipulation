"""The fixed crossing has safe endpoints/isolated paths and a real cross-arm contact."""
import unittest
import mujoco
from mujoco_ros2_manipulation.workcells.config import load_workcell
from mujoco_ros2_manipulation.workcells.integrity import check_workcell
from mujoco_ros2_manipulation.workcells.interference import load_case


class CrossingModelTests(unittest.TestCase):
    def test_fixed_case_has_isolated_free_paths_and_inter_arm_midpoint_contact(self):
        cell = load_workcell()
        if not (cell.output / "source.json").exists():
            self.skipTest("Shared workcell not generated in this matrix job")
        check_workcell(cell)
        _, case = load_case(cell)
        model = mujoco.MjModel.from_xml_path(str(cell.output / "cell.xml"))
        data = mujoco.MjData(model)
        addresses = [model.joint(n).qposadr[0] for n in cell.joints]

        def contacts(state):
            data.qpos[addresses] = [state[n] for n in cell.joints]
            mujoco.mj_forward(model, data)
            return [
                (
                    model.body(model.geom_bodyid[c.geom[0]]).name,
                    model.body(model.geom_bodyid[c.geom[1]]).name,
                )
                for c in data.contact
            ]

        start, goal = case["start_rad"], case["goal_rad"]
        self.assertEqual(contacts(start), [])
        self.assertEqual(contacts(goal), [])
        for inst in cell.instances:
            for step in range(101):
                state = dict(start)
                state.update(
                    {
                        n: start[n] + (goal[n] - start[n]) * step / 100
                        for n in inst.joints
                    }
                )
                self.assertEqual(contacts(state), [])
        midpoint = {n: (start[n] + goal[n]) / 2 for n in cell.joints}
        pairs = contacts(midpoint)
        self.assertTrue(
            any(
                (a.startswith("left_") and b.startswith("right_"))
                or (b.startswith("left_") and a.startswith("right_"))
                for a, b in pairs
            )
        )
