# SPDX-License-Identifier: LGPL-2.1-or-later
import os
import sys
import unittest

import numpy as np

HERE = os.path.dirname(__file__)
DATA = os.path.join(HERE, "data")
sys.path.insert(0, os.path.join(HERE, ".."))

from camrotarysim.core.kinematics import TurningKinematics  # noqa: E402
from camrotarysim.core.simulation import OpInput, Simulation  # noqa: E402
from camrotarysim.core.stock import RotaryAxis, RotaryStock  # noqa: E402
from camrotarysim.core.tool import ToolOutline  # noqa: E402


def pass_ops(tool, *cmd_lists):
    return [OpInput("op%d" % i, c, tool) for i, c in enumerate(cmd_lists)]


class TestTurning(unittest.TestCase):
    def setUp(self):
        self.tool = ToolOutline.from_iso(5.0, 60.0, 0.4, 57.0)

    def stock(self, r=20.0):
        return RotaryStock(-50.0, 0.0, 0.05, r)

    def test_straight_pass_leaves_programmed_radius(self):
        st = self.stock()
        ops = pass_ops(self.tool, [("G0", {"X": 25.0, "Z": 3.0}), ("G0", {"X": 15.0}),
                                   ("G1", {"Z": -30.0, "F": 1}), ("G0", {"X": 25.0})])
        sim = Simulation(st, ops)
        sim.run_to_end()
        prof = st.profile()
        mid = (st.s > -29.0) & (st.s < -1.0)
        self.assertLess(np.max(np.abs(prof[mid] - 15.0)), 1e-3)
        self.assertTrue(np.all(prof[st.s < -31.0] == 20.0))
        self.assertFalse(sim.events)

    def test_face_to_centre(self):
        st = self.stock()
        ops = pass_ops(self.tool, [("G0", {"X": 25.0, "Z": -1.0}), ("G1", {"X": -0.4, "F": 1})])
        Simulation(st, ops).run_to_end()
        self.assertLess(st.profile()[st.s > -0.9].max(), 1e-3)
        self.assertEqual(st.profile()[st.s < -1.1].min(), 20.0)

    def test_rapid_into_stock_is_reported(self):
        st = self.stock()
        ops = pass_ops(self.tool, [("G0", {"X": 25.0, "Z": -5.0}), ("G0", {"X": 18.0})])
        sim = Simulation(st, ops)
        sim.run_to_end()
        self.assertEqual([e.kind for e in sim.events], ["rapid_cut"])
        self.assertAlmostEqual(sim.events[0].value, 2.0, 2)

    def test_holder_collision_on_deep_pass(self):
        st = self.stock()
        # 10 mm deep in one pass with a 6.7 mm tall insert
        ops = pass_ops(self.tool, [("G0", {"X": 25.0, "Z": 3.0}), ("G0", {"X": 10.0}),
                                   ("G1", {"Z": -20.0, "F": 1})])
        sim = Simulation(st, ops)
        sim.run_to_end()
        kinds = [e.kind for e in sim.events]
        self.assertIn("holder_collision", kinds)

    def test_thin_wafer_is_not_a_collision(self):
        st = self.stock()
        # passes stop 0.01 mm short of a shoulder, then the finish pass reaches it
        cmds = [("G0", {"X": 25.0, "Z": 3.0})]
        for x in (19.0, 18.0, 17.0, 16.0, 15.0, 14.0, 13.0):
            cmds += [("G0", {"X": x}), ("G1", {"Z": -20.0, "F": 1}), ("G0", {"X": 25.0}), ("G0", {"Z": 3.0})]
        cmds += [("G0", {"X": 13.0}), ("G1", {"Z": -20.01, "F": 1}), ("G1", {"X": 21.0})]
        sim = Simulation(st, pass_ops(self.tool, cmds))
        sim.run_to_end()
        self.assertFalse([e for e in sim.events if e.kind == "holder_collision"])

    def test_parting_event(self):
        st = self.stock()
        ops = pass_ops(self.tool, [("G0", {"X": 25.0, "Z": -30.0}), ("G1", {"X": 0.0, "F": 1})])
        sim = Simulation(st, ops)
        sim.run_to_end()
        self.assertIn("parted", [e.kind for e in sim.events])

    def test_seek_backwards_and_forwards(self):
        cmds = [("G0", {"X": 25.0, "Z": 3.0})]
        for x in (19.0, 17.0, 15.0):
            cmds += [("G0", {"X": x}), ("G1", {"Z": -20.0, "F": 1}), ("G0", {"X": 25.0}), ("G0", {"Z": 3.0})]
        st = self.stock()
        sim = Simulation(st, pass_ops(self.tool, cmds))
        sim.run_to_end()
        final = st.R.copy()
        sim.seek(sim.total_samples // 3)
        third = st.R.copy()
        sim.seek(sim.total_samples)
        self.assertTrue(np.array_equal(final, st.R))
        sim.seek(sim.total_samples // 3)
        self.assertTrue(np.array_equal(third, st.R))
        # stepping move by move reaches the same end state
        sim.reset()
        while not sim.done:
            sim.advance_to_move_end()
        self.assertTrue(np.array_equal(final, st.R))

    def test_reversed_axis(self):
        st = RotaryStock(0.0, 50.0, 0.05, 20.0, axis=RotaryAxis(direction=(0, 0, -1)))
        kin = TurningKinematics(st.axis)
        x, s = kin.tool_frame((15.0, 0.0, -10.0))
        self.assertEqual((x, s), (15.0, 10.0))


if __name__ == "__main__":
    unittest.main()
