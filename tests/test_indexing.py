# SPDX-License-Identifier: LGPL-2.1-or-later
"""Indexing between work planes: retract, rotation as a move, the tool carried between frames."""

import json
import os
import sys
import unittest

import numpy as np

HERE = os.path.dirname(__file__)
DATA = os.path.join(HERE, "data", "rotary")
sys.path.insert(0, os.path.join(HERE, ".."))

from camrotarysim.core.indexing import describe_retract, rotary_moves, swing_radius, with_indexing  # noqa: E402
from camrotarysim.core.milling import (MillingKinematics, MillTool, VoxelStock, needed_angle,  # noqa: E402
                                       placement_frame, rotation_matrix)
from camrotarysim.core.motion import parse_gcode_text  # noqa: E402
from camrotarysim.core.simulation import OpInput, Simulation  # noqa: E402
from camrotarysim.core.stock import rotary_axes_from_machine  # noqa: E402


def _read(fn):
    with open(os.path.join(DATA, fn)) as f:
        return f.read()


JOB = json.loads(_read("job.json"))
MACHINE = json.loads(_read("tormach_4-axis.fcm"))
AXIS = rotary_axes_from_machine(MACHINE)["A"]


def job_ops():
    tool = MillTool.from_cam("endmill", 5.0, 30.0)
    ops = []
    for n in JOB["order"]:
        f = JOB["frames"][n]
        R, t = placement_frame(f["position"], f["quaternion"])
        ops.append(OpInput(n, parse_gcode_text(_read(n + ".nc")), tool, kinematics=MillingKinematics(AXIS, (R, t)),
                           meta={"angle": needed_angle(R[:, 2], AXIS.direction)[0]}))
    return ops


def run(retract_z):
    st = VoxelStock.from_box(JOB["stock"]["lo"], JOB["stock"]["hi"], 0.5, AXIS)
    sim = Simulation(st, with_indexing(job_ops(), retract_z), step=0.25)
    sim.run_to_end()
    return sim, st


class TestRetract(unittest.TestCase):
    def test_describe(self):
        z, note = describe_retract("G53 G0 Z-40", 35.0, 5.0)
        self.assertEqual(z, 40.0)            # machine coordinates: clear the swing instead
        self.assertIn("machine coordinates", note)
        z, note = describe_retract("G0 Z60", 35.0, 5.0)
        self.assertEqual(z, 60.0)
        z, note = describe_retract("", 35.0, 5.0)
        self.assertIsNone(z)

    def test_swing_radius_of_the_box(self):
        st = VoxelStock.from_box(JOB["stock"]["lo"], JOB["stock"]["hi"], 0.5, AXIS)
        self.assertAlmostEqual(swing_radius(st, AXIS), 25 * 2 ** 0.5, 3)    # the box's edges


class TestIndexing(unittest.TestCase):
    def test_sequence_and_carried_tool(self):
        ops = with_indexing(job_ops(), 40.0)
        self.assertEqual([o.name for o in ops], [
            "Adaptive (index A +0.0 to -90.0, retracted)", "Adaptive",
            "Adaptive001 (index A -90.0 to +0.0, retracted)", "Adaptive001",
            "Adaptive002 (index A +0.0 to +90.0, retracted)", "Adaptive002"])
        sim = Simulation(VoxelStock.from_box(JOB["stock"]["lo"], JOB["stock"]["hi"], 2.0, AXIS), ops)
        rot = rotary_moves(sim)
        self.assertEqual([(m.start[3], m.end[3]) for m in rot], [(0.0, -90.0), (-90.0, 0.0), (0.0, 90.0)])
        for m in rot:                         # the tool is up at the retract while the part turns
            self.assertEqual(m.start[2], 40.0)
            self.assertEqual(len(m.sample(0.25, 1.0)), 90)
        # each op starts where the tool is: its first move begins at the index's end (same machine point)
        for k in (1, 3, 5):
            op_moves = [m for m in sim.moves if m.op == k]
            index_end = [m for m in sim.moves if m.op == k - 1][-1].end
            kin = ops[k].kinematics
            angle = ops[k].meta["angle"]
            tip, _d = kin.to_workpiece(op_moves[0].start)
            machine = rotation_matrix(AXIS.direction, angle) @ tip
            np.testing.assert_allclose(machine, index_end[:3], atol=1e-9)

    def test_with_the_retract_nothing_hits(self):
        sim, st = run(40.355)
        self.assertEqual(sim.events, [])
        self.assertAlmostEqual(st.removed(), 4415.1, delta=2)

    def test_without_a_retract_the_turning_stock_hits_the_tool(self):
        # the tool stays at the last op's clearance, 5 mm over its face (30 from the axis); the box's
        # edges (35.4 from it) sweep through it on both indexes
        sim, st = run(None)
        hits = [e for e in sim.events if e.kind == "rapid_cut"]
        self.assertEqual([e.op_name for e in hits], ["Adaptive001 (index A -90.0 to +0.0)",
                                                     "Adaptive002 (index A +0.0 to +90.0)"])
        self.assertTrue(all("turning the rotary axis sweeps the stock into the tool" in e.message for e in hits))
        self.assertAlmostEqual(sum(e.value for e in hits), st.removed() - 4415.1, delta=2)


if __name__ == "__main__":
    unittest.main()
