# SPDX-License-Identifier: LGPL-2.1-or-later
"""Spindle-synchronized (G33) threading: helix, hand, start angle, retract, measurement."""

import math
import os
import sys
import unittest

import numpy as np

HERE = os.path.dirname(__file__)
sys.path.insert(0, os.path.join(HERE, ".."))

from camrotarysim.core import thread_check as T  # noqa: E402
from camrotarysim.core.simulation import OpInput, Simulation  # noqa: E402
from camrotarysim.core.stock import RotaryStock  # noqa: E402
from camrotarysim.core.tool import ToolOutline  # noqa: E402

P = 1.5
H = 0.866025 * P


def one_pass(x, spindle=1, d=None, retract=True, angles=72):
    tool = ToolOutline.from_threading(60.0, 0.0, 4.0)
    st = RotaryStock(-20.0, 0.0, 0.025, 6.0)
    g33 = {"X": x, "Z": -15.0, "K": P}
    if d is not None:
        g33["D"] = d
    cmds = [("G18", {}), ("G0", {"X": 7.0, "Z": 4.5}), ("G0", {"X": x}), ("G33", g33)]
    if retract:
        cmds.append(("G0", {"X": 7.0}))
    sim = Simulation(st, [OpInput("thread", cmds, tool, spindle=spindle)], step=0.025, thread_angles=angles)
    sim.run_to_end()
    return sim, st


def root_z(st, i, z_lo=-10.0, z_hi=-5.0):
    """Z of the deepest point near the middle, at angle index i."""
    m = (st.s > z_lo) & (st.s < z_hi)
    return st.s[m][int(np.argmin(st.R[i, m]))]


class TestThreading(unittest.TestCase):
    def test_helix_pitch_and_hand(self):
        sim, st = one_pass(5.0)
        self.assertEqual(st.n_theta, 72)
        z0 = root_z(st, 0)
        # a quarter turn on: the root has moved a quarter pitch (mod P)
        dz = (root_z(st, 18) - z0) % P
        # spindle +1 (workpiece counter-clockwise from +Z), feeding -Z: right-hand helix,
        # the groove advances toward +Z as the angle increases
        self.assertAlmostEqual(dz, P / 4, delta=0.03)
        sim, st = one_pass(5.0, spindle=-1)
        self.assertAlmostEqual((root_z(st, 18) - root_z(st, 0)) % P, 3 * P / 4, delta=0.03)

    def test_start_angle_shifts_the_helix(self):
        _, a = one_pass(5.0)
        _, b = one_pass(5.0, d=90.0)
        # starting a quarter turn past the index, the tool is a quarter pitch behind (+Z) at
        # every spindle angle, so the groove sits a quarter pitch toward +Z
        self.assertAlmostEqual((root_z(b, 0) - root_z(a, 0)) % P, P / 4, delta=0.03)

    def test_groove_depth_and_pitch(self):
        sim, st = one_pass(5.0)
        m = (st.s > -12) & (st.s < -3)
        self.assertAlmostEqual(st.R[:, m].min(), 5.0, 2)
        self.assertEqual(st.R[:, m].max(), 6.0)
        row = st.R[0, m]
        roots = st.s[m][np.r_[False, (row[1:-1] < row[:-2]) & (row[1:-1] <= row[2:]), False]]
        self.assertTrue(np.allclose(np.diff(roots), P, atol=0.03))

    def test_retract_after_g33_cuts_no_ring(self):
        sim, st = one_pass(5.0)
        # at the end of the pass the groove ends; a ring at full depth would be at r 5.0 for every angle
        j = int(np.argmin(np.abs(st.s + 15.0)))
        self.assertGreater(st.R[:, j].max(), 5.9)
        self.assertTrue(sim.moves[-1].sync_retract)

    def test_ideal_thread_measures_its_pitch_diameter(self):
        # a sharp 60 degree V cut 7H/8 deep from the major diameter (the basic profile's sharp V)
        sim, st = one_pass(6.0 - 7 * H / 8, angles=120)
        zones = T.thread_zones(sim)
        self.assertEqual(len(zones), 1)
        res = T.measure(st, zones[0], 2.0)
        self.assertAlmostEqual(res.pitch_diameter, 12.0 - 0.649519 * P, delta=0.01)
        self.assertAlmostEqual(res.major, 12.0 - 2 * H / 8 * 0, delta=0.01)  # crests stay at the stock
        # the same with the op's range around the basic value: ok; 0.2 mm shallower: shallow
        z = zones[0]
        d2 = 12.0 - 0.649519 * P
        z.meta = {"ThreadReport": "Pitch diameter %.3f-%.3f, cut to x" % (d2 - 0.05, d2 + 0.05)}
        self.assertEqual(T.measure(st, z, 2.0).verdict, "ok")
        sim, st = one_pass(6.0 - 7 * H / 8 + 0.1, angles=120)
        z2 = T.thread_zones(sim)[0]
        z2.meta = z.meta
        r2 = T.measure(st, z2, 2.0)
        self.assertEqual(r2.verdict, "shallow")
        self.assertAlmostEqual(r2.pitch_diameter - d2, 0.2, delta=0.02)

    def test_seek_with_angles_and_snapshot_budget(self):
        tool = ToolOutline.from_threading(60.0, 0.05, 4.0)
        cmds = [("G18", {}), ("G0", {"X": 7.0, "Z": 4.5})]
        for x in (5.8, 5.6, 5.4, 5.2, 5.0):
            cmds += [("G0", {"X": x}), ("G33", {"X": x, "Z": -15.0, "K": P}), ("G0", {"X": 7.0}), ("G0", {"Z": 4.5})]
        st = RotaryStock(-20.0, 0.0, 0.05, 6.0)
        # a budget of two snapshots forces thinning
        sim = Simulation(st, [OpInput("t", cmds, tool)], thread_angles=60, snapshot_budget=2 * 60 * 400 * 8 + 1)
        sim.run_to_end()
        final = st.R.copy()
        self.assertGreater(sim._snap_stride, 1)
        self.assertLessEqual(sim._snap_bytes, sim.snapshot_budget)
        sim.seek(sim.total_samples // 2)
        half = st.R.copy()
        sim.seek(0)
        self.assertEqual(st.n_theta, 1)  # back to the axisymmetric start
        sim.seek(sim.total_samples)
        self.assertTrue(np.array_equal(final, st.R))
        sim.seek(sim.total_samples // 2)
        self.assertTrue(np.array_equal(half, st.R))
        self.assertTrue(math.isfinite(sim.spindle_angle))


class TestThreadReportLines(unittest.TestCase):
    def test_minor_range_only_for_internal_threads(self):
        # an external thread's class minor range is the basic profile's d1, not the root an insert cuts:
        # it isn't compared, so it isn't printed either
        for internal, shown in ((False, False), (True, True)):
            zone = T.ThreadZone(0, "t", 1.75, -50.0, 0.0, 4.87, 9, internal=internal)
            res = T.ThreadResult(zone, expected_pitch=(10.679, 10.829), expected_major=(11.701, 11.966),
                                 expected_minor=(9.85, 10.07))
            text = "\n".join(T.format_thread(res))
            self.assertEqual("minor 9.850-10.070" in text, shown)
            self.assertIn("major 11.701-11.966", text)


if __name__ == "__main__":
    unittest.main()
