# SPDX-License-Identifier: LGPL-2.1-or-later
"""
End-to-end run of the thread Job (threading_test.FCStd): Turn Thread,
M12x1.75 6g, 8 passes + 1 spring at 29.5 degree flank infeed, on a 12.000
stock, with the 'threading' tool (60 degrees, tip radius 0.05, width 6).
"""

import json
import os
import sys
import unittest

import numpy as np

HERE = os.path.dirname(__file__)
D = os.path.join(HERE, "data", "thread")
sys.path.insert(0, os.path.join(HERE, ".."))

from camrotarysim.core import compare as C  # noqa: E402
from camrotarysim.core import thread_check as T  # noqa: E402
from camrotarysim.core.motion import parse_gcode_text  # noqa: E402
from camrotarysim.core.simulation import OpInput, Simulation  # noqa: E402
from camrotarysim.core.stock import RotaryStock  # noqa: E402
from camrotarysim.core.tool import REF_ORIGIN, ToolOutline  # noqa: E402


def _load(fn):
    with open(os.path.join(D, fn)) as f:
        return f.read()


def simulate():
    tool = ToolOutline.from_section(json.loads(_load("tool_threading_section.json"))["edges"], 0.05,
                                    reference=REF_ORIGIN)
    sd = json.loads(_load("stock.json"))
    st = RotaryStock(sd["z_min"], sd["z_max"], 0.05, sd["radius"])
    meta = json.loads(_load("op.json"))
    sim = Simulation(st, [OpInput("TurnThread", parse_gcode_text(_load("TurnThread.nc")), tool, meta=meta)])
    sim.run_to_end()
    return sim, st, tool


class TestThreadJob(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.sim, cls.st, cls.tool = simulate()
        cls.zones = T.thread_zones(cls.sim)
        cls.res = T.measure(cls.st, cls.zones[0], 3.0)

    def test_moves_and_zone(self):
        self.assertEqual(len(self.sim.thread_moves()), 9)
        self.assertEqual(self.sim.events, [])
        z = self.zones[0]
        self.assertEqual((z.pitch, z.passes), (1.75, 9))
        self.assertAlmostEqual(2 * z.deepest, 9.744, 3)
        self.assertAlmostEqual(z.z0, -56.196, 3)

    def test_measured_thread(self):
        r = self.res
        self.assertAlmostEqual(r.major, 12.000, 3)            # crests are the 12.000 stock
        self.assertAlmostEqual(r.minor, 9.744 + 2 * 0.05, 2)  # the sharp point plus the tip radius
        # the op cuts its sharp point to its root (9.744), the basic profile's rounded root d3
        # shifted for 6g; the 60 degree flanks meet H/6 deeper than d3, so the flanks are
        # H/6 high and the pitch diameter 2 * H/6 over the 10.754 it aims for
        H = 0.866025 * 1.75
        self.assertAlmostEqual(r.pitch_diameter, 10.754 + 2 * H / 6, delta=0.01)
        self.assertEqual(r.verdict, "shallow")
        self.assertEqual(r.expected_pitch, (10.679, 10.829))
        self.assertTrue(any("crests at diameter 12.000" in n for n in r.notes))

    def test_thread_excluded_from_gouge_check(self):
        pls = [e["points"] for e in json.loads(_load("part_section.json"))["edges"]]
        part = C.part_profile(pls, self.st.s)
        res = C.compare(self.st.profile(), part, self.st.s, 0.01, 0.05, None,
                        [(z.z0, z.z1) for z in self.zones])
        self.assertIsNone(res.worst("gouge"))
        self.assertTrue(np.all(res.classes == C.THREAD))

    def test_right_hand_by_default(self):
        st = self.st
        m = (st.s > -30) & (st.s < -20)
        q = st.n_theta // 4
        z0 = st.s[m][int(np.argmin(st.R[0, m]))]
        z1 = st.s[m][int(np.argmin(st.R[q, m]))]
        self.assertAlmostEqual((z1 - z0) % 1.75, 1.75 / 4, delta=0.06)


class TestG76(unittest.TestCase):
    """The same thread from LibLathe as G33 passes and as one G76 cycle (devtools/make_thread_g76_fixture.py)."""

    def run_file(self, fn):
        tool = ToolOutline.from_section(json.loads(_load("tool_threading_section.json"))["edges"], 0.05,
                                        reference=REF_ORIGIN)
        sd = json.loads(_load("stock.json"))
        st = RotaryStock(sd["z_min"], sd["z_max"], 0.05, sd["radius"])
        sim = Simulation(st, [OpInput("TurnThread", parse_gcode_text(_load(fn)), tool,
                                      meta=json.loads(_load("op.json")))])
        sim.run_to_end()
        return sim, st

    def test_g76_cuts_what_the_g33_passes_cut(self):
        sim76, st76 = self.run_file("TurnThread_G76.nc")
        sim33, st33 = self.run_file("TurnThread_G33_liblathe.nc")
        self.assertEqual(sim76.events, [])
        self.assertEqual(sim76.diagnostics, [])
        self.assertEqual(len(sim76.thread_moves()), len(sim33.thread_moves()))
        self.assertLess(float(np.max(np.abs(st76.R - st33.R))), 1e-6)
        r = T.measure(st76, T.thread_zones(sim76)[0], 3.0)
        self.assertAlmostEqual(r.pitch_diameter, 11.2592, 3)

    def test_run_out(self):
        # with an exit taper: LibLathe's explicit passes (since ffd5f7a the tapered G33's K follows the
        # move, so its lead along Z is the pitch) and the G76 (LinuxCNC's taper pitch) cut the same thread
        sim76, st76 = self.run_file("TurnThread_G76_runout.nc")
        sim33, st33 = self.run_file("TurnThread_G33_runout.nc")
        self.assertEqual(sim76.events, [])
        self.assertLess(float(np.max(np.abs(st76.R - st33.R))), 1e-6)


if __name__ == "__main__":
    unittest.main()
