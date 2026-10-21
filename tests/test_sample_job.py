# SPDX-License-Identifier: LGPL-2.1-or-later
"""
End-to-end offline run of the sample Job (fixtures from turning_test2.FCStd).
The expected values are what the simulator found on that Job; see
devtools/run_sample_job.py for the plots that explain them.
"""

import os
import sys
import unittest

HERE = os.path.dirname(__file__)
sys.path.insert(0, os.path.join(HERE, ".."))
sys.path.insert(0, os.path.join(HERE, "..", "devtools"))

import run_sample_job as job  # noqa: E402
from camrotarysim.core import compare as C  # noqa: E402
from camrotarysim.core.simulation import Simulation  # noqa: E402


def simulate(partoff="none"):
    tool, stock, ops, polylines, _ = job.load(partoff)
    sim = Simulation(stock, ops, step=0.05)
    sim.run_to_end()
    part = C.part_profile(polylines, stock.s)
    res = C.compare(stock.profile(), part, stock.s, 0.01, 0.05, C.reachable_profile(part, stock.s, tool))
    return sim, res


class TestSampleJob(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.sim, cls.res = simulate()

    def test_runs_all_moves(self):
        self.assertTrue(self.sim.done)
        self.assertEqual(len(self.sim.moves), 209)

    def test_no_events(self):
        self.assertEqual(self.sim.events, [])

    def test_rough_reentry_gouges_the_ball(self):
        # TurnRough #85-#88 re-enters at Z-3.73 after the ball: the insert's
        # 27 deg trailing edge cuts about 0.2 mm into the ball
        g = self.res.worst("gouge")
        self.assertAlmostEqual(g.worst, -0.196, 2)
        self.assertTrue(-3.7 < g.at < -1.8)
        self.assertEqual(len([r for r in self.res.ranges if r.kind == "gouge"]), 1)

    def test_groove_front_flank_unreachable(self):
        u = self.res.worst("unreachable")
        self.assertTrue(-10.0 < u.at < -6.4)
        self.assertGreater(u.worst, 2.0)

    def test_shoulder_nose_fillet(self):
        r = [x for x in self.res.ranges if x.kind == "unreachable" and -39.4 < x.s0 < -39.0]
        self.assertTrue(r)
        self.assertLess(r[0].worst, 0.4)  # bounded by the 0.4 nose radius

    def test_zero_feed_diagnostics(self):
        f = [d for d in self.sim.diagnostics if d.kind == "feed"]
        self.assertEqual([d.index for d in f], [86, 98, 110, 122, 134])

    def test_partoff_guesses(self):
        sim, res = simulate("default")
        # LibLathe's default Partoff plunges at FinalDepth - EdgeLength, past the stock end
        self.assertNotIn("parted", [e.kind for e in sim.events])
        sim, res = simulate("fd-60")
        parted = [e for e in sim.events if e.kind == "parted"]
        self.assertEqual(len(parted), 1)
        self.assertTrue(-65.0 < parted[0].value < -60.0)


if __name__ == "__main__":
    unittest.main()
