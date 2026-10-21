# SPDX-License-Identifier: LGPL-2.1-or-later
"""Drilling on the spindle axis: the drill outline, the hole it cuts, the boring Job drilled."""

import json
import math
import os
import sys
import unittest

import numpy as np

HERE = os.path.dirname(__file__)
DATA = os.path.join(HERE, "data")
sys.path.insert(0, os.path.join(HERE, ".."))

from camrotarysim.core import compare as C  # noqa: E402
from camrotarysim.core import thread_check as T  # noqa: E402
from camrotarysim.core.motion import parse_gcode_text  # noqa: E402
from camrotarysim.core.simulation import OpInput, Simulation  # noqa: E402
from camrotarysim.core.stock import RotaryStock  # noqa: E402
from camrotarysim.core.tool import REF_ORIGIN, ToolOutline  # noqa: E402

DRILL = ToolOutline.from_drill(10.0, 118.0, 150.0)
POINT = 5.0 / math.tan(math.radians(59.0))


def _read(*p):
    with open(os.path.join(DATA, *p)) as f:
        return f.read()


def drill(cmds, stock=None):
    st = stock or RotaryStock(-60.0, 0.0, 0.05, 15.0)
    sim = Simulation(st, [OpInput("drill", cmds, DRILL, internal=True)])
    sim.run_to_end()
    return sim, st


class TestDrillTool(unittest.TestCase):
    def test_cam_drill_shape_section(self):
        d = json.loads(_read("drill", "tool_drill_section.json"))
        sec = ToolOutline.from_section(d["edges"], None, reference=REF_ORIGIN)
        par = ToolOutline.from_drill(3.0, 119.0, 50.0)
        self.assertEqual(sec.control_in_shape, (0.0, 0.0))   # the point, at the shape origin
        dz = np.linspace(0.01, 49.9, 300)
        self.assertLess(np.max(np.abs(sec.upper(dz) - par.upper(dz))), 1e-6)
        self.assertAlmostEqual(float(par.upper(0.5)), 0.5 * math.tan(math.radians(59.5)), 6)
        # symmetric: mirroring it for internal use changes nothing
        np.testing.assert_allclose(par.mirrored().upper(dz), par.upper(dz), atol=1e-9)


class TestDrilling(unittest.TestCase):
    def test_hole_and_point(self):
        sim, st = drill([("G18", {}), ("G0", {"X": 0.0, "Z": 3.0}), ("G1", {"Z": -30.0, "F": 1}),
                         ("G0", {"Z": 3.0})])
        self.assertEqual(sim.events, [])
        bore = st.inner_profile()
        self.assertLess(np.max(np.abs(bore[st.s > -30 + POINT + 0.1] - 5.0)), 1e-9)
        # the point's cone below the full diameter, nothing below the tip
        j = int(np.argmin(np.abs(st.s - (-30 + POINT / 2))))
        self.assertAlmostEqual(bore[j], 2.5, delta=0.05)
        self.assertEqual(bore[st.s < -30.05].max(), 0.0)
        self.assertEqual(st.profile().min(), 15.0)

    def test_canned_and_written_out_cycles_cut_the_same_hole(self):
        holes = []
        for f in ("TurnDrill_G83.nc", "TurnDrill_G83_explicit.nc", "TurnDrill_G81.nc"):
            st = RotaryStock(-113.0, 0.0, 0.05, 15.0)
            st.R[:, st.s > -3.0] = 0.0     # faced to Z -3 first, as in the boring Job
            sim, st = drill(parse_gcode_text(_read("drill", f)), st)
            self.assertEqual(sim.events, [], f)
            self.assertEqual(sim.diagnostics, [], f)
            holes.append(st.inner_profile().copy())
        for h in holes[1:]:
            np.testing.assert_allclose(h, holes[0], atol=1e-9)
        # through: the full diameter reaches the back (Turn Drill's default, the point goes on past it)
        self.assertAlmostEqual(holes[0][0], 5.0, 9)

    def test_rapid_into_solid(self):
        sim, st = drill([("G18", {}), ("G0", {"X": 0.0, "Z": 3.0}), ("G0", {"Z": -10.0})])
        self.assertEqual([e.kind for e in sim.events], ["rapid_cut"])


class TestDrilledBoringJob(unittest.TestCase):
    """boring_test.FCStd with a Turn Drill (10 mm, G83, through) instead of a starting hole."""

    def test_same_result_as_the_starting_hole(self):
        spec = json.loads(_read("boring", "ops.json"))
        iso = ToolOutline.from_section(json.loads(_read("boring", "tool_iso_section.json"))["edges"], 0.4)
        thr = ToolOutline.from_section(json.loads(_read("boring", "tool_threading_section.json"))["edges"], 0.05,
                                       reference=REF_ORIGIN)
        sd = json.loads(_read("boring", "stock.json"))
        st = RotaryStock(sd["z_min"], sd["z_max"], 0.05, sd["radius"])   # no hole: the drill makes it
        ops = []
        for n in ("TurnFace", "TurnDrill", "TurnProfile", "TurnRough", "TurnThread"):
            if n == "TurnDrill":
                ops.append(OpInput(n, parse_gcode_text(_read("drill", "TurnDrill_G83.nc")), DRILL, internal=True))
                continue
            o = spec["ops"][n]
            ops.append(OpInput(n, parse_gcode_text(_read("boring", n + ".nc")), iso if o["tool"] == "iso" else thr,
                               internal=o["internal"], meta=o))
        sim = Simulation(st, ops, thread_angles=120)
        sim.run_to_end()
        self.assertEqual(sim.events, [])
        pl = [e["points"] for e in json.loads(_read("boring", "part_section.json"))["edges"]]
        pin, pout = C.part_inner_profile(pl, st.s), C.part_profile(pl, st.s)
        self.assertEqual(C.compare(st.outer_profile(), pout, st.s, 0.01, 0.05).ranges, [])
        self.assertIsNone(C.compare_bore(st.inner_profile(), pin, pout, st.s).worst("gouge"))
        z = T.thread_zones(sim)[0]
        self.assertAlmostEqual(T.measure(st, z, 3.0).pitch_diameter, 19.949, delta=0.005)


if __name__ == "__main__":
    unittest.main()
