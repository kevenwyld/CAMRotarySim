# SPDX-License-Identifier: LGPL-2.1-or-later
"""Bores: the stock's inner radius, internal tools, the bore check, internal threads, the boring Job."""

import json
import os
import sys
import unittest

import numpy as np

HERE = os.path.dirname(__file__)
DATA = os.path.join(HERE, "data", "boring")
sys.path.insert(0, os.path.join(HERE, ".."))

from camrotarysim.core import compare as C  # noqa: E402
from camrotarysim.core import thread_check as T  # noqa: E402
from camrotarysim.core.mesh import bore_colours, column_colours, revolve  # noqa: E402
from camrotarysim.core.motion import parse_gcode_text  # noqa: E402
from camrotarysim.core.simulation import OpInput, Simulation  # noqa: E402
from camrotarysim.core.stock import RotaryStock  # noqa: E402
from camrotarysim.core.tool import REF_ORIGIN, ToolOutline  # noqa: E402

ISO = ToolOutline.from_iso(5.0, 60.0, 0.4, 45.0)


def _read(fn):
    with open(os.path.join(DATA, fn)) as f:
        return f.read()


def bore_pass(x, hole=5.0, z_end=-30.0, rapid=False):
    st = RotaryStock(-50.0, 0.0, 0.05, 15.0)
    if hole:
        st.add_hole(hole, -50.0, 0.0)
    cut = ("G0" if rapid else "G1", {"Z": z_end, "F": 1})
    cmds = [("G18", {}), ("G0", {"X": 2.0, "Z": 3.0}), ("G0", {"X": x}), cut, ("G0", {"X": 2.0})]
    sim = Simulation(st, [OpInput("bore", cmds, ISO, internal=True)])
    sim.run_to_end()
    return sim, st


class TestStockBore(unittest.TestCase):
    def test_hole_material_and_snapshot(self):
        st = RotaryStock(-10.0, 0.0, 0.5, 5.0)
        st.add_hole(2.0, -4.0, 0.0)
        self.assertEqual(st.inner_profile()[-1], 2.0)
        self.assertEqual(st.inner_profile()[0], 0.0)
        snap = st.snapshot()
        st.R[:, -2:] = 1.0             # faced below the hole: empty
        self.assertEqual(st.outer_profile()[-1], 0.0)
        st.restore(snap)
        self.assertEqual(st.outer_profile()[-1], 5.0)
        st.set_angles(8)
        self.assertEqual(st.Rin.shape, (8, st.n_s))
        st.restore(snap)
        self.assertEqual(st.Rin.shape, (1, st.n_s))

    def test_mirrored_outline(self):
        m = ISO.mirrored()
        self.assertTrue(m.internal)
        self.assertAlmostEqual(m.nose_centre[0], -0.4)
        self.assertAlmostEqual(m.nose_centre[1], 0.4)
        dz = np.linspace(0.05, 4.0, 40)
        np.testing.assert_allclose(m.upper(dz), -ISO.lower(dz), atol=1e-9)
        self.assertFalse(m.mirrored().internal)

    def test_mesh_with_bore_is_closed(self):
        st = RotaryStock(-10.0, 0.0, 0.5, 5.0)
        st.add_hole(2.0, -10.0, 0.0)
        st.R[0, 8:12] = 1.0            # cut through in the middle: two pieces
        v, t, c = revolve(st, column_colours(st), segments=16, inner_colours=bore_colours(st))
        edges = {}
        for a, b, d in t:
            for e in ((a, b), (b, d), (d, a)):
                k = tuple(sorted(e))
                edges[k] = edges.get(k, 0) + 1
        self.assertTrue(all(n == 2 for n in edges.values()))
        vol = np.einsum("ij,ij->i", v[t[:, 0]], np.cross(v[t[:, 1]], v[t[:, 2]])).sum() / 6.0
        # two pieces of 3.5 mm between column centres, a 16-gon ring of radii 5 and 2
        self.assertAlmostEqual(vol, 0.5 * 16 * np.sin(2 * np.pi / 16) * (25 - 4) * 7.0, 6)


class TestBoring(unittest.TestCase):
    def test_bore_pass_reaches_programmed_radius(self):
        sim, st = bore_pass(9.0)
        m = (st.s > -29.0) & (st.s < -1.0)
        # an internal tool's imaginary tip is the nose's outermost point: the bore is X
        self.assertLess(np.max(np.abs(st.inner_profile()[m] - 9.0)), 2e-3)  # 24-segment nose polygon
        self.assertEqual(st.inner_profile()[st.s < -31.0].max(), 5.0)
        self.assertEqual(st.profile().min(), 15.0)
        self.assertFalse(sim.events)

    def test_boring_from_solid_hits_the_bar(self):
        sim, st = bore_pass(9.0, hole=0.0)
        self.assertIn("holder_collision", [e.kind for e in sim.events])
        self.assertIn("boring bar", [e for e in sim.events if e.kind == "holder_collision"][0].message)

    def test_rapid_inside_material(self):
        sim, st = bore_pass(7.0, rapid=True)
        e = [e for e in sim.events if e.kind == "rapid_cut"]
        self.assertEqual(len(e), 1)
        self.assertAlmostEqual(e[0].value, 2.0, 2)

    def test_facing_across_a_hole_empties_the_columns(self):
        st = RotaryStock(-20.0, 0.0, 0.05, 15.0)
        st.add_hole(5.0, -20.0, 0.0)
        cmds = [("G0", {"X": 17.0, "Z": -1.0}), ("G1", {"X": -0.4, "F": 1})]
        Simulation(st, [OpInput("face", cmds, ToolOutline.from_iso(5.0, 60.0, 0.4, 57.0))]).run_to_end()
        self.assertEqual(st.outer_profile()[st.s > -0.9].max(), 0.0)
        self.assertEqual(st.outer_profile()[st.s < -1.1].min(), 15.0)


class TestBoreCheck(unittest.TestCase):
    def test_inner_profile_of_a_blind_bore(self):
        # a part r 10, Z -20..0, with a bore r 4 down to Z -12
        pl = [[(0, -20), (10, -20), (10, 0), (4, 0), (4, -12), (0, -12)],
              [(0, -12), (-4, -12), (-4, 0), (-10, 0), (-10, -20), (0, -20)]]
        s = -20 + (np.arange(400) + 0.5) * 0.05
        pin = C.part_inner_profile(pl, s)
        self.assertEqual(pin[s > -11.9].min(), 4.0)
        self.assertEqual(pin[s < -12.1].max(), 0.0)
        pout = C.part_profile(pl, s)
        # a stock hole r 5 drilled too deep and too wide: gouge; a bore r 3: uncut
        res = C.compare_bore(np.where(s > -15, 5.0, 0.0), pin, pout, s, 0.01, 0.05)
        self.assertAlmostEqual(res.worst("gouge").worst, -5.0)    # the hole where the part is solid
        self.assertTrue(-15 < res.worst("gouge").at < -12)
        res = C.compare_bore(np.where(s > -12, 3.0, 0.0), pin, pout, s, 0.01, 0.05)
        self.assertAlmostEqual(res.worst("uncut").worst, 1.0)

    def test_reachable_bore_leaves_the_nose_fillet(self):
        # bore r 10 from the front, stepping down to r 8 at Z -20 (a shoulder facing +Z inside)
        s = -40 + (np.arange(800) + 0.5) * 0.05
        pin = np.where(s > -20, 10.0, 8.0)
        pout = np.full_like(s, 15.0)
        reach = C.reachable_bore(pin, pout, s, ISO.mirrored())
        near = (s > -20) & (s < -19.6)
        self.assertLess(reach[near][0], 9.75)       # the 0.4 nose can't fill the inside corner
        self.assertLess(np.max(np.abs(reach[s > -19.5] - 10.0)), 1e-6)
        self.assertLess(np.max(np.abs(reach[s < -20.0] - 8.0)), 1e-6)


class TestBoringJob(unittest.TestCase):
    """boring_test.FCStd: a tube from a r 15 cylinder, a ⌀10 starting hole through."""

    @classmethod
    def setUpClass(cls):
        spec = json.loads(_read("ops.json"))
        iso = ToolOutline.from_section(json.loads(_read("tool_iso_section.json"))["edges"], 0.4)
        thr = ToolOutline.from_section(json.loads(_read("tool_threading_section.json"))["edges"], 0.05,
                                       reference=REF_ORIGIN)
        sd = json.loads(_read("stock.json"))
        st = RotaryStock(sd["z_min"], sd["z_max"], 0.05, sd["radius"])
        st.add_hole(5.0, sd["z_min"], sd["z_max"])
        ops = [OpInput(n, parse_gcode_text(_read(n + ".nc")), iso if spec["ops"][n]["tool"] == "iso" else thr,
                       internal=spec["ops"][n]["internal"], meta=spec["ops"][n]) for n in spec["order"]]
        cls.sim = Simulation(st, ops, thread_angles=120)
        cls.sim.run_to_end()
        cls.st = st
        pl = [e["points"] for e in json.loads(_read("part_section.json"))["edges"]]
        cls.pin, cls.pout = C.part_inner_profile(pl, st.s), C.part_profile(pl, st.s)
        cls.zones = T.thread_zones(cls.sim)

    def test_no_events(self):
        self.assertEqual(self.sim.events, [])

    def test_outside_and_bore(self):
        st = self.st
        out = C.compare(st.outer_profile(), self.pout, st.s, 0.01, 0.05,
                        C.reachable_profile(self.pout, st.s, self.sim.ops[0].tool))
        self.assertEqual(out.ranges, [])      # faced front is empty; the OD is 0.003 over, within tolerance
        bore = C.compare_bore(st.inner_profile(), self.pin, self.pout, st.s, 0.01, 0.05,
                              C.reachable_bore(self.pin, self.pout, st.s, self.sim.ops[1].tool),
                              [(z.z0, z.z1) for z in self.zones if z.internal])
        self.assertIsNone(bore.worst("gouge"))
        # what's left at the back end: the nose, as the profile pass stops at the stock's end
        self.assertTrue(all(r.s1 < -112 for r in bore.ranges if r.kind != "thread"))

    def test_internal_thread(self):
        z = self.zones[0]
        self.assertTrue(z.internal)
        self.assertAlmostEqual(2 * z.deepest, 22.112, 3)
        r = T.measure(self.st, z, 3.0)
        H = 0.866025 * 2.5
        self.assertAlmostEqual(r.major, 22.112 - 2 * 0.05, 2)    # the sharp point minus the tip radius
        self.assertAlmostEqual(r.minor, 19.992, 2)               # crests: the bore
        # the op cuts the sharp point to its major (22.112: D shifted for 6H); a sharp V's flanks
        # meet H/8 beyond D, so the pitch diameter is 2 * H/8 under the 20.488 it aims for. The
        # groove is narrower than the ridge even at the crests, so it is extrapolated.
        self.assertAlmostEqual(r.pitch_diameter, 20.488 - 2 * H / 8, delta=0.01)
        self.assertTrue(any("extrapolated" in n for n in r.notes))
        self.assertEqual(r.verdict, "shallow")
        self.assertEqual(r.expected_minor, (19.294, 19.744))
        self.assertTrue(any("crests at diameter 19.992, outside the op's minor" in n for n in r.notes))


if __name__ == "__main__":
    unittest.main()
