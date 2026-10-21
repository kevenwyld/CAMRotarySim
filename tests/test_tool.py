# SPDX-License-Identifier: LGPL-2.1-or-later
import json
import math
import os
import sys
import unittest

import numpy as np

HERE = os.path.dirname(__file__)
DATA = os.path.join(HERE, "data")
sys.path.insert(0, os.path.join(HERE, ".."))

from camrotarysim.core.tool import REF_ORIGIN, ToolOutline  # noqa: E402


def _read(path):
    with open(path) as f:
        return f.read()


def t37_section():
    d = json.loads(_read(os.path.join(DATA, "tool_T37_section.json")))
    return ToolOutline.from_section(d["edges"], tip_radius=0.4)


class TestTool(unittest.TestCase):
    def test_section_matches_parameters(self):
        sec = t37_section()
        iso = ToolOutline.from_iso(5.0, 60.0, 0.4, 57.0)
        # control point in shape coordinates: nose centre (0.671, 0.436) minus (r, r)
        for a, b in zip(sec.control_in_shape, (0.27094, 0.03571)):
            self.assertAlmostEqual(a, b, 4)
        for t in (sec, iso):
            self.assertAlmostEqual(t.nose_centre[0], 0.4, 4)
            self.assertAlmostEqual(t.nose_centre[1], 0.4, 4)
        dz = np.linspace(0.01, 4.6, 200)
        self.assertLess(np.max(np.abs(sec.lower(dz) - iso.lower(dz))), 0.005)
        self.assertLess(np.max(np.abs(sec.upper(dz) - iso.upper(dz))), 0.01)

    def test_nose_envelope_is_a_circle(self):
        t = ToolOutline.from_iso(5.0, 60.0, 0.4, 57.0)
        dz = np.linspace(0.05, 0.6, 12)
        expected = 0.4 - np.sqrt(0.4 ** 2 - (dz - 0.4) ** 2)
        self.assertLess(np.max(np.abs(t.lower(dz) - expected)), 1e-3)

    def test_edge_directions(self):
        # Rotation 45, TipAngle 80 (CCMT060204R): edges at 5 and 85 deg from +Z toward +X
        t = ToolOutline.from_iso(6.35, 80.0, 0.0, 45.0)
        p = t.points + np.asarray(t.control_in_shape)  # back to shape coordinates (sharp corner at origin)
        ang = sorted(math.degrees(math.atan2(x, z)) for x, z in p if math.hypot(x, z) > 1)
        self.assertAlmostEqual(ang[0], 5.0, 6)
        self.assertAlmostEqual(ang[-1], 85.0, 6)

    def test_threading_section_matches_parameters(self):
        d = json.loads(_read(os.path.join(DATA, "thread", "tool_threading_section.json")))
        sec = ToolOutline.from_section(d["edges"], tip_radius=0.05, reference=REF_ORIGIN)
        par = ToolOutline.from_threading(60.0, 0.05, 6.0)
        self.assertEqual(sec.control_in_shape, (0.0, 0.0))
        self.assertEqual(par.control_in_shape, (0.0, 0.0))
        dz = np.linspace(-2.95, 2.95, 119)
        self.assertLess(np.max(np.abs(sec.lower(dz) - par.lower(dz))), 1e-4)
        # 60 degree flanks: x = |dz| / tan(30) away from the tip
        self.assertAlmostEqual(float(par.lower(1.0)), 1.0 / math.tan(math.radians(30)), 6)
        self.assertAlmostEqual(float(par.lower(0.0)), 0.05, 6)

    def test_no_nose_arc_fallback(self):
        sq = [{"kind": "line", "points": [(1, 1), (3, 1)]}, {"kind": "line", "points": [(3, 1), (3, 3)]},
              {"kind": "line", "points": [(3, 3), (1, 3)]}, {"kind": "line", "points": [(1, 3), (1, 1)]}]
        t = ToolOutline.from_section(sq)
        self.assertEqual(t.control_in_shape, (1.0, 1.0))


if __name__ == "__main__":
    unittest.main()
