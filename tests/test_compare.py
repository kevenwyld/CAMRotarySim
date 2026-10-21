# SPDX-License-Identifier: LGPL-2.1-or-later
import os
import sys
import unittest

import numpy as np

HERE = os.path.dirname(__file__)
DATA = os.path.join(HERE, "data")
sys.path.insert(0, os.path.join(HERE, ".."))

from camrotarysim.core import compare as C  # noqa: E402
from camrotarysim.core.tool import ToolOutline  # noqa: E402


class TestCompare(unittest.TestCase):
    def test_part_profile_and_bulge(self):
        # half disc of radius 5 centred at Z=-5: from (0,0) over the top to (0,-10)
        pts = C.bulge_segments_to_polyline([(0.0, 0.0, 0.0, -10.0, 1.0)], ccw_positive=True)
        s = np.linspace(-9.5, -0.5, 10)
        prof = C.part_profile([pts], s)
        self.assertLess(np.max(np.abs(prof - np.sqrt(25 - (s + 5) ** 2))), 0.01)
        # the other sign goes below the axis: nothing on +X
        pts = C.bulge_segments_to_polyline([(0.0, 0.0, 0.0, -10.0, 1.0)], ccw_positive=False)
        self.assertEqual(C.part_profile([pts], s).max(), 0.0)

    def test_reachable_internal_corner(self):
        tool = ToolOutline.from_iso(5.0, 60.0, 0.4, 57.0)
        s = -40 + (np.arange(800) + 0.5) * 0.05
        part = np.where(s < -20.0, 15.0, 10.0)  # shoulder facing +Z at Z=-20
        reach = C.reachable_profile(part, s, tool)
        near = (s > -20.0) & (s < -19.6)
        # the nose leaves a fillet of radius 0.4 in the corner
        self.assertGreater(reach[near][0], 10.25)
        self.assertLess(np.max(np.abs(reach[s > -19.5] - 10.0)), 1e-6)
        self.assertLess(np.max(np.abs(reach[s < -20.0] - 15.0)), 1e-6)

    def test_classification(self):
        s = np.arange(10) * 1.0
        part = np.full(10, 10.0)
        stock = part.copy()
        stock[1] = 9.9   # gouge
        stock[4] = 10.2  # uncut, reachable
        stock[7] = 10.3  # uncut, unreachable
        reach = part.copy()
        reach[7] = 10.3
        res = C.compare(stock, part, s, 0.01, 0.05, reach)
        self.assertEqual(list(res.classes), [0, C.GOUGE, 0, 0, C.UNCUT, 0, 0, C.UNREACHABLE, 0, 0])
        self.assertAlmostEqual(res.worst("gouge").worst, -0.1)


if __name__ == "__main__":
    unittest.main()
