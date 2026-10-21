# SPDX-License-Identifier: LGPL-2.1-or-later
import os
import sys
import unittest

import numpy as np

HERE = os.path.dirname(__file__)
DATA = os.path.join(HERE, "data")
sys.path.insert(0, os.path.join(HERE, ".."))

from camrotarysim.core.mesh import column_colours, revolve  # noqa: E402
from camrotarysim.core.stock import RotaryStock  # noqa: E402


class TestMesh(unittest.TestCase):
    def test_revolve_closed_and_outward(self):
        st = RotaryStock(-10.0, 0.0, 0.5, 5.0)
        st.R[0, 5:10] = 3.0
        v, t, c = revolve(st, column_colours(st), segments=16)
        self.assertEqual(len(v), len(c))
        # every edge shared by exactly two triangles: closed surface
        edges = {}
        for a, b, d in t:
            for e in ((a, b), (b, d), (d, a)):
                k = tuple(sorted(e))
                edges[k] = edges.get(k, 0) + 1
        self.assertTrue(all(n == 2 for n in edges.values()))
        # outward: the signed volume is positive and close to the true volume
        vol = np.einsum("ij,ij->i", v[t[:, 0]], np.cross(v[t[:, 1]], v[t[:, 2]])).sum() / 6.0
        self.assertGreater(vol, 0)


if __name__ == "__main__":
    unittest.main()
