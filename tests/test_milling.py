# SPDX-License-Identifier: LGPL-2.1-or-later
"""Milling in the workpiece frame: cutters, the distance-grid stock, surfaces, frames, the indexed rotary Job."""

import json
import math
import os
import sys
import unittest

import numpy as np

HERE = os.path.dirname(__file__)
DATA = os.path.join(HERE, "data", "rotary")
sys.path.insert(0, os.path.join(HERE, ".."))

from camrotarysim.core.milling import (MillingKinematics, MillTool, VoxelStock, compare_part,  # noqa: E402
                                       needed_angle, placement_frame, rotation_matrix)
from camrotarysim.core.motion import parse_gcode_text  # noqa: E402
from camrotarysim.core.simulation import OpInput, Simulation  # noqa: E402
from camrotarysim.core.stock import rotary_axes_from_machine  # noqa: E402
from camrotarysim.core.surface import mesh_sdf, sample, surface_nets  # noqa: E402


def _read(fn):
    with open(os.path.join(DATA, fn)) as f:
        return f.read()


def closed_volume(v, t):
    edges = {}
    for a, b, c in t:
        for e in ((a, b), (b, c), (c, a)):
            k = tuple(sorted(e))
            edges[k] = edges.get(k, 0) + 1
    closed = all(n == 2 for n in edges.values())
    return closed, np.einsum("ij,ij->i", v[t[:, 0]], np.cross(v[t[:, 1]], v[t[:, 2]])).sum() / 6.0


class TestCutters(unittest.TestCase):
    def test_flat_and_ball_distances(self):
        tip, axis = np.zeros(3), np.array([0.0, 0.0, 1.0])
        flat = MillTool("flat", 2.5, 10.0)
        ball = MillTool("ball", 2.5, 10.0)
        p = np.array([[0, 0, -1.0], [4.0, 0, 5.0], [0, 0, 5.0], [2.5, 0, -0.5]])
        np.testing.assert_allclose(flat.distance(p, tip, axis), [1.0, 1.5, -2.5, 0.5], atol=1e-12)
        # the ball's centre is a radius up the axis
        np.testing.assert_allclose(ball.distance(p[:1], tip, axis), [1.0], atol=1e-12)
        self.assertAlmostEqual(float(ball.distance(np.array([[2.5, 0, 0.0]]), tip, axis)[0]),
                               math.hypot(2.5, 2.5) - 2.5, 12)
        self.assertEqual(MillTool.from_cam("ballend", 6, 20).kind, "ball")
        self.assertIn("taken as a flat", MillTool.from_cam("bullnose", 6, 20).source)


class TestGrid(unittest.TestCase):
    def test_box_surface_and_slot(self):
        st = VoxelStock.from_box((0, -5, -5), (20, 5, 5), 0.5)
        closed, vol = closed_volume(*surface_nets(st.phi, st.lo, st.h))
        self.assertTrue(closed)
        self.assertAlmostEqual(vol, 2000.0, delta=40)          # edges chamfered by surface nets
        self.assertAlmostEqual(st.volume(), 2000.0, delta=20)   # grid points weighted by distance
        kin = MillingKinematics()
        tool = MillTool("flat", 2.5, 10.0)
        for x in np.arange(-5, 25, 0.25):                       # a 5 wide, 1 deep slot along the top
            kin.apply(st, tool, (x, 0, 4, 0, 0, 0))
        # the slot's floor is exact on the grid: phi at Z 4 + 0.5 under the slot is -0.5
        j = int(np.argmin(np.abs(st.axes[2] - 4.5)))
        self.assertAlmostEqual(float(st.phi[20, 12, j]), -0.5, 6)
        _closed, vol = closed_volume(*surface_nets(st.phi, st.lo, st.h))
        self.assertAlmostEqual(vol, 2000.0 - 100.0, delta=40)

    def test_mesh_sdf_of_a_box_mesh(self):
        st = VoxelStock.from_box((0, -5, -5), (20, 5, 5), 0.5)
        c = np.array([[x, y, z] for x in (0, 20) for y in (-5, 5) for z in (-5, 5)], float)
        F = [(0, 1, 3), (0, 3, 2), (4, 6, 7), (4, 7, 5), (0, 4, 5), (0, 5, 1), (2, 3, 7), (2, 7, 6), (0, 2, 6),
             (0, 6, 4), (1, 5, 7), (1, 7, 3)]
        phi = mesh_sdf(st.lo, st.h, st.shape, c[np.array(F)], band=3)
        m = (np.abs(st.phi) < 1.4) & (np.abs(st.phi) > 1e-6)
        self.assertLess(float(np.max(np.abs(phi[m] - st.phi[m]))), 1e-6)
        self.assertAlmostEqual(float(sample(phi, st.lo, st.h, np.array([[10.0, 0.0, 4.75]]))[0]), 0.25, 6)

    def test_gouge_on_the_grid(self):
        st = VoxelStock.from_box((0, -5, -5), (20, 5, 5), 0.5)
        part = st.phi.copy()                      # the part is the stock: anything cut is a gouge
        kin = MillingKinematics()
        tool = MillTool("flat", 2.5, 10.0)
        for x in np.arange(5, 15, 0.25):
            kin.apply(st, tool, (x, 0, 4, 0, 0, 0))
        c = compare_part(st, part, 0.01, 0.05)
        self.assertAlmostEqual(c.gouge, 1.0, delta=0.01)
        self.assertGreater(c.gouge_volume, 30)
        self.assertEqual(c.uncut, 0.0)


class TestFrames(unittest.TestCase):
    def test_placement_and_rotary_words(self):
        R, t = placement_frame((1, 2, 3), (math.sin(math.pi / 4), 0, 0, math.cos(math.pi / 4)))  # 90 about X
        np.testing.assert_allclose(R @ [0, 0, 1], [0, -1, 0], atol=1e-12)
        kin = MillingKinematics(frame=(R, t))
        tip, d = kin.to_workpiece((0, 0, 0, 0, 0, 0))
        np.testing.assert_allclose(tip, [1, 2, 3])
        np.testing.assert_allclose(d, [0, -1, 0], atol=1e-12)
        # an A word turns the workpiece: in its frame the tool turns by -A
        tip, d = MillingKinematics().to_workpiece((0, 0, 10, 90, 0, 0))
        np.testing.assert_allclose(tip, [0, 10, 0], atol=1e-9)
        np.testing.assert_allclose(d, [0, 1, 0], atol=1e-12)

    def test_needed_angle(self):
        # machine = R(axis, A) * world, as CAM's compute_rotation_matrix
        for n, a in (((0, -1, 0), -90.0), ((0, 1, 0), 90.0), ((0, 0, 1), 0.0), ((0, -1, 1), -45.0)):
            angle, err = needed_angle(n, (1, 0, 0))
            self.assertAlmostEqual(angle, a, 9)
            self.assertAlmostEqual(err, 0.0, 9)
            n = np.asarray(n, float) / np.linalg.norm(n)
            np.testing.assert_allclose(rotation_matrix((1, 0, 0), angle) @ n, [0, 0, 1], atol=1e-12)
        self.assertGreater(needed_angle((1, 0, 1), (1, 0, 0))[1], 0.5)   # tilted along A: out of reach


class TestIndexedRotaryJob(unittest.TestCase):
    """inexed_rotary_test.FCStd: Adaptive ops on work planes facing -Y, +Z and +Y."""

    @classmethod
    def setUpClass(cls):
        job = json.loads(_read("job.json"))
        axis = rotary_axes_from_machine(json.loads(_read("tormach_4-axis.fcm")))["A"]
        st = VoxelStock.from_box(job["stock"]["lo"], job["stock"]["hi"], 0.5, axis)
        tool = MillTool.from_cam("endmill", 5.0, 30.0)
        ops, cls.angles = [], []
        for n in job["order"]:
            f = job["frames"][n]
            R, t = placement_frame(f["position"], f["quaternion"])
            cls.angles.append(needed_angle(R[:, 2], axis.direction)[0])
            ops.append(OpInput(n, parse_gcode_text(_read(n + ".nc")), tool, kinematics=MillingKinematics(axis, (R, t))))
        cls.sim = Simulation(st, ops, step=0.25)
        cls.sim.run_to_end()
        cls.st = st
        cls.part = mesh_sdf(st.lo, st.h, st.shape, np.array(json.loads(_read("part_triangles.json"))["triangles"]))
        cls.ngc = job["ngc_A"]

    def test_angles(self):
        self.assertEqual([round(a, 6) for a in self.angles], [-90.0, 0.0, 90.0])
        # the posted program has A-90 for Adaptive002, whose plane faces +Y: that would turn it down
        self.assertEqual(self.ngc["Adaptive002"], -90.0)

    def test_no_events_and_removed_volume(self):
        self.assertEqual(self.sim.events, [])
        self.assertEqual(self.sim.diagnostics, [])
        self.assertAlmostEqual(self.st.volume(self.st.phi0), 200045.0, delta=1)   # the box, on the 0.5 grid
        self.assertAlmostEqual(self.st.removed(), 4415.1, delta=2)
        # nearly all of the model's voids left of X 50 (the machined end), never more
        st = self.st
        xs = st.axes[0][:, None, None]
        w0 = np.clip(st.phi0 / st.h + 0.5, 0, 1)
        wp = np.clip(self.part / st.h + 0.5, 0, 1)
        voids = float((np.clip(w0 - wp, 0, 1) * (xs < 50)).sum()) * st.h ** 3
        self.assertGreater(st.removed(), 0.97 * voids)
        self.assertLess(st.removed(), voids)

    def test_matches_the_model(self):
        c = compare_part(self.st, self.part, 0.01, 0.05)
        self.assertEqual(c.gouge, 0.0)
        # the pockets are where the model's are: where the model is empty inside the machined depth,
        # the simulated stock is too (sections through each face, 2.5 mm under it)
        for ax, value, region in ((2, 22.5, (slice(None), slice(None))), (1, -22.5, None), (1, 22.5, None)):
            i = int(np.argmin(np.abs(self.st.axes[ax] - value)))
            S, P = np.take(self.st.phi, i, axis=ax), np.take(self.part, i, axis=ax)
            empty_model = (P < -0.3) & (np.take(self.st.phi0, i, axis=ax) > 0.3)
            xs = self.st.axes[0]
            empty_model[xs > 50] = False          # the right-hand end isn't machined by these ops
            self.assertGreater(empty_model.sum(), 100)
            self.assertTrue(np.all(S[empty_model] < 0), (ax, value))


if __name__ == "__main__":
    unittest.main()
