# SPDX-License-Identifier: LGPL-2.1-or-later
import math
import os
import sys
import unittest

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from camrotarysim.core.motion import (ARC, FEED, RAPID, MotionInterpreter,  # noqa: E402
                                      normalise_name, parse_gcode_text)


def run(cmds):
    it = MotionInterpreter()
    return it.run(cmds), it.diagnostics


class TestMotion(unittest.TestCase):
    def test_names(self):
        self.assertEqual(normalise_name("G01"), "G1")
        self.assertEqual(normalise_name("g0"), "G0")
        self.assertEqual(normalise_name("(comment)"), "")

    def test_unknown_start_is_skipped(self):
        moves, _ = run([("G0", {"X": 28.7}), ("G0", {"X": 28.7, "Z": 3.0}), ("G1", {"X": 20.0, "F": 5})])
        self.assertEqual(len(moves), 1)  # the X-only move has no Z yet; the second only sets Z
        self.assertEqual(moves[0].kind, FEED)
        self.assertEqual(moves[0].start, (28.7, 0.0, 3.0, 0.0, 0.0, 0.0))

    def test_g18_ccw_convex_ball(self):
        # the first ball-end arc of TurnProfile in the sample Job
        moves, diags = run([("G0", {"X": 13.645616, "Z": 0.0}), ("G18", {}),
                            ("G3", {"X": 13.290803, "Z": -3.793572, "I": -5.487626, "K": -1.400120, "F": 5})])
        a = moves[-1]
        self.assertEqual(a.kind, ARC)
        self.assertAlmostEqual(a.centre[0], 8.15799, 5)
        self.assertAlmostEqual(a.centre[2], -1.40012, 5)
        self.assertGreater(a.sweep, 0)  # CCW
        self.assertAlmostEqual(math.degrees(a.sweep), 114.99983 - 75.68685, 3)
        pts = a.sample(0.01)
        # the arc bulges outward: the midpoint lies above the chord
        mid = pts[len(pts) // 2]
        self.assertGreater(mid[0], 13.645616)
        r = np.hypot(pts[:, 0] - a.centre[0], pts[:, 2] - a.centre[2])
        self.assertLess(np.ptp(r), 1e-6)
        self.assertFalse([d for d in diags if d.kind == "arc"])

    def test_g18_cw(self):
        # quarter circle, centre (0,0): from Z=5,X=0 to Z=0,X=5 is CCW; G2 must go the long way
        moves, _ = run([("G0", {"X": 0.0, "Z": 5.0}), ("G18", {}), ("G2", {"X": 5.0, "Z": 0.0, "K": -5.0, "F": 1})])
        self.assertAlmostEqual(math.degrees(moves[-1].sweep), -270.0, 6)
        moves, _ = run([("G0", {"X": 0.0, "Z": 5.0}), ("G18", {}), ("G3", {"X": 5.0, "Z": 0.0, "K": -5.0, "F": 1})])
        self.assertAlmostEqual(math.degrees(moves[-1].sweep), 90.0, 6)

    def test_g17_default_plane(self):
        moves, _ = run([("G0", {"X": 5.0, "Y": 0.0, "Z": 0.0}), ("G3", {"X": 0.0, "Y": 5.0, "I": -5.0, "F": 1})])
        self.assertEqual(moves[-1].plane, (0, 1, 2))
        self.assertAlmostEqual(math.degrees(moves[-1].sweep), 90.0, 6)

    def test_r_word_arc(self):
        moves, _ = run([("G0", {"X": 0.0, "Z": 5.0}), ("G18", {}), ("G3", {"X": 5.0, "Z": 0.0, "R": 5.0, "F": 1})])
        c = moves[-1].centre
        self.assertAlmostEqual(c[0], 0.0, 6)
        self.assertAlmostEqual(c[2], 0.0, 6)

    def test_full_circle(self):
        moves, _ = run([("G0", {"X": 5.0, "Z": 0.0}), ("G18", {}), ("G2", {"X": 5.0, "Z": 0.0, "I": -5.0, "F": 1})])
        self.assertAlmostEqual(abs(moves[-1].sweep), 2 * math.pi, 9)

    def test_radius_mismatch_diagnostic(self):
        _, diags = run([("G0", {"X": 0.0, "Z": 5.0}), ("G18", {}), ("G3", {"X": 5.1, "Z": 0.0, "K": -5.0, "F": 1})])
        self.assertTrue([d for d in diags if d.kind == "arc"])

    def test_zero_feed_and_g7_diagnostics(self):
        _, diags = run([("G7", {}), ("G0", {"X": 1.0, "Z": 1.0}), ("G1", {"X": 0.0, "F": 0.0})])
        kinds = sorted(d.kind for d in diags)
        self.assertEqual(kinds, ["feed", "unsupported"])

    def test_state_carries_across_ops(self):
        it = MotionInterpreter()
        it.run([("G0", {"X": 10.0, "Z": 3.0})], 0)
        m = it.run([("G1", {"Z": -1.0, "F": 1})], 1)
        self.assertEqual(m[0].start[:3], (10.0, 0.0, 3.0))
        self.assertEqual(m[0].op, 1)

    def test_sampling_spacing(self):
        moves, _ = run([("G0", {"X": 0.0, "Z": 0.0}), ("G1", {"X": 0.0, "Z": -1.0, "F": 1})])
        pts = moves[-1].sample(0.05)
        self.assertEqual(len(pts), 20)
        self.assertAlmostEqual(pts[-1][2], -1.0)

    def test_rotary_words_interpolate(self):
        moves, _ = run([("G0", {"X": 0.0, "Y": 0.0, "Z": 10.0, "A": 0.0}), ("G0", {"A": 90.0})])
        self.assertEqual(moves[-1].kind, RAPID)
        pts = moves[-1].sample(1.0, angle_step=1.0)
        self.assertEqual(len(pts), 90)

    def test_g33_thread_moves(self):
        moves, diags = run([("G18", {}), ("G0", {"X": 7.0, "Z": 5.0}), ("G0", {"X": 5.7}),
                            ("G33", {"X": 5.7, "Z": -20.0, "K": 1.75}),
                            ("G1", {"X": 5.7, "Z": -20.0, "F": 1.0}),   # the addon's zero-length display move
                            ("G0", {"X": 7.0}), ("G33", {"Z": -30.0, "K": 2.0, "D": -90.0}),
                            ("G33", {"Z": -40.0}), ("G76", {"P": 1.5})])
        kinds = [m.kind for m in moves]
        self.assertEqual(kinds, ["rapid", "thread", "rapid", "thread"])
        self.assertEqual(moves[1].pitch, 1.75)
        self.assertTrue(moves[2].sync_retract)
        self.assertFalse(moves[0].sync_retract)
        self.assertEqual(moves[3].start_angle, 90.0)  # D's magnitude, as LinuxCNC uses it
        msgs = [d.message for d in diags]
        self.assertTrue(any("positive K" in m for m in msgs))
        self.assertTrue(any("G76" in m for m in msgs))

    def test_consecutive_g33_continue_the_helix(self):
        moves, _ = run([("G18", {}), ("G0", {"X": 5.0, "Z": 10.0}),
                        ("G33", {"Z": 0.0, "K": 2.0, "D": 30.0}),        # 5 turns
                        ("G1", {"Z": 0.0, "F": 1.0}),                    # zero length: no move, sync kept
                        ("G33", {"X": 6.0, "Z": -1.0, "K": 2.0}),        # continues: starts 5 turns on
                        ("G0", {"X": 8.0}), ("G0", {"Z": 10.0}), ("G0", {"X": 5.0}),
                        ("G33", {"Z": 0.0, "K": 2.0, "D": 30.0})])        # after rapids: a fresh sync
        th = [m for m in moves if m.kind == "thread"]
        self.assertEqual(th[0].start_angle, 30.0)
        self.assertAlmostEqual(th[1].start_angle, 30.0 + 5 * 360.0, 9)
        self.assertEqual(th[2].start_angle, 30.0)

    def test_canned_cycles(self):
        def z_path(cmds):
            moves, diags = run(cmds)
            return [(m.kind, round(m.end[2], 3)) for m in moves], diags

        start = [("G0", {"X": 0.0, "Y": 0.0, "Z": 5.0}), ("G17", {})]
        # G81 under G98, old Z (5) above R (1): back to old Z
        path, _ = z_path(start + [("G98", {}), ("G81", {"X": 0.0, "Y": 0.0, "Z": -10.0, "R": 1.0, "F": 2})])
        self.assertEqual(path, [("rapid", 1.0), ("feed", -10.0), ("rapid", 5.0)])
        # G99: back to R; a second G81 in the series keeps R (sticky) and the series' old Z
        path, _ = z_path(start + [("G99", {}), ("G81", {"Z": -10.0, "R": 1.0, "F": 2}), ("G81", {"Z": -4.0})])
        self.assertEqual(path, [("rapid", 1.0), ("feed", -10.0), ("rapid", 1.0), ("feed", -4.0), ("rapid", 1.0)])
        # G83: pecks of Q, out to R after each, back down to 0.254 above the last depth
        path, _ = z_path(start + [("G98", {}), ("G83", {"Z": -5.0, "R": 1.0, "Q": 2.0, "F": 2})])
        self.assertEqual(path, [("rapid", 1.0), ("feed", -1.0), ("rapid", 1.0), ("rapid", -0.746), ("feed", -3.0),
                                ("rapid", 1.0), ("rapid", -2.746), ("feed", -5.0), ("rapid", 5.0)])
        # G73: pecks of Q, backing off the clearance (D here) only
        path, _ = z_path(start + [("G98", {}), ("G73", {"Z": -3.0, "R": 1.0, "Q": 2.0, "D": 0.5, "F": 2})])
        self.assertEqual(path, [("rapid", 1.0), ("feed", -1.0), ("rapid", -0.5), ("feed", -3.0), ("rapid", 5.0)])
        # below R at the start: up to R first (preliminary motion)
        path, _ = z_path([("G0", {"X": 0.0, "Y": 0.0, "Z": -1.0}), ("G17", {}), ("G98", {}),
                          ("G81", {"Z": -6.0, "R": 2.0, "F": 2})])
        self.assertEqual(path, [("rapid", 2.0), ("feed", -6.0), ("rapid", 2.0)])
        # errors are reported, not simulated
        _, diags = z_path(start + [("G81", {"Z": 2.0, "R": 1.0})])
        self.assertTrue(any("R below the bottom" in d.message for d in diags))
        _, diags = z_path(start + [("G83", {"Z": -5.0, "R": 1.0})])
        self.assertTrue(any("positive Q" in d.message for d in diags))
        # the series ends at G80 (no diagnostic), G4 dwells are silent
        moves, diags = run(start + [("G81", {"Z": -2.0, "R": 1.0, "F": 2}), ("G80", {}), ("G4", {"P": 0.5}),
                                    ("G0", {"Z": 9.0})])
        self.assertEqual(diags, [])
        self.assertEqual(moves[-1].end[2], 9.0)

    def test_parse_nc_text(self):
        c = parse_gcode_text("G18\nG0 F5.000000 X1.0 Z2.0\n(comment)\nG3 F5 I-1 K0 X0 Z1\n")
        self.assertEqual(c[0], ("G18", {}))
        self.assertEqual(c[1][1]["Z"], 2.0)
        self.assertEqual(len(c), 3)


if __name__ == "__main__":
    unittest.main()
