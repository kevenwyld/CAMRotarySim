#!/usr/bin/env python3
# SPDX-License-Identifier: LGPL-2.1-or-later
"""
The thread Job's thread (threading_test.FCStd: M12x1.75, 1.129 deep from
12.002, modified flank infeed 29.5, degression 1.5, 1 spring pass) made
by LibLathe's own ThreadOP, as explicit G33 passes or, with --g76, as the
one G76 cycle CAM Lathe Turning writes with "Use G76 cycle". Both come
from the same settings, so simulated they should cut the same thread.

Usage: make_thread_g76_fixture.py LIBLATHE_DIR [--g76] [--run-out MM] > TurnThread.nc
"""

import argparse
import json
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
DATA = os.path.join(HERE, "..", "tests", "data", "thread")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("liblathe")
    ap.add_argument("--g76", action="store_true")
    ap.add_argument("--run-out", type=float, default=0.0,
                    help="run-out taper at the exit (G76 E with L2); 0: stop dead (CAM Lathe Turning's default "
                         "is one pitch)")
    a = ap.parse_args()
    sys.path.insert(0, a.liblathe)
    from liblathe.base.boundbox import BoundBox
    from liblathe.base.point import Point
    from liblathe.base.segment import Segment
    from liblathe.op.thread import ThreadOP

    with open(os.path.join(DATA, "stock.json")) as f:
        stock = json.load(f)
    with open(os.path.join(DATA, "op.json")) as f:
        meta = json.load(f)
    r, zmin, zmax = stock["radius"], stock["z_min"], stock["z_max"]
    major = meta["ThreadDiameter"] / 2
    op = ThreadOP()
    op.set_params({"pitch": meta["Pitch"], "thread_depth": meta["ThreadDepth"], "thread_radius": major,
                   "internal": False, "passes": 0, "first_depth": 0.0, "min_depth": 0.05, "degression": 1.5,
                   "infeed_angle": 29.5, "spring_passes": 1, "lead_in": 0.0, "run_out": a.run_out,
                   "canned": a.g76, "thread_length": 0.0, "tool_angle": 60.0, "tool_width": 6.0,
                   "start_offset": 0.0, "end_offset": 0.0, "clearance": 3.0, "min_dia": 0.0, "extra_dia": 0.0,
                   "allow_grooving": False, "step_over": 1.0, "finish_passes": 1, "stock_to_leave": 0.0,
                   "hfeed": 1.0, "vfeed": 1.0})
    op.add_stock(BoundBox(Point(-r, 0, zmin), Point(0, 0, zmax)))
    # the part: a plain cylinder of the thread's major diameter (LibLathe's -X side)
    op.add_part_edges([Segment(Point(0, 0, zmax), Point(-major, 0, zmax)),
                       Segment(Point(-major, 0, zmax), Point(-major, 0, zmin)),
                       Segment(Point(-major, 0, zmin), Point(0, 0, zmin))])
    kind = "G76 cycle" if a.g76 else "G33 passes"
    print("(TurnThread: LibLathe ThreadOP, %s%s)" % (kind, ", run-out %g" % a.run_out if a.run_out else ""))
    for c in op.get_gcode():
        words = " ".join("%s%.9f" % (k, v) for k, v in c.get_params().items())   # near FreeCAD's doubles
        print(("%s %s" % (c.get_movement(), words)).strip())


if __name__ == "__main__":
    main()
