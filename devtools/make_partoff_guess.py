#!/usr/bin/env python3
# SPDX-License-Identifier: LGPL-2.1-or-later
"""
Generate a Partoff path for the test Job with LibLathe's PartoffOP, without
FreeCAD, so the simulator has a Partoff input that matches what the
TurningAddon would produce. This is a stand-in until a real Partoff op is
exported from FreeCAD.

It mirrors PathTurnBase/PathTurnPartoff: stock silhouette and part outline
on the -X side (LibLathe's internal frame), chain_profile() taken from the
TurningAddon source, Isoshape parameters as the tool.

Usage:
  make_partoff_guess.py LIBLATHE_DIR TURNINGADDON_DIR [--final-depth Z] > TurnPartoff.nc
"""

import argparse
import ast
import json
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
DATA = os.path.join(HERE, "..", "tests", "data")


def _read(path):
    with open(path) as f:
        return f.read()


def load_chain_profile(turning_addon_dir):
    """Extract chain_profile() from PathTurnBase.py without importing FreeCAD."""
    path = os.path.join(turning_addon_dir, "PathTurnScripts", "PathTurnBase.py")
    tree = ast.parse(_read(path))
    fn = next(n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name == "chain_profile")
    ns = {"math": math}
    exec(compile(ast.Module([fn], []), path, "exec"), ns)
    return ns["chain_profile"]


def part_items(edges):
    """Section edges on the -X half as chain_profile items."""
    items = []
    for e in edges:
        pts = e["points"]
        if max(p[0] for p in pts) > 1e-6:
            continue
        p0, p1 = tuple(pts[0]), tuple(pts[-1])
        item = {"p0": p0, "p1": p1, "mid": None, "angle": 0}
        if e["kind"] == "arc":
            cx, cz = e["centre"]
            item["mid"] = tuple(pts[len(pts) // 2])
            a = [math.atan2(p[1] - cz, p[0] - cx) for p in pts]
            item["angle"] = abs(sum((b - c + math.pi) % (2 * math.pi) - math.pi for b, c in zip(a[1:], a[:-1])))
        items.append(item)
    return items


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("liblathe")
    ap.add_argument("turningaddon")
    ap.add_argument("--final-depth", type=float, default=None,
                    help="op FinalDepth (default: stock Z min, the op's default)")
    ap.add_argument("--clearance", type=float, default=3.0)
    a = ap.parse_args()
    sys.path.insert(0, a.liblathe)
    from liblathe.base.boundbox import BoundBox
    from liblathe.base.point import Point
    from liblathe.base.segment import Segment
    from liblathe.base.tool import Tool
    from liblathe.op.partoff import PartoffOP

    stock = json.loads(_read(os.path.join(DATA, "stock.json")))
    part = json.loads(_read(os.path.join(DATA, "part_section.json")))
    tool_d = json.loads(_read(os.path.join(DATA, "tool_T37_section.json")))["params"]

    chain, unchained = load_chain_profile(a.turningaddon)(part_items(part["edges"]))
    segs = []
    for s, e, ang in chain:
        seg = Segment(Point(s[0], 0, s[1]), Point(e[0], 0, e[1]))
        if ang:
            seg.set_bulge(ang)
        segs.append(seg)

    tool = Tool()
    tool.set_tip_angle(tool_d["TipAngle"])
    tool.set_edge_length(tool_d["EdgeLength"])
    tool.set_nose_radius(tool_d["TipRadius"])
    tool.set_rotation(tool_d["Rotation"])

    r, zmin, zmax = stock["radius"], stock["z_min"], stock["z_max"]
    final = zmin if a.final_depth is None else a.final_depth
    op = PartoffOP()
    op.set_params({"min_dia": 0.0, "extra_dia": 0.0, "start_offset": 0.0, "end_offset": zmin - final,
                   "allow_grooving": False, "step_over": 1.0, "finish_passes": 1, "stock_to_leave": 0.0,
                   "hfeed": 5.0, "vfeed": 5.0, "clearance": a.clearance})
    op.add_stock(BoundBox(Point(-r, 0, zmin), Point(0, 0, zmax)))
    op.add_part_edges(segs)
    op.add_tool(tool)
    print("(Partoff guess: LibLathe PartoffOP, FinalDepth %.4f, %d unchained edges)" % (final, unchained))
    for c in op.get_gcode():
        words = " ".join("%s%.6f" % (k, v) for k, v in c.get_params().items())
        print(("%s %s" % (c.get_movement(), words)).strip())


if __name__ == "__main__":
    main()
