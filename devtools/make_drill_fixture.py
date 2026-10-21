#!/usr/bin/env python3
# SPDX-License-Identifier: LGPL-2.1-or-later
"""
Turn Drill fixtures for the boring Job, made by the TurningAddon's own code:
ObjectTurnDrill.op_generate_gcode() and explicit_moves() are taken from
PathTurnDrill.py (via ast, so FreeCAD isn't needed) and run with stand-in
objects. The hole is the boring Job's bore (Z -3 to -113, through) drilled
with a 10 mm, 118 degree drill.

Usage: make_drill_fixture.py TURNINGADDON_DIR [--cycle G83|G73|G81|G82] [--explicit]
                             [--depth-to-tip] [--peck 2.0] > TurnDrill.nc
"""

import argparse
import ast
import math
import os
import types

KIND = {"G83": "Peck (G83)", "G73": "Chip break (G73)", "G81": "Drill (G81)", "G82": "Dwell (G82)"}


class Command:
    def __init__(self, name, params=None):
        self.Name, self.Parameters = name, dict(params or {})


def q(v):
    return types.SimpleNamespace(Value=v)


def load(addon):
    path = os.path.join(addon, "PathTurnScripts", "PathTurnDrill.py")
    with open(path) as f:
        tree = ast.parse(f.read())
    cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == "ObjectTurnDrill")
    keep = [n for n in tree.body if isinstance(n, ast.Assign) and any(
        getattr(t, "id", "") in ("CODE", "CHIP_BREAK") for t in n.targets)]
    methods = [n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name in ("op_generate_gcode",
                                                                                    "explicit_moves")]
    cls.body = methods
    cls.bases = []
    cls.decorator_list = []
    ns = {"math": math, "Path": types.SimpleNamespace(Command=Command),
          "translate": lambda c, t, d=None: t,
          "FreeCAD": types.SimpleNamespace(Console=types.SimpleNamespace(PrintWarning=lambda s: None))}
    exec(compile(ast.Module(keep + [cls], []), path, "exec"), ns)
    return ns


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("turningaddon")
    ap.add_argument("--cycle", default="G83", choices=sorted(KIND))
    ap.add_argument("--explicit", action="store_true", help="moves written out, not a canned cycle")
    ap.add_argument("--depth-to-tip", action="store_true")
    ap.add_argument("--peck", type=float, default=2.0)
    ap.add_argument("--diameter", type=float, default=10.0)
    a = ap.parse_args()
    ns = load(a.turningaddon)
    surface = types.SimpleNamespace(Center=types.SimpleNamespace(x=0.0, y=0.0), Axis=types.SimpleNamespace(z=1.0),
                                    Radius=5.0)
    op = ns["ObjectTurnDrill"]()
    op.selected_hole = lambda obj: types.SimpleNamespace(Surface=surface)
    op.stock = types.SimpleNamespace(Shape=types.SimpleNamespace(
        BoundBox=types.SimpleNamespace(ZMax=0.0, XLength=30.0)))
    op.commandlist = []
    job = types.SimpleNamespace(SetupSheet=types.SimpleNamespace(SafeHeightOffset=q(3.0)))
    ns["PathUtils"] = types.SimpleNamespace(findParentJob=lambda obj: job)
    obj = types.SimpleNamespace(Label="TurnDrill", Cycle=KIND[a.cycle], PeckDepth=q(a.peck), DwellTime=0.5,
                                RetractHeight=q(1.0), DepthToTip=a.depth_to_tip, UseCannedCycle=not a.explicit,
                                StartDepth=q(-3.0), FinalDepth=q(-113.0), DrillReport="",
                                ToolController=types.SimpleNamespace(VertFeed=q(3.333333)))
    tool = types.SimpleNamespace(Diameter=q(a.diameter), TipAngle=q(118.0))
    op.op_generate_gcode(obj, tool)
    print("(TurnDrill: TurningAddon PathTurnDrill.op_generate_gcode, %s%s)"
          % (KIND[a.cycle], ", written out" if a.explicit else ""))
    for line in obj.DrillReport.splitlines():
        print("(%s)" % line.replace("(", "[").replace(")", "]"))
    for c in op.commandlist:
        print(("%s %s" % (c.Name, " ".join("%s%.6f" % (k, v) for k, v in c.Parameters.items()))).strip())


if __name__ == "__main__":
    main()
