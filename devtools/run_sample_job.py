#!/usr/bin/env python3
# SPDX-License-Identifier: LGPL-2.1-or-later
"""
Offline run of the sample Job fixtures (tests/data, taken from
turning_test2.FCStd): simulate, compare with the part, print the report and
write PNG plots for visual checks (insert outline, result, close-ups).

Usage: run_sample_job.py [--partoff default|fd-60|none] [--out DIR]
"""

import argparse
import json
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, ".."))
DATA = os.path.join(HERE, "..", "tests", "data")

from camrotarysim.core import compare as C  # noqa: E402
from camrotarysim.core.motion import parse_gcode_text  # noqa: E402
from camrotarysim.core.report import format_report  # noqa: E402
from camrotarysim.core.simulation import OpInput, Simulation  # noqa: E402
from camrotarysim.core.stock import RotaryStock  # noqa: E402
from camrotarysim.core.tool import ToolOutline  # noqa: E402


def _read(path):
    with open(path) as f:
        return f.read()


def load(partoff="none", ds=0.05):
    td = json.loads(_read(os.path.join(DATA, "tool_T37_section.json")))
    tool = ToolOutline.from_section(td["edges"], tip_radius=td["params"]["TipRadius"])
    sd = json.loads(_read(os.path.join(DATA, "stock.json")))
    stock = RotaryStock(sd["z_min"], sd["z_max"], ds, sd["radius"])
    names = ["TurnFace", "TurnRough", "TurnProfile"]
    files = [n + ".nc" for n in names]
    if partoff != "none":
        names.append("TurnPartoff (%s guess)" % partoff)
        files.append("TurnPartoff_%s.nc" % partoff)
    ops = [OpInput(n, parse_gcode_text(_read(os.path.join(DATA, f))), tool) for n, f in zip(names, files)]
    part = json.loads(_read(os.path.join(DATA, "part_section.json")))
    polylines = [e["points"] for e in part["edges"]]
    return tool, stock, ops, polylines, td


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--partoff", default="none", choices=["none", "default", "fd-60"])
    ap.add_argument("--out", default=None)
    a = ap.parse_args()
    tool, stock, ops, polylines, td = load(a.partoff)
    sim = Simulation(stock, ops, step=0.05)
    sim.run_to_end()
    part_R = C.part_profile(polylines, stock.s)
    reach = C.reachable_profile(part_R, stock.s, tool)
    res = C.compare(stock.profile(), part_R, stock.s, 0.01, 0.05, reach)
    print(format_report(sim, res, part_R, C.part_profile(polylines, stock.s, -1)))
    if a.out:
        plot(a.out, tool, td, stock, sim, part_R, reach, res)


def plot(out, tool, td, stock, sim, part_R, reach, res):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from camrotarysim.core.mesh import CLASS_COLOURS

    os.makedirs(out, exist_ok=True)
    p = td["params"]
    iso = ToolOutline.from_iso(p["EdgeLength"], p["TipAngle"], p["TipRadius"], p["Rotation"])

    # 1. insert outline, lathe frame: Z right, X up, control point at the origin
    fig, ax = plt.subplots(figsize=(6, 6))
    for t, style, lab in ((tool, "-", "from tool shape section"), (iso, "--", "from Isoshape parameters")):
        q = np.vstack([t.points, t.points[:1]])
        ax.plot(q[:, 1], q[:, 0], style, label=lab)
    nc = tool.nose_centre
    ax.add_patch(plt.Circle((nc[1], nc[0]), tool.nose_radius, fill=False, color="k", lw=0.5))
    ax.plot([0], [0], "ko")
    ax.annotate("control point (imaginary tip)", (0, 0), (1.2, -0.45), fontsize=8,
                arrowprops=dict(arrowstyle="-", lw=0.5))
    sx, sz = -tool.control_in_shape[0], -tool.control_in_shape[1]
    ax.plot([sz], [sx], "rx")
    ax.annotate("sharp corner (shape origin)", (sz, sx), (1.2, -0.95), fontsize=8, color="r",
                arrowprops=dict(arrowstyle="-", lw=0.5, color="r"))
    ax.set_ylim(-1.3, 7.3)
    for ang in (p["Rotation"] - p["TipAngle"] / 2, p["Rotation"] + p["TipAngle"] / 2):
        r = np.radians(ang)
        ax.plot([sz, sz + 6 * np.cos(r)], [sx, sx + 6 * np.sin(r)], ":", color="gray")
        ax.annotate("%.0f° from +Z" % ang, (sz + 5.2 * np.cos(r), sx + 5.2 * np.sin(r)), fontsize=8)
    ax.set_aspect("equal")
    ax.set_xlabel("Z (mm)")
    ax.set_ylabel("X = radius (mm)")
    ax.set_title("T37 insert: EdgeLength %g, TipAngle %g°, TipRadius %g, Rotation %g°"
                 % (p["EdgeLength"], p["TipAngle"], p["TipRadius"], p["Rotation"]), fontsize=9)
    ax.legend(fontsize=8, loc="upper right")
    ax.grid(alpha=0.3)
    fig.savefig(os.path.join(out, "insert_outline.png"), dpi=130, bbox_inches="tight")
    plt.close(fig)

    # 2. result with deviation colours, plus close-ups
    views = [("result_full.png", None), ("result_ball_groove.png", (-21, 0.5, 5, 14)),
             ("result_shoulder_corner.png", (-39.6, -38.4, 12.6, 13.8))]
    gouge_tool = None
    for e in sim.moves:
        if sim.ops[e.op].name == "TurnRough" and e.index == 87:
            gouge_tool = e.end
    for fn, box in views:
        fig, ax = plt.subplots(figsize=(11, 5) if box is None else (8, 6))
        s = stock.s
        ax.fill_between(s, 0, stock.R0[0], color="0.92", label="initial stock")
        ax.plot(s, stock.profile(), color="tab:blue", lw=1, label="simulated")
        ax.plot(s, part_R, "k", lw=0.8, label="part (Job model section)")
        ax.plot(s, reach, color="tab:purple", lw=0.6, ls="--", label="reachable by this insert")
        for c, rgb in CLASS_COLOURS.items():
            m = res.classes == c
            if m.any():
                ax.scatter(s[m], stock.profile()[m], s=4, color=rgb, label=C.CLASS_NAMES[c], zorder=3)
        if gouge_tool is not None:
            q = np.vstack([tool.points, tool.points[:1]])
            ax.plot(gouge_tool[2] + q[:, 1], gouge_tool[0] + q[:, 0], color="tab:red", lw=0.8,
                    label="insert at TurnRough #87 end")
        if box:
            ax.set_xlim(box[0], box[1])
            ax.set_ylim(box[2], box[3])
            ax.set_aspect("equal")
        ax.set_xlabel("Z (mm)")
        ax.set_ylabel("radius (mm)")
        ax.legend(fontsize=7, loc="lower left" if box is None else "best")
        ax.grid(alpha=0.3)
        fig.savefig(os.path.join(out, fn), dpi=130, bbox_inches="tight")
        plt.close(fig)


if __name__ == "__main__":
    main()
