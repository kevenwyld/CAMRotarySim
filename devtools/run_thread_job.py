#!/usr/bin/env python3
# SPDX-License-Identifier: LGPL-2.1-or-later
"""
Offline run of the thread Job fixtures (tests/data/thread, from
threading_test.FCStd): simulate, measure the thread, print the report and,
with --out, plot:
- thread_section.png: the simulated axial section at angle 0 over three
  pitches, with the ISO basic profile moved to the op's target pitch
  diameter (and the 6g pitch diameter band);
- thread_unrolled.png: remaining radius against Z and angle, which shows
  the helix's hand (a right-hand thread's grooves climb toward +Z as the
  angle grows).

Usage: run_thread_job.py [--angles N] [--left] [--out DIR]
"""

import argparse
import json
import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, ".."))
DATA = os.path.join(HERE, "..", "tests", "data", "thread")

from camrotarysim.core import compare as C  # noqa: E402
from camrotarysim.core import thread_check as T  # noqa: E402
from camrotarysim.core.motion import parse_gcode_text  # noqa: E402
from camrotarysim.core.report import format_report  # noqa: E402
from camrotarysim.core.simulation import OpInput, Simulation  # noqa: E402
from camrotarysim.core.stock import RotaryStock  # noqa: E402
from camrotarysim.core.tool import REF_ORIGIN, ToolOutline  # noqa: E402


def _read(fn):
    with open(os.path.join(DATA, fn)) as f:
        return f.read()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--angles", type=int, default=120)
    ap.add_argument("--left", action="store_true", help="Forward spindle cuts a left-hand thread")
    ap.add_argument("--out", default=None)
    a = ap.parse_args()
    td = json.loads(_read("tool_threading_section.json"))
    tool = ToolOutline.from_section(td["edges"], td["params"]["TipRadius"], reference=REF_ORIGIN)
    sd = json.loads(_read("stock.json"))
    st = RotaryStock(sd["z_min"], sd["z_max"], 0.05, sd["radius"])
    meta = json.loads(_read("op.json"))
    op = OpInput("TurnThread", parse_gcode_text(_read("TurnThread.nc")), tool, spindle=-1 if a.left else 1, meta=meta)
    sim = Simulation(st, [op], step=0.05, thread_angles=a.angles)
    sim.run_to_end()
    pls = [e["points"] for e in json.loads(_read("part_section.json"))["edges"]]
    part = C.part_profile(pls, st.s)
    zones = T.thread_zones(sim)
    res = C.compare(st.profile(), part, st.s, 0.01, 0.05, None, [(z.z0, z.z1) for z in zones])
    threads = [T.measure(st, z, max(-tool.zmin, tool.zmax)) for z in zones]
    print(format_report(sim, res, part, C.part_profile(pls, st.s, -1), threads=threads))
    if a.out:
        plot(a.out, st, threads[0])


def basic_profile(z, pitch, d2, major, phase):
    """ISO basic external profile (flanks of the fundamental triangle, crest at major,
    root clipped at d3) with pitch diameter d2; phase = Z of a groove centre."""
    H = 0.866025 * pitch
    apex = d2 / 2 - H / 2                       # sharp V of the groove
    u = np.mod(z - phase + pitch / 2, pitch) - pitch / 2
    r = apex + np.abs(u) / math.tan(math.radians(30))
    d3 = major - 2 * 17 / 24 * H
    return np.clip(r, d3 / 2, major / 2)


def plot(out, st, res):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    os.makedirs(out, exist_ok=True)
    p = res.zone.pitch
    zc = -28.0
    m = (st.s > zc - 1.5 * p) & (st.s < zc + 1.5 * p)
    s, row = st.s[m], st.R[0, m]
    target = 10.754
    # groove centre at angle 0: midway between the flanks at the measured pitch radius
    r_p = res.pitch_diameter / 2
    below = s[row < r_p]
    groups = np.split(below, np.nonzero(np.diff(below) > st.ds * 1.5)[0] + 1)
    centre = min((g for g in groups if len(g)), key=lambda g: abs(g.mean() - zc)).mean()
    fig, ax = plt.subplots(figsize=(9, 4.5))
    ax.plot(s, row, color="tab:blue", lw=1.5, label="simulated (angle 0)")
    zz = np.linspace(s[0], s[-1], 2000)
    ax.plot(zz, basic_profile(zz, p, target, 12.0, centre), "k--", lw=1,
            label="ISO basic profile at pitch diameter %.3f (the op's target)" % target)
    for d, ls in ((10.679, ":"), (10.829, ":")):
        ax.axhline(d / 2, color="tab:green", ls=ls, lw=0.8)
    ax.axhline(r_p, color="tab:orange", lw=0.8,
               label="measured pitch diameter %.3f (groove = ridge)" % res.pitch_diameter)
    ax.text(s[0] + 0.05, 10.829 / 2 + 0.02, "6g pitch diameter 10.679-10.829", color="tab:green", fontsize=8)
    ax.set_aspect("equal")
    ax.set_xlabel("Z (mm)")
    ax.set_ylabel("radius (mm)")
    ax.legend(fontsize=7, loc="upper center", bbox_to_anchor=(0.5, -0.2), ncol=2)
    ax.grid(alpha=0.3)
    ax.set_title("TurnThread, M12x1.75: axial section", fontsize=9)
    fig.savefig(os.path.join(out, "thread_section.png"), dpi=130, bbox_inches="tight")
    plt.close(fig)

    m = (st.s > zc - 2 * p) & (st.s < zc + 2 * p)
    fig, ax = plt.subplots(figsize=(8, 4.5))
    im = ax.imshow(st.R[:, m], aspect="auto", origin="lower", cmap="viridis",
                   extent=(st.s[m][0], st.s[m][-1], 0, 360))
    fig.colorbar(im, ax=ax, label="remaining radius (mm)")
    ax.set_xlabel("Z (mm)")
    ax.set_ylabel("workpiece angle (deg, counter-clockwise seen from +Z)")
    ax.set_title("Unrolled thread: grooves climbing toward +Z as the angle grows = right-hand", fontsize=9)
    fig.savefig(os.path.join(out, "thread_unrolled.png"), dpi=130, bbox_inches="tight")
    plt.close(fig)


if __name__ == "__main__":
    main()
