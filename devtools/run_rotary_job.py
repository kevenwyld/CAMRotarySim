#!/usr/bin/env python3
# SPDX-License-Identifier: LGPL-2.1-or-later
"""
Offline run of the indexed rotary Job (tests/data/rotary, from
inexed_rotary_test.FCStd): three Adaptive ops on work planes facing -Y, +Z
and +Y, simulated in the workpiece frame. Prints the report and, with
--out, plots sections 2.5 mm under each machined face (simulated material
filled, the model's outline drawn).

Usage: run_rotary_job.py [--grid 0.5] [--out DIR]
"""

import argparse
import json
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, ".."))
DATA = os.path.join(HERE, "..", "tests", "data", "rotary")

from camrotarysim.core.milling import (MillingKinematics, MillTool, VoxelStock, compare_part,  # noqa: E402
                                       format_milling, needed_angle, placement_frame)
from camrotarysim.core.motion import parse_gcode_text  # noqa: E402
from camrotarysim.core.simulation import OpInput, Simulation  # noqa: E402
from camrotarysim.core.stock import rotary_axes_from_machine  # noqa: E402
from camrotarysim.core.surface import mesh_sdf  # noqa: E402


def _read(fn):
    with open(os.path.join(DATA, fn)) as f:
        return f.read()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--grid", type=float, default=0.5)
    ap.add_argument("--out", default=None)
    a = ap.parse_args()
    job = json.loads(_read("job.json"))
    axis = rotary_axes_from_machine(json.loads(_read("tormach_4-axis.fcm")))["A"]
    st = VoxelStock.from_box(job["stock"]["lo"], job["stock"]["hi"], a.grid, axis)
    tool = MillTool.from_cam("endmill", job["tool"]["Diameter"], job["tool"]["CuttingEdgeHeight"])
    ops, info = [], []
    for n in job["order"]:
        f = job["frames"][n]
        R, t = placement_frame(f["position"], f["quaternion"])
        angle, _err = needed_angle(R[:, 2], axis.direction)
        info.append("%s: (%.3f, %.3f, %.3f), A %+.1f (the posted program has A %+.1f)"
                    % ((n,) + tuple(R[:, 2]) + (angle, job["ngc_A"][n])))
        ops.append(OpInput(n, parse_gcode_text(_read(n + ".nc")), tool, kinematics=MillingKinematics(axis, (R, t))))
    sim = Simulation(st, ops, step=min(0.25, a.grid / 2))
    sim.run_to_end()
    part = mesh_sdf(st.lo, st.h, st.shape, np.array(json.loads(_read("part_triangles.json"))["triangles"]))
    print(format_milling(sim, compare_part(st, part), info))
    print("\nRemoved %.1f mm3." % st.removed())
    if a.out:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        os.makedirs(a.out, exist_ok=True)
        fig, axs = plt.subplots(1, 3, figsize=(15, 4.5))
        for ax, (name, k, value, (a0, a1)) in zip(axs, (("Adaptive001 (+Z face)", 2, 22.5, (0, 1)),
                                                        ("Adaptive (-Y face)", 1, -22.5, (0, 2)),
                                                        ("Adaptive002 (+Y face)", 1, 22.5, (0, 2)))):
            i = int(np.argmin(np.abs(st.axes[k] - value)))
            S, P = np.take(st.phi, i, axis=k), np.take(part, i, axis=k)
            ax.contourf(st.axes[a0], st.axes[a1], (S > 0).T.astype(float), levels=[0.5, 1.5], colors=["#9ab"])
            ax.contour(st.axes[a0], st.axes[a1], P.T, levels=[0], colors="k", linewidths=0.8)
            ax.set_title("%s: section at %s=%.1f (fill: simulated, line: model)" % (name, "XYZ"[k], st.axes[k][i]),
                         fontsize=8)
            ax.set_xlabel("XYZ"[a0])
            ax.set_ylabel("XYZ"[a1])
            ax.set_aspect("equal")
        fig.savefig(os.path.join(a.out, "rotary_sections.png"), dpi=110, bbox_inches="tight")


if __name__ == "__main__":
    main()
