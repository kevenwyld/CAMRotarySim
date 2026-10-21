# SPDX-License-Identifier: LGPL-2.1-or-later
"""
Thread check: find the threads a simulation cut and measure them.

Measured on the simulated stock, over whole pitches in the interior of the
thread (away from its ends):
- major diameter: the crests (largest radius per angle, median over angles);
- minor diameter: the roots (smallest radius per angle);
- pitch diameter: where the groove and the ridge are equally wide. Over a
  whole number of pitches that is the median radius, which needs no
  assumption about the profile.

Compared, when the op provides them (TurningAddon Turn Thread), with the
pitch diameter range and major diameter range in its ThreadReport, and the
programmed root with ThreadDiameter / 2 - ThreadDepth.
"""

import math
import re
from dataclasses import dataclass, field
from typing import List, Optional

import numpy as np

from .motion import THREAD

RANGE = r"([0-9]+(?:\.[0-9]+)?)\s*-\s*([0-9]+(?:\.[0-9]+)?)"


@dataclass
class ThreadZone:
    op: int
    name: str
    pitch: float
    z0: float           # axial extent cut by the passes (tool outline included)
    z1: float
    deepest: float      # smallest programmed radius of the passes (the tool's control point)
    passes: int
    pitches: List[float] = field(default_factory=list)
    meta: dict = field(default_factory=dict)


@dataclass
class ThreadResult:
    zone: ThreadZone
    major: Optional[float] = None
    minor: Optional[float] = None
    pitch_diameter: Optional[float] = None
    span: Optional[tuple] = None
    expected_pitch: Optional[tuple] = None    # (min, max) diameters
    expected_major: Optional[tuple] = None
    expected_root: Optional[float] = None     # diameter the control point should reach
    verdict: str = ""                          # "ok" | "shallow" | "deep" | ""
    notes: List[str] = field(default_factory=list)


def thread_zones(sim):
    zones = {}
    for m in sim.moves:
        if m.kind != THREAD:
            continue
        op = sim.ops[m.op]
        z = zones.get(m.op)
        lo, hi = min(m.start[2], m.end[2]) + op.tool.zmin, max(m.start[2], m.end[2]) + op.tool.zmax
        x = min(m.start[0], m.end[0])
        if z is None:
            zones[m.op] = ThreadZone(m.op, op.name, m.pitch, lo, hi, x, 1, [m.pitch], dict(op.meta or {}))
        else:
            z.z0, z.z1, z.deepest = min(z.z0, lo), max(z.z1, hi), min(z.deepest, x)
            z.passes += 1
            z.pitches.append(m.pitch)
    st = sim.stock
    out = []
    for z in zones.values():
        z.z0, z.z1 = max(z.z0, st.s_min), min(z.z1, st.s_max)
        if z.z1 > z.z0:
            out.append(z)
    return out


def _range(text, label):
    m = re.search(re.escape(label) + r"[^0-9]*" + RANGE, text or "")
    return (float(m.group(1)), float(m.group(2))) if m else None


def measure(stock, zone, tool_half_width=0.0, tol=0.01):
    res = ThreadResult(zone)
    p = zone.pitch
    if max(zone.pitches) - min(zone.pitches) > 1e-9:
        res.notes.append("passes use different pitches (%s): the grooves won't line up"
                         % ", ".join("%g" % k for k in sorted(set(zone.pitches))))
    meta = zone.meta or {}
    report = meta.get("ThreadReport", "")
    res.expected_pitch = _range(report, "Pitch diameter")
    res.expected_major = _range(report, "major diameter to")
    if meta.get("ThreadDiameter") and meta.get("ThreadDepth"):
        res.expected_root = float(meta["ThreadDiameter"]) - 2 * float(meta["ThreadDepth"])
        short = 2 * zone.deepest - res.expected_root
        if abs(short) > 2 * tol:
            res.notes.append("the deepest pass reaches diameter %.3f, the op's root is %.3f"
                             % (2 * zone.deepest, res.expected_root))
    if stock.n_theta == 1:
        res.notes.append("stock has no angular detail: nothing to measure")
        return res
    # interior: away from both ends by the tool's half width and a pitch, whole pitches long
    margin = tool_half_width + p
    a, b = zone.z0 + margin, zone.z1 - margin
    n_p = int(math.floor((b - a) / p))
    if n_p < 1:
        res.notes.append("thread too short to measure (needs a pitch clear of both ends)")
        return res
    mid = (a + b) / 2
    a, b = mid - n_p * p / 2, mid + n_p * p / 2
    j0, j1 = stock.column_range(a, b)
    R = stock.R[:, j0:j1]
    res.span = (a, b)
    res.major = 2 * float(np.median(R.max(axis=1)))
    res.minor = 2 * float(np.median(R.min(axis=1)))
    res.pitch_diameter = 2 * float(np.median(R))
    if res.expected_pitch:
        lo, hi = res.expected_pitch
        if res.pitch_diameter > hi + tol:
            res.verdict = "shallow"
            res.notes.append("pitch diameter %.3f is %.3f over the op's range %.3f-%.3f: not cut deep enough"
                             % (res.pitch_diameter, res.pitch_diameter - hi, lo, hi))
        elif res.pitch_diameter < lo - tol:
            res.verdict = "deep"
            res.notes.append("pitch diameter %.3f is %.3f under the op's range %.3f-%.3f: cut too deep"
                             % (res.pitch_diameter, lo - res.pitch_diameter, lo, hi))
        else:
            res.verdict = "ok"
    if res.expected_major:
        lo, hi = res.expected_major
        if res.major > hi + tol or res.major < lo - tol:
            res.notes.append("crests at diameter %.3f, outside the op's major diameter %.3f-%.3f"
                             % (res.major, lo, hi))
    return res


def format_thread(res):
    z = res.zone
    lines = ["  %s: pitch %g, %d passes, Z %.3f to %.3f" % (z.name, z.pitch, z.passes, z.z1, z.z0)]
    if res.pitch_diameter is not None:
        lines.append("    measured over Z %.3f to %.3f: major %.3f, pitch %.3f, minor %.3f"
                     % (res.span[1], res.span[0], res.major, res.pitch_diameter, res.minor))
    if res.expected_pitch:
        lines.append("    op: pitch diameter %.3f-%.3f%s" % (res.expected_pitch + (
            ", major %.3f-%.3f" % res.expected_major if res.expected_major else "",)))
    lines.append("    deepest pass: control point at diameter %.3f" % (2 * z.deepest)
                 + (" (op's root %.3f)" % res.expected_root if res.expected_root else ""))
    lines += ["    " + n for n in res.notes]
    return lines
