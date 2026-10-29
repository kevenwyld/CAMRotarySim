# SPDX-License-Identifier: LGPL-2.1-or-later
"""
Find and measure the threads a simulation cut: major, minor and pitch diameter (the median
radius over whole pitches) on the stock, compared with the Turn Thread op's diameters.
"""

import math
import re
from dataclasses import dataclass, field
from typing import List, Optional

import numpy as np

from .motion import THREAD

RANGE = r"([0-9]+(?:\.[0-9]+)?)\s*-\s*([0-9]+(?:\.[0-9]+)?)"
INCH = re.compile(r'[0-9]"')


@dataclass
class ThreadZone:
    op: int
    name: str
    pitch: float
    z0: float           # axial extent cut by the passes (tool outline included)
    z1: float
    deepest: float      # deepest programmed radius of the passes (the tool's control point)
    passes: int
    pitches: List[float] = field(default_factory=list)
    meta: dict = field(default_factory=dict)
    internal: bool = False


@dataclass
class ThreadResult:
    zone: ThreadZone
    major: Optional[float] = None
    minor: Optional[float] = None
    pitch_diameter: Optional[float] = None
    span: Optional[tuple] = None
    expected_pitch: Optional[tuple] = None    # (min, max) diameters
    expected_major: Optional[tuple] = None
    expected_minor: Optional[tuple] = None
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
        internal = bool(op.internal)
        x = max(m.start[0], m.end[0]) if internal else min(m.start[0], m.end[0])
        if z is None:
            zones[m.op] = ThreadZone(m.op, op.name, m.pitch, lo, hi, x, 1, [m.pitch], dict(op.meta or {}), internal)
        else:
            z.z0, z.z1 = min(z.z0, lo), max(z.z1, hi)
            z.deepest = max(z.deepest, x) if internal else min(z.deepest, x)
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


def _pair(meta, lo, hi):
    a, b = float(meta.get(lo) or 0.0), float(meta.get(hi) or 0.0)
    return (a, b) if a > 0 and b > 0 else None


def expected_diameters(meta, notes):
    """(pitch, major, minor) (min, max) ranges in mm: the op's Thread*Dia properties, else the
    ThreadReport text; an inch report (numbers with a ") is not compared."""
    if "ThreadPitchDiaMin" in meta:
        return tuple(_pair(meta, "Thread%sDiaMin" % k, "Thread%sDiaMax" % k) for k in ("Pitch", "Major", "Minor"))
    report = meta.get("ThreadReport", "") or ""
    if INCH.search(report):
        notes.append("inch report: not compared")
        return None, None, None
    return _range(report, "Pitch diameter"), _range(report, "major diameter to"), _range(report, "minor diameter to")


def measure(stock, zone, tool_half_width=0.0, tol=0.01):
    res = ThreadResult(zone)
    p = zone.pitch
    if max(zone.pitches) - min(zone.pitches) > 1e-9:
        res.notes.append("passes use different pitches (%s): the grooves won't line up"
                         % ", ".join("%g" % k for k in sorted(set(zone.pitches))))
    meta = zone.meta or {}
    res.expected_pitch, res.expected_major, res.expected_minor = expected_diameters(meta, res.notes)
    if meta.get("ThreadDiameter") and meta.get("ThreadDepth"):
        sign = 1 if zone.internal else -1
        res.expected_root = float(meta["ThreadDiameter"]) + sign * 2 * float(meta["ThreadDepth"])
    if meta.get("ThreadRootDia"):
        res.expected_root = float(meta["ThreadRootDia"])
    if res.expected_root is not None:
        if abs(2 * zone.deepest - res.expected_root) > 2 * tol:
            res.notes.append("deepest pass reaches diameter %.3f; the op's %s is %.3f"
                             % (2 * zone.deepest, "major" if zone.internal else "root", res.expected_root))
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
    R = (stock.Rin if zone.internal else stock.R)[:, j0:j1]
    res.span = (a, b)
    res.major = 2 * float(np.median(R.max(axis=1)))
    res.minor = 2 * float(np.median(R.min(axis=1)))
    res.pitch_diameter = 2 * float(np.median(R))
    lo_r, hi_r = res.minor / 2, res.major / 2
    if hi_r - lo_r > 1e-6 and min(abs(res.pitch_diameter / 2 - lo_r), abs(res.pitch_diameter / 2 - hi_r)) < 1e-6:
        # saturated at the crests or roots: extrapolate the groove fraction along the flanks

        def groove(r):  # share of the surface cut deeper than radius r
            return float(np.mean(R > r)) if zone.internal else float(np.mean(R < r))

        r1, r2 = lo_r + 0.3 * (hi_r - lo_r), lo_r + 0.7 * (hi_r - lo_r)
        f1, f2 = groove(r1), groove(r2)
        if abs(f2 - f1) > 1e-9:
            res.pitch_diameter = 2 * (r1 + (0.5 - f1) * (r2 - r1) / (f2 - f1))
            res.notes.append("pitch line outside the material (groove %s than the ridge at the %s): "
                             "extrapolated along the flanks"
                             % (("narrower", "crests") if (f1 + f2) / 2 < 0.5 else ("wider", "roots")))
    if res.expected_pitch:
        lo, hi = res.expected_pitch
        # external threads come out too big when shallow, internal ones too small
        over, under = ("deep", "shallow") if zone.internal else ("shallow", "deep")
        say = {"shallow": "not cut deep enough", "deep": "cut too deep"}
        if res.pitch_diameter > hi + tol:
            res.verdict = over
            res.notes.append("pitch diameter %.3f is %.3f over the op's range %.3f-%.3f: %s"
                             % (res.pitch_diameter, res.pitch_diameter - hi, lo, hi, say[over]))
        elif res.pitch_diameter < lo - tol:
            res.verdict = under
            res.notes.append("pitch diameter %.3f is %.3f under the op's range %.3f-%.3f: %s"
                             % (res.pitch_diameter, lo - res.pitch_diameter, lo, hi, say[under]))
        else:
            res.verdict = "ok"
    # the crests: the major diameter of an external thread, the minor of an internal one
    crest, rng, name = ((res.minor, res.expected_minor, "minor") if zone.internal
                        else (res.major, res.expected_major, "major"))
    if rng:
        lo, hi = rng
        if crest > hi + tol or crest < lo - tol:
            res.notes.append("crests at diameter %.3f, outside the op's %s diameter %.3f-%.3f" % (crest, name, lo, hi))
    return res


def format_thread(res):
    z = res.zone
    lines = ["  %s: %s, pitch %g, %d passes, Z %.3f to %.3f"
             % (z.name, "internal" if z.internal else "external", z.pitch, z.passes, z.z1, z.z0)]
    if res.pitch_diameter is not None:
        lines.append("    measured over Z %.3f to %.3f: major %.3f, pitch %.3f, minor %.3f"
                     % (res.span[1], res.span[0], res.major, res.pitch_diameter, res.minor))
    if res.expected_pitch:
        extra = ""
        if res.expected_major:
            extra += ", major %.3f-%.3f" % res.expected_major
        if res.expected_minor and z.internal:      # external: the class's d1, not the root an insert cuts
            extra += ", minor %.3f-%.3f" % res.expected_minor
        lines.append("    op: pitch diameter %.3f-%.3f%s" % (res.expected_pitch + (extra,)))
    lines.append("    deepest pass: control point at diameter %.3f" % (2 * z.deepest)
                 + (" (op's %s %.3f)" % ("major" if z.internal else "root", res.expected_root)
                    if res.expected_root else ""))
    lines += ["    " + n for n in res.notes]
    return lines
