# SPDX-License-Identifier: LGPL-2.1-or-later
"""
Quality check: compare the simulated stock with the part.

- part_profile(): the part's outer radius per axial column, from section
  polylines of the Job model in the XZ plane.
- reachable_profile(): the best result this tool shape can produce on this
  part from outside (a 1D morphological opening of the part by the tool's
  lower envelope). Material the tool can't reach without cutting into the
  part, such as an undercut steeper than the insert's trailing edge, stays
  above this profile whatever the path does.
- compare(): deviation and a class per column:
    OK, GOUGE (cut into the part), UNCUT (left by the path, reachable),
    UNREACHABLE (left because the tool shape can't get there).
"""

import math
from dataclasses import dataclass, field
from typing import List

import numpy as np

OK, GOUGE, UNCUT, UNREACHABLE = 0, 1, 2, 3
CLASS_NAMES = {OK: "ok", GOUGE: "gouge", UNCUT: "uncut", UNREACHABLE: "unreachable"}


def bulge_segments_to_polyline(segments, ccw_positive=True, max_seg_angle=2.0):
    """
    (x0, z0, x1, z1, bulge) segments -> list of (x, z) points.
    bulge = tan(included_angle / 4). With ccw_positive, a positive bulge is
    counter-clockwise in the view with Z to the right and X up (the G18
    view); otherwise clockwise.
    """
    pts = []
    for x0, z0, x1, z1, b in segments:
        if not pts or math.dist(pts[-1], (x0, z0)) > 1e-9:
            pts.append((x0, z0))
        if abs(b) < 1e-12:
            pts.append((x1, z1))
            continue
        theta = 4.0 * math.atan(b)  # signed included angle
        if not ccw_positive:
            theta = -theta
        # work in (u, v) = (z, x): CCW positive
        u0, v0, u1, v1 = z0, x0, z1, x1
        chord = math.hypot(u1 - u0, v1 - v0)
        r = chord / (2.0 * math.sin(abs(theta) / 2.0))
        mu, mv = (u0 + u1) / 2, (v0 + v1) / 2
        h = math.sqrt(max(r * r - chord * chord / 4, 0.0))
        nu, nv = -(v1 - v0) / chord, (u1 - u0) / chord  # left normal
        side = 1.0 if (theta > 0) == (abs(theta) < math.pi) else -1.0
        cu, cv = mu + side * h * nu, mv + side * h * nv
        a0 = math.atan2(v0 - cv, u0 - cu)
        n = max(2, int(math.ceil(abs(math.degrees(theta)) / max_seg_angle)))
        for k in range(1, n + 1):
            a = a0 + theta * k / n
            pts.append((cv + r * math.sin(a), cu + r * math.cos(a)))
    return pts


def part_profile(polylines, s, side=1.0):
    """
    Outer radius per column s from section polylines [(x, z), ...] of the
    part. side=+1 uses the +X half of the section, -1 the -X half (returned
    as positive radii). Columns the part doesn't cover get 0.
    """
    s = np.asarray(s, dtype=float)
    out = np.zeros_like(s)
    for pl in polylines:
        p = np.asarray(pl, dtype=float)
        if len(p) < 2:
            continue
        x = p[:, 0] * side
        z = p[:, 1]
        for i in range(len(p) - 1):
            xa, za, xb, zb = x[i], z[i], x[i + 1], z[i + 1]
            if xa <= 0 and xb <= 0:
                continue
            lo, hi = min(za, zb), max(za, zb)
            j0, j1 = np.searchsorted(s, lo, "left"), np.searchsorted(s, hi, "right")
            if j0 >= j1:
                continue
            ss = s[j0:j1]
            if abs(zb - za) < 1e-12:
                xv = np.full_like(ss, max(xa, xb))
            else:
                xv = xa + (ss - za) / (zb - za) * (xb - xa)
            np.maximum(out[j0:j1], np.maximum(xv, 0.0), out=out[j0:j1])
    return out


def reachable_profile(part_R, s, tool):
    """
    Lowest stock radius per column the tool can leave without cutting the
    part, approaching from outside. Tool placements are restricted to the
    column grid.
    """
    s = np.asarray(s, dtype=float)
    part_R = np.asarray(part_R, dtype=float)
    ds = s[1] - s[0] if len(s) > 1 else 1.0
    # include the columns holding the outline's ends (slightly conservative)
    kmin = int(math.floor(tool.zmin / ds + 1e-9))
    kmax = int(math.ceil(tool.zmax / ds - 1e-9))
    ks = np.arange(kmin, kmax + 1)
    # clip to the outline's extent so a column at the very tip still counts
    lows = tool.lower(np.clip(ks * ds, tool.zmin, tool.zmax))
    # pad both ends with "no part" so the tool may sit beyond the stock ends
    pad = int(max(abs(kmin), abs(kmax))) + 1
    P = np.concatenate([np.zeros(pad), part_R, np.zeros(pad)])
    n = len(P)
    # erosion: lowest control-point radius x(i) at column i with the tool clear of the part
    xmin = np.full(n, -np.inf)
    for k, L in zip(ks, lows):
        shifted = np.zeros(n)
        if k >= 0:
            shifted[: n - k] = P[k:]
        else:
            shifted[-k:] = P[: n + k]
        np.maximum(xmin, shifted - L, out=xmin)
    # dilation: lowest radius reached by any valid placement
    reach = np.full(n, np.inf)
    for k, L in zip(ks, lows):
        cand = np.full(n, np.inf)
        if k >= 0:
            cand[k:] = xmin[: n - k] + L
        else:
            cand[: n + k] = xmin[-k:] + L
        np.minimum(reach, cand, out=reach)
    reach = reach[pad:pad + len(part_R)]
    return np.maximum(reach, part_R)


@dataclass
class Range:
    kind: str
    s0: float
    s1: float
    worst: float      # deviation (mm), negative for gouges
    at: float         # s of the worst value


@dataclass
class CompareResult:
    deviation: np.ndarray
    classes: np.ndarray
    ranges: List[Range] = field(default_factory=list)

    def worst(self, kind):
        r = [x for x in self.ranges if x.kind == kind]
        if not r:
            return None
        return min(r, key=lambda x: x.worst) if kind == "gouge" else max(r, key=lambda x: x.worst)


def compare(stock_R, part_R, s, gouge_tol=0.01, uncut_tol=0.05, reachable=None):
    stock_R = np.asarray(stock_R, dtype=float)
    part_R = np.asarray(part_R, dtype=float)
    dev = stock_R - part_R
    cls = np.full(len(dev), OK, dtype=np.int8)
    cls[dev < -gouge_tol] = GOUGE
    left = dev > uncut_tol
    cls[left] = UNCUT
    if reachable is not None:
        cls[left & ((np.asarray(reachable) - part_R) > uncut_tol)] = UNREACHABLE
    ranges = []
    j = 0
    n = len(cls)
    while j < n:
        c = cls[j]
        k = j
        while k + 1 < n and cls[k + 1] == c:
            k += 1
        if c != OK:
            seg = dev[j:k + 1]
            w = int(np.argmin(seg)) if c == GOUGE else int(np.argmax(seg))
            ranges.append(Range(CLASS_NAMES[c], float(s[j]), float(s[k]), float(seg[w]), float(s[j + w])))
        j = k + 1
    return CompareResult(dev, cls, ranges)
