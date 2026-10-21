# SPDX-License-Identifier: LGPL-2.1-or-later
"""
Triangle mesh of a RotaryStock for display (no Coin/FreeCAD here).
"""

import math

import numpy as np

from .compare import GOUGE, THREAD, UNCUT, UNREACHABLE

# colours follow the new CAM simulator where it has one (stock/cut colours)
STOCK_COLOUR = (0.5, 0.55, 0.9)
CUT_COLOUR = (0.5, 0.84, 0.73)
THREAD_COLOURS = {"ok": (0.5, 0.84, 0.73), "shallow": (0.95, 0.7, 0.1), "deep": (0.9, 0.15, 0.15),
                  "": (0.55, 0.75, 0.85)}
CLASS_COLOURS = {
    GOUGE: (0.9, 0.15, 0.15),
    UNCUT: (0.95, 0.7, 0.1),
    UNREACHABLE: (0.75, 0.3, 0.85),
    THREAD: THREAD_COLOURS[""],
}


def column_colours(stock, classes=None, cut_tol=1e-6, thread_verdicts=()):
    """
    RGB per column: stock colour, cut colour where material was removed,
    class colours; thread zones by their check's verdict
    (thread_verdicts: ((z0, z1), verdict) pairs).
    """
    prof = stock.profile()
    cut = prof < stock.R0.max(axis=0) - cut_tol
    col = np.tile(np.asarray(STOCK_COLOUR), (len(prof), 1))
    col[cut] = CUT_COLOUR
    if classes is not None:
        for c, rgb in CLASS_COLOURS.items():
            col[classes == c] = rgb
    for (z0, z1), verdict in thread_verdicts:
        col[(stock.s >= z0) & (stock.s <= z1)] = THREAD_COLOURS.get(verdict, THREAD_COLOURS[""])
    return col


def bore_colours(stock, classes=None, cut_tol=1e-6, thread_verdicts=()):
    """RGB per column for the bore: stock colour, cut colour where it was bored, class colours, thread verdicts."""
    bore = stock.Rin.min(axis=0)
    cut = bore > stock.Rin0.min(axis=0) + cut_tol
    col = np.tile(np.asarray(STOCK_COLOUR), (stock.n_s, 1))
    col[cut] = CUT_COLOUR
    if classes is not None:
        for c, rgb in CLASS_COLOURS.items():
            col[classes == c] = rgb
    for (z0, z1), verdict in thread_verdicts:
        col[(stock.s >= z0) & (stock.s <= z1)] = THREAD_COLOURS.get(verdict, THREAD_COLOURS[""])
    return col


def keep_columns(R, colours, tol=1e-4):
    """Indices of columns needed to draw the profile within tol (drops collinear runs)."""
    n = len(R)
    if n <= 2:
        return np.arange(n)
    curv = np.abs(R[:-2] - 2 * R[1:-1] + R[2:]) > tol
    colour_change = np.any(colours[1:-1] != colours[:-2], axis=1) | np.any(colours[1:-1] != colours[2:], axis=1)
    keep = np.concatenate([[True], curv | colour_change, [True]])
    return np.nonzero(keep)[0]


def _ring_grid(stock, segments):
    """Angles and the (outer, inner) radius grids to draw: revolved or the stock's own angles."""
    if stock.n_theta == 1:
        theta = np.arange(segments) * (2 * math.pi / segments)
        return theta, np.repeat(stock.R, segments, axis=0), np.repeat(stock.Rin, segments, axis=0)
    return stock.theta, stock.R, stock.Rin


def _runs(mask):
    """[start, end) index pairs of the runs of True in mask."""
    d = np.diff(np.concatenate([[0], mask.astype(np.int8), [0]]))
    return list(zip(np.nonzero(d == 1)[0], np.nonzero(d == -1)[0]))


def revolve(stock, colours=None, segments=72, tol=1e-4, inner_colours=None, min_wall=1e-6):
    """
    Mesh of the stock: for each run of columns that still hold material, the
    outside, the bore (if any) and the two ends (rings, or discs without a
    bore), as one closed surface with outward normals.
    Returns (vertices (n, 3), triangles (m, 3) int, vertex colours (n, 3)).
    The axis is the stock axis (must be Z for now: vertices are global XYZ).
    """
    if colours is None:
        colours = column_colours(stock)
    if inner_colours is None:
        inner_colours = colours
    theta, RO, RI = _ring_grid(stock, segments)
    nt = len(theta)
    cos, sin = np.cos(theta)[:, None], np.sin(theta)[:, None]
    ox, oy, oz = stock.axis.origin
    wall = (RO - RI).max(axis=0)
    verts, tris, cols_out = [], [], []
    base = 0

    def surface(ring, s, col, flip):
        nonlocal base
        nc = ring.shape[1]
        v = np.stack([(ox + ring * cos).ravel(), (oy + ring * sin).ravel(), np.tile(oz + s[None, :], (nt, 1)).ravel()],
                     axis=1)
        t = np.arange(nt)[:, None]
        c = np.arange(nc - 1)[None, :]
        a = base + t * nc + c
        b = base + ((t + 1) % nt) * nc + c
        f = np.concatenate([np.stack([a, b, a + 1], axis=-1).reshape(-1, 3),
                            np.stack([a + 1, b, b + 1], axis=-1).reshape(-1, 3)])
        if flip:
            f = f[:, ::-1]
        verts.append(v)
        tris.append(f)
        cols_out.append(np.tile(col, (nt, 1)))
        start = base
        base += len(v)
        return start, nc

    for a, b in _runs(wall > min_wall):
        prof_o, prof_i = RO[:, a:b].max(axis=0), RI[:, a:b].min(axis=0)
        keep = np.zeros(b - a, dtype=bool)
        keep[keep_columns(prof_o, colours[a:b], tol)] = True
        keep[keep_columns(prof_i, inner_colours[a:b], tol)] = True
        varies = (np.ptp(RO[:, a:b], axis=0) > tol) | (np.ptp(RI[:, a:b], axis=0) > tol)
        keep |= varies | np.roll(varies, 1) | np.roll(varies, -1)
        idx = a + np.nonzero(keep)[0]
        s = stock.s[idx]
        outer = np.maximum(RO[:, idx], RI[:, idx])   # where a groove cuts through, zero thickness
        inner = RI[:, idx]
        o0, nc = surface(outer, s, colours[idx], False)
        ti = np.arange(nt)
        tn = (ti + 1) % nt
        if np.any(inner > 0):
            i0, _ = surface(inner, s, inner_colours[idx], True)
            for k, sign in ((0, -1), (nc - 1, 1)):   # end rings between the bore and the outside
                po, pi_, qo, qi = o0 + ti * nc + k, i0 + ti * nc + k, o0 + tn * nc + k, i0 + tn * nc + k
                f = np.concatenate([np.stack([po, qo, qi], 1), np.stack([po, qi, pi_], 1)])
                tris.append(f if sign > 0 else f[:, ::-1])
        else:
            centres = np.array([[ox, oy, oz + s[0]], [ox, oy, oz + s[-1]]])
            verts.append(centres)
            cols_out.append(np.array([colours[idx[0]], colours[idx[-1]]]))
            c0, c1 = base, base + 1
            base += 2
            tris.append(np.stack([np.full(nt, c0), o0 + tn * nc, o0 + ti * nc], 1))
            tris.append(np.stack([np.full(nt, c1), o0 + ti * nc + nc - 1, o0 + tn * nc + nc - 1], 1))
    if not verts:
        return np.zeros((0, 3)), np.zeros((0, 3), dtype=np.int64), np.zeros((0, 3))
    return np.vstack(verts), np.vstack(tris).astype(np.int64), np.vstack(cols_out)
