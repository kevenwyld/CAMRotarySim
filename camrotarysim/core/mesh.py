# SPDX-License-Identifier: LGPL-2.1-or-later
"""
Triangle mesh of a RotaryStock for display (no Coin/FreeCAD here).
"""

import math

import numpy as np

from .compare import GOUGE, UNCUT, UNREACHABLE

# colours follow the new CAM simulator where it has one (stock/cut colours)
STOCK_COLOUR = (0.5, 0.55, 0.9)
CUT_COLOUR = (0.5, 0.84, 0.73)
CLASS_COLOURS = {
    GOUGE: (0.9, 0.15, 0.15),
    UNCUT: (0.95, 0.7, 0.1),
    UNREACHABLE: (0.75, 0.3, 0.85),
}


def column_colours(stock, classes=None, cut_tol=1e-6):
    """RGB per column: stock colour, cut colour where material was removed, class colours."""
    prof = stock.profile()
    cut = prof < stock.R0.max(axis=0) - cut_tol
    col = np.tile(np.asarray(STOCK_COLOUR), (len(prof), 1))
    col[cut] = CUT_COLOUR
    if classes is not None:
        for c, rgb in CLASS_COLOURS.items():
            col[classes == c] = rgb
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


def revolve(stock, colours=None, segments=72, tol=1e-4):
    """
    Mesh of the stock surface plus end caps.
    Returns (vertices (n, 3), triangles (m, 3) int, vertex colours (n, 3)).
    The axis is the stock axis (must be Z for now: vertices are global XYZ).
    """
    if colours is None:
        colours = column_colours(stock)
    if stock.n_theta == 1:
        prof = stock.R[0]
        cols = keep_columns(prof, colours, tol)
        ring = np.tile(prof[cols][None, :], (segments, 1))
        theta = np.arange(segments) * (2 * math.pi / segments)
    else:
        cols = np.arange(stock.n_s)
        ring = stock.R[:, cols]
        theta = stock.theta
        segments = stock.n_theta
    s = stock.s[cols]
    ox, oy, oz = stock.axis.origin
    nt, nc = ring.shape
    # surface vertices: index = t * nc + c
    X = ox + ring * np.cos(theta)[:, None]
    Y = oy + ring * np.sin(theta)[:, None]
    Z = oz + np.tile(s[None, :], (nt, 1))
    verts = np.stack([X.ravel(), Y.ravel(), Z.ravel()], axis=1)
    vcol = np.tile(colours[cols], (nt, 1))
    t = np.arange(nt)[:, None]
    c = np.arange(nc - 1)[None, :]
    a = t * nc + c
    b = ((t + 1) % nt) * nc + c
    tris = np.concatenate([
        np.stack([a, b, a + 1], axis=-1).reshape(-1, 3),
        np.stack([a + 1, b, b + 1], axis=-1).reshape(-1, 3),
    ])
    # end caps
    base = len(verts)
    caps_v = np.array([[ox, oy, oz + s[0]], [ox, oy, oz + s[-1]]])
    caps_c = np.array([colours[cols[0]], colours[cols[-1]]])
    ti = np.arange(nt)
    cap0 = np.stack([np.full(nt, base), ((ti + 1) % nt) * nc, ti * nc], axis=1)
    cap1 = np.stack([np.full(nt, base + 1), ti * nc + nc - 1, ((ti + 1) % nt) * nc + nc - 1], axis=1)
    verts = np.vstack([verts, caps_v])
    vcol = np.vstack([vcol, caps_c])
    tris = np.vstack([tris, cap0, cap1]).astype(np.int64)
    return verts, tris, vcol
