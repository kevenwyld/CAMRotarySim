# SPDX-License-Identifier: LGPL-2.1-or-later
"""
Surfaces of signed-distance grids (VoxelStock), and grids from triangle meshes.

- surface_nets(): the phi = 0 surface as triangles (naive surface nets: a
  vertex per grid cell the surface crosses, at the mean of its edges'
  interpolated crossings; a quad per grid edge the surface crosses),
  outward facing.
- sample(): trilinear interpolation of a grid at points.
- mesh_sdf(): a signed distance grid (positive inside) of a closed triangle
  mesh: inside from ray parity along X, exact distances within a band
  around the surface, clamped beyond it.
"""

import numpy as np

# the 12 edges of a cell as pairs of corner offsets
_CORNERS = np.array([(i, j, k) for i in (0, 1) for j in (0, 1) for k in (0, 1)])
_EDGES = [(a, b) for a in range(8) for b in range(a + 1, 8)
          if np.abs(_CORNERS[a] - _CORNERS[b]).sum() == 1]


def surface_nets(phi, lo, h):
    """(vertices (n, 3), triangles (m, 3)) of phi = 0, triangles facing out of phi > 0."""
    phi = np.asarray(phi, dtype=np.float64)
    inside = phi > 0
    nx, ny, nz = phi.shape
    cells = np.zeros((nx - 1, ny - 1, nz - 1), dtype=np.int8)
    for c in _CORNERS:
        cells += inside[c[0]:nx - 1 + c[0], c[1]:ny - 1 + c[1], c[2]:nz - 1 + c[2]]
    active = (cells > 0) & (cells < 8)
    ci = np.nonzero(active)
    n = len(ci[0])
    if n == 0:
        return np.zeros((0, 3)), np.zeros((0, 3), dtype=np.int64)
    base = np.stack(ci, axis=1)
    vals = np.stack([phi[base[:, 0] + c[0], base[:, 1] + c[1], base[:, 2] + c[2]] for c in _CORNERS], axis=1)
    acc = np.zeros((n, 3))
    cnt = np.zeros(n)
    for a, b in _EDGES:
        va, vb = vals[:, a], vals[:, b]
        m = (va > 0) != (vb > 0)
        t = np.where(m, va / np.where(m, va - vb, 1.0), 0.0)
        acc += m[:, None] * (_CORNERS[a] + t[:, None] * (_CORNERS[b] - _CORNERS[a]))
        cnt += m
    verts = np.asarray(lo) + h * (base + acc / np.maximum(cnt, 1)[:, None])
    vid = np.full(cells.shape, -1, dtype=np.int64)
    vid[ci] = np.arange(n)
    quads = []
    # an edge along axis `ax` between grid points p and p + e_ax, crossed by the surface,
    # is shared by the four cells around it
    for ax in range(3):
        o1, o2 = [a for a in range(3) if a != ax]
        sl0 = [slice(None)] * 3
        sl1 = [slice(None)] * 3
        sl0[ax] = slice(0, phi.shape[ax] - 1)
        sl1[ax] = slice(1, phi.shape[ax])
        a, b = inside[tuple(sl0)], inside[tuple(sl1)]
        cross = a != b
        # the four cells need index >= 1 along both other axes and within the cell grid
        cross[tuple(slice(0, 1) if i == o1 else slice(None) for i in range(3))] = False
        cross[tuple(slice(0, 1) if i == o2 else slice(None) for i in range(3))] = False
        lim = [s - 1 for s in phi.shape]
        cross = cross[tuple(slice(0, lim[i]) if i != ax else slice(None) for i in range(3))]
        e = np.stack(np.nonzero(cross), axis=1)
        if not len(e):
            continue
        out_dir = a[tuple(e.T)]            # material at the lower point: the normal points +ax

        def cell(d1, d2):
            c = e.copy()
            c[:, o1] -= d1
            c[:, o2] -= d2
            return vid[tuple(c.T)]

        q = np.stack([cell(1, 1), cell(0, 1), cell(0, 0), cell(1, 0)], axis=1)
        # (o1, o2, ax) right-handed for ax = 0 and 2, left-handed for ax = 1
        flip = ~out_dir if ax != 1 else out_dir
        q[flip] = q[flip][:, ::-1]
        quads.append(q)
    if not quads:
        return verts, np.zeros((0, 3), dtype=np.int64)
    q = np.vstack(quads)
    q = q[np.all(q >= 0, axis=1)]
    tris = np.vstack([q[:, [0, 1, 2]], q[:, [0, 2, 3]]])
    return verts, tris


def sample(grid, lo, h, pts, outside=None):
    """Trilinear interpolation of grid (values at lo + i h) at pts (n, 3)."""
    g = np.asarray(grid, dtype=np.float64)
    f = (np.asarray(pts, dtype=float) - np.asarray(lo)) / h
    i0 = np.floor(f).astype(int)
    shape = np.array(g.shape)
    ok = np.all((i0 >= 0) & (i0 < shape - 1), axis=1)
    i0 = np.clip(i0, 0, shape - 2)
    t = np.clip(f - i0, 0.0, 1.0)
    out = np.zeros(len(f))
    for c in _CORNERS:
        w = np.prod(np.where(c, t, 1 - t), axis=1)
        out += w * g[i0[:, 0] + c[0], i0[:, 1] + c[1], i0[:, 2] + c[2]]
    if outside is not None:
        out[~ok] = outside
    return out


def _point_triangle_distance(P, A, B, C):
    """Distances from points P (m, 3) to triangles (A, B, C each (n, 3)): (m, n). Ericson's closest point."""
    P = P[:, None, :]
    ab, ac, ap = B - A, C - A, P - A
    d1, d2 = np.sum(ab * ap, -1), np.sum(ac * ap, -1)
    bp = P - B
    d3, d4 = np.sum(ab * bp, -1), np.sum(ac * bp, -1)
    cp = P - C
    d5, d6 = np.sum(ab * cp, -1), np.sum(ac * cp, -1)
    va = d3 * d6 - d5 * d4
    vb = d5 * d2 - d1 * d6
    vc = d1 * d4 - d3 * d2
    denom = va + vb + vc
    with np.errstate(divide="ignore", invalid="ignore"):
        v = vb / denom
        w = vc / denom
        q = A + v[..., None] * ab + w[..., None] * ac            # inside the face
        t_ab = d1 / (d1 - d3)
        t_ac = d2 / (d2 - d6)
        t_bc = (d4 - d3) / ((d4 - d3) + (d5 - d6))
    cond = [
        (d1 <= 0) & (d2 <= 0),                                    # A
        (d3 >= 0) & (d4 <= d3),                                   # B
        (d6 >= 0) & (d5 <= d6),                                   # C
        (vc <= 0) & (d1 >= 0) & (d3 <= 0),                        # AB
        (vb <= 0) & (d2 >= 0) & (d6 <= 0),                        # AC
        (va <= 0) & ((d4 - d3) >= 0) & ((d5 - d6) >= 0),          # BC
    ]
    cands = [A + 0 * P, B + 0 * P, C + 0 * P, A + t_ab[..., None] * ab, A + t_ac[..., None] * ac,
             B + t_bc[..., None] * (C - B)]
    for cnd, cand in zip(reversed(cond), reversed(cands)):
        q = np.where(cnd[..., None], cand, q)
    return np.linalg.norm(P - q, axis=-1)


def mesh_inside(lo, h, shape, tris_xyz, eps=1e-7):
    """Grid points inside a closed triangle mesh (n, 3, 3), by crossings of rays along +X."""
    ys = lo[1] + h * np.arange(shape[1]) + eps
    zs = lo[2] + h * np.arange(shape[2]) + 1.3 * eps
    hits = {}
    for tri in tris_xyz:
        (x0, y0, z0), (x1, y1, z1), (x2, y2, z2) = tri
        j0 = int(np.searchsorted(ys, min(y0, y1, y2)))
        j1 = int(np.searchsorted(ys, max(y0, y1, y2), "right"))
        k0 = int(np.searchsorted(zs, min(z0, z1, z2)))
        k1 = int(np.searchsorted(zs, max(z0, z1, z2), "right"))
        if j0 >= j1 or k0 >= k1:
            continue
        Y, Z = np.meshgrid(ys[j0:j1], zs[k0:k1], indexing="ij")
        det = (y1 - y0) * (z2 - z0) - (y2 - y0) * (z1 - z0)
        if abs(det) < 1e-15:
            continue
        u = ((Y - y0) * (z2 - z0) - (y2 - y0) * (Z - z0)) / det
        v = ((y1 - y0) * (Z - z0) - (Y - y0) * (z1 - z0)) / det
        m = (u >= 0) & (v >= 0) & (u + v <= 1)
        if not m.any():
            continue
        X = x0 + u * (x1 - x0) + v * (x2 - x0)
        for j, k, x in zip(*np.nonzero(m), X[m]):
            hits.setdefault((j0 + j, k0 + k), []).append(x)
    xs = lo[0] + h * np.arange(shape[0])
    inside = np.zeros(shape, dtype=bool)
    for (j, k), xl in hits.items():
        xl = np.sort(xl)
        for a, b in zip(xl[0::2], xl[1::2]):
            inside[(xs > a) & (xs < b), j, k] = True
    return inside


def mesh_sdf(lo, h, shape, tris_xyz, band=3):
    """Signed distance (positive inside) of a closed mesh on the grid lo + i h; exact within band cells."""
    tris_xyz = np.asarray(tris_xyz, dtype=float)
    inside = mesh_inside(lo, h, shape, tris_xyz)
    sign = np.where(inside, 1.0, -1.0)
    # points near the surface: an inside/outside change within `band` points
    near = np.zeros(shape, dtype=bool)
    for ax in range(3):
        d = np.diff(inside.astype(np.int8), axis=ax) != 0
        sl0 = [slice(None)] * 3
        sl1 = [slice(None)] * 3
        sl0[ax] = slice(0, shape[ax] - 1)
        sl1[ax] = slice(1, shape[ax])
        near[tuple(sl0)] |= d
        near[tuple(sl1)] |= d
    for _ in range(2 * band):    # face neighbours: reach diagonals too
        grown = near.copy()
        for ax in range(3):
            grown[tuple(slice(1, None) if i == ax else slice(None) for i in range(3))] |= \
                near[tuple(slice(0, -1) if i == ax else slice(None) for i in range(3))]
            grown[tuple(slice(0, -1) if i == ax else slice(None) for i in range(3))] |= \
                near[tuple(slice(1, None) if i == ax else slice(None) for i in range(3))]
        near = grown
    phi = sign * band * h
    idx = np.stack(np.nonzero(near), axis=1)
    if len(idx):
        P = np.asarray(lo) + h * idx
        A, B, C = tris_xyz[:, 0], tris_xyz[:, 1], tris_xyz[:, 2]
        tmin, tmax = tris_xyz.min(axis=1), tris_xyz.max(axis=1)
        reach = band * h * 1.01     # beyond that the distance is clamped anyway
        # buckets of points, each against the triangles whose boxes come near it
        cell = 6 * h
        keys = np.floor((P - np.asarray(lo)) / cell).astype(int)
        order = np.lexsort(keys.T)
        keys, P_sorted, idx_sorted = keys[order], P[order], idx[order]
        splits = np.nonzero(np.any(np.diff(keys, axis=0) != 0, axis=1))[0] + 1
        for grp in np.split(np.arange(len(P_sorted)), splits):
            pts = P_sorted[grp]
            plo, phi_ = pts.min(axis=0) - reach, pts.max(axis=0) + reach
            sel = np.all((tmax >= plo) & (tmin <= phi_), axis=1)
            if not sel.any():
                continue
            dist = _point_triangle_distance(pts, A[sel], B[sel], C[sel]).min(axis=1)
            ii = idx_sorted[grp]
            phi[ii[:, 0], ii[:, 1], ii[:, 2]] = sign[ii[:, 0], ii[:, 1], ii[:, 2]] * np.minimum(dist, band * h)
    return phi.astype(np.float32)
