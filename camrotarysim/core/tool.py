# SPDX-License-Identifier: LGPL-2.1-or-later
"""
Turning tool outline in the lathe XZ plane.

The outline is a closed polygon in (x, z), relative to the tool's control
point, the point the path is programmed to. For the TurningAddon ops that
is the insert's imaginary tip: the nose centre is at (x + r, z + r).

The outline normally comes from the tool solid of the op's ToolController
(sectioned in the XZ plane by the FreeCAD adapter). from_iso() builds the
same outline from the Isoshape parameters, as a fallback and for tests.

Isoshape conventions (checked against isoshape.fcstd, whose sketch binds
TipAngle, -Rotation, EdgeLength and TipRadius, and against
liblathe Tool.get_tool_cutting_angle()):
- the shape origin is the theoretical sharp corner of the active tip;
- the two edges at the active tip leave it at Rotation - TipAngle/2 and
  Rotation + TipAngle/2, measured from +Z toward +X;
- the insert is a rhombus with side EdgeLength between sharp corners and
  every corner rounded with TipRadius.
"""

import math

import numpy as np


def _polygon_area(p):
    x, z = p[:, 0], p[:, 1]
    return 0.5 * float(np.sum(x * np.roll(z, -1) - np.roll(x, -1) * z))


def _dedupe(points, tol=1e-9):
    out = [points[0]]
    for p in points[1:]:
        if math.dist(p, out[-1]) > tol:
            out.append(p)
    if len(out) > 1 and math.dist(out[0], out[-1]) <= tol:
        out.pop()
    return out


def chain_edges(edges, tol=1e-4):
    """
    Chain section edges (each a dict with 'points', a list of (x, z)) into
    closed loops. Returns a list of point lists.
    """
    remaining = [list(map(tuple, e["points"])) for e in edges if len(e["points"]) >= 2]
    loops = []
    while remaining:
        loop = list(remaining.pop(0))
        progressed = True
        while progressed and math.dist(loop[0], loop[-1]) > tol:
            progressed = False
            for i, pts in enumerate(remaining):
                if math.dist(pts[0], loop[-1]) <= tol:
                    loop.extend(pts[1:])
                elif math.dist(pts[-1], loop[-1]) <= tol:
                    loop.extend(reversed(pts[:-1]))
                else:
                    continue
                remaining.pop(i)
                progressed = True
                break
        loops.append(loop)
    return loops


class ToolOutline:
    """Closed tool outline in (x, z) relative to the control point."""

    def __init__(self, points, control=(0.0, 0.0), nose_centre=None, nose_radius=0.0,
                 source="", envelope_step=0.005):
        pts = np.asarray(_dedupe([tuple(p) for p in points]), dtype=float)
        if len(pts) < 3:
            raise ValueError("tool outline needs at least 3 points")
        if _polygon_area(pts) < 0:
            pts = pts[::-1]
        self.control_in_shape = (float(control[0]), float(control[1]))
        self.points = pts - np.asarray(control, dtype=float)
        self.nose_radius = float(nose_radius)
        self.nose_centre = None
        if nose_centre is not None:
            self.nose_centre = (nose_centre[0] - control[0], nose_centre[1] - control[1])
        self.source = source
        self._build_envelope(envelope_step)

    # ------------------------------------------------------------------ build
    @classmethod
    def from_section(cls, edges, tip_radius=None, source="tool shape", tol=1e-4):
        """
        Build from section edges of the tool solid. edges: list of dicts with
        'kind' ('line' | 'arc' | ...), 'points' [(x, z), ...] and, for arcs,
        'centre' (x, z) and 'radius'. Coordinates are the tool shape's own
        (origin = sharp corner of the active tip for Isoshape tools).
        """
        loops = chain_edges(edges, tol)
        if not loops:
            raise ValueError("tool section is empty")
        loops = [lp for lp in loops if len(lp) >= 3]
        if not loops:
            raise ValueError("tool section has no closed outline")
        loop = max(loops, key=lambda lp: abs(_polygon_area(np.asarray(lp))))
        arcs = [e for e in edges if e.get("kind") == "arc" and e.get("radius")]
        nose = None
        if arcs:
            if tip_radius:
                cand = [a for a in arcs if abs(a["radius"] - tip_radius) <= max(1e-3, 0.01 * tip_radius)]
                arcs = cand or arcs
            nose = min(arcs, key=lambda a: math.hypot(*a["centre"]))
        if nose is not None:
            r = float(nose["radius"])
            cx, cz = nose["centre"]
            control = (cx - r, cz - r)
            return cls(loop, control, (cx, cz), r, source)
        arr = np.asarray(loop)
        return cls(loop, (float(arr[:, 0].min()), float(arr[:, 1].min())), None, 0.0, source + " (no nose arc)")

    @classmethod
    def from_iso(cls, edge_length, tip_angle, tip_radius, rotation, nose_segments=24):
        """Rhombus insert from the Isoshape parameters (lengths in mm, angles in degrees)."""
        L, r = float(edge_length), float(tip_radius)
        a1 = math.radians(rotation - tip_angle / 2.0)
        a2 = math.radians(rotation + tip_angle / 2.0)
        d1 = np.array([math.sin(a1), math.cos(a1)])
        d2 = np.array([math.sin(a2), math.cos(a2)])
        corners = [np.zeros(2), L * d1, L * d1 + L * d2, L * d2]
        pts = []
        nose_centre = None
        for i, c in enumerate(corners):
            prev_c, next_c = corners[i - 1], corners[(i + 1) % 4]
            u1 = (prev_c - c) / np.linalg.norm(prev_c - c)
            u2 = (next_c - c) / np.linalg.norm(next_c - c)
            half = math.acos(max(-1.0, min(1.0, float(np.dot(u1, u2))))) / 2.0
            if r <= 0:
                pts.append(tuple(c))
                if i == 0:
                    nose_centre = (0.0, 0.0)
                continue
            bis = (u1 + u2) / np.linalg.norm(u1 + u2)
            centre = c + bis * (r / math.sin(half))
            t1 = c + u1 * (r / math.tan(half))
            t2 = c + u2 * (r / math.tan(half))
            b1 = math.atan2(t1[1] - centre[1], t1[0] - centre[0])
            b2 = math.atan2(t2[1] - centre[1], t2[0] - centre[0])
            sweep = (b2 - b1 + math.pi) % (2 * math.pi) - math.pi
            for k in range(nose_segments + 1):
                b = b1 + sweep * k / nose_segments
                pts.append((centre[0] + r * math.cos(b), centre[1] + r * math.sin(b)))
            if i == 0:
                nose_centre = (float(centre[0]), float(centre[1]))
        control = (nose_centre[0] - r, nose_centre[1] - r)
        return cls(pts, control, nose_centre, r, "Isoshape parameters")

    # --------------------------------------------------------------- envelope
    def _build_envelope(self, step):
        p = self.points
        q = np.roll(p, -1, axis=0)
        self.zmin, self.zmax = float(p[:, 1].min()), float(p[:, 1].max())
        self.xmin, self.xmax = float(p[:, 0].min()), float(p[:, 0].max())
        n = max(2, int(math.ceil((self.zmax - self.zmin) / step)) + 1)
        zs = np.linspace(self.zmin, self.zmax, n)
        za, zb = p[:, 1][None, :], q[:, 1][None, :]
        xa, xb = p[:, 0][None, :], q[:, 0][None, :]
        zk = zs[:, None]
        lo_z, hi_z = np.minimum(za, zb), np.maximum(za, zb)
        hit = (zk >= lo_z - 1e-12) & (zk <= hi_z + 1e-12)
        dz = zb - za
        with np.errstate(divide="ignore", invalid="ignore"):
            t = np.where(np.abs(dz) > 1e-15, (zk - za) / dz, 0.0)
        x = xa + np.clip(t, 0.0, 1.0) * (xb - xa)
        flat = np.abs(dz) <= 1e-15  # horizontal edges: both ends count
        xlo = np.where(flat, np.minimum(xa, xb), x)
        xhi = np.where(flat, np.maximum(xa, xb), x)
        self._env_z = zs
        self._env_lo = np.where(hit, xlo, np.inf).min(axis=1)
        self._env_hi = np.where(hit, xhi, -np.inf).max(axis=1)

    def lower(self, dz):
        """Lowest x of the outline at axial offset dz (inf outside the outline)."""
        return np.interp(dz, self._env_z, self._env_lo, left=np.inf, right=np.inf)

    def upper(self, dz):
        """Highest x of the outline at axial offset dz (-inf outside the outline)."""
        return np.interp(dz, self._env_z, self._env_hi, left=-np.inf, right=-np.inf)
