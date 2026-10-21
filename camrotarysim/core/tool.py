# SPDX-License-Identifier: LGPL-2.1-or-later
"""
Turning tool outline in the lathe XZ plane.

The outline is a closed polygon in (x, z), relative to the tool's control
point, the point the path is programmed to. For the TurningAddon ops that
is the insert's imaginary tip: the nose centre is at (x + r, z + r).

The outline normally comes from the tool solid of the op's ToolController
(sectioned in the XZ plane by the FreeCAD adapter). from_iso() and
from_threading() build it from the shape parameters, as a fallback and
for tests.

Control point by shape (TurningAddon TurnToolShapes.py and shape files):
- Isoshape: the shape origin is the sharp corner; paths are programmed to
  the imaginary tip, the nose centre minus (r, r) (REF_IMAGINARY_TIP);
- Threading, Grooving, FullRadius: the shape origin is the point the path
  is programmed to (REF_ORIGIN): the profile's sharp point for threading,
  the edge's lowest point at the chuck-side flank for grooving and full
  radius blades.
- CAM's Drill (and other revolved milling shapes): a revolution about Z
  with the tip at the origin, the body toward +Z; the tip is programmed
  (REF_ORIGIN). On a lathe it drills along the spindle axis at X 0, as an
  internal tool.

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

REF_IMAGINARY_TIP = "imaginary_tip"
REF_ORIGIN = "origin"


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
        self.internal = False
        self._build_envelope(envelope_step)

    # ------------------------------------------------------------------ build
    @classmethod
    def from_section(cls, edges, tip_radius=None, source="tool shape", tol=1e-4,
                     reference=REF_IMAGINARY_TIP):
        """
        Build from section edges of the tool solid. edges: list of dicts with
        'kind' ('line' | 'arc' | ...), 'points' [(x, z), ...] and, for arcs,
        'centre' (x, z) and 'radius'. Coordinates are the tool shape's own.
        reference: REF_IMAGINARY_TIP (Isoshape) or REF_ORIGIN (the shape
        origin is the control point).
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
        if reference == REF_ORIGIN:
            if nose is not None:
                return cls(loop, (0.0, 0.0), nose["centre"], float(nose["radius"]), source)
            return cls(loop, (0.0, 0.0), None, 0.0, source)
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

    @classmethod
    def from_threading(cls, profile_angle, tip_radius, width, height=None, nose_segments=16):
        """
        Threading insert from the Threading shape parameters: a V of
        profile_angle pointing toward the axis (-X), symmetric about Z = 0,
        with tip_radius at its tip, flanks up to the insert's width, then
        straight. The control point is the V's sharp point, at the origin.
        """
        half = math.radians(profile_angle / 2.0)
        r = max(float(tip_radius), 0.0)
        w2 = float(width) / 2.0
        x_flank_top = w2 / math.tan(half)  # where the flanks reach the full width
        top = x_flank_top + (float(height) if height else float(width))
        if r > 0:
            cx = r / math.sin(half)
            # tangent points on the flanks, r / tan(half) from the sharp point
            t = r / math.tan(half)
            beta = math.atan2(t * math.sin(half), t * math.cos(half) - cx)  # +Z tangent point
            pts = [(cx + r * math.cos(a), r * math.sin(a))
                   for a in np.linspace(beta, 2 * math.pi - beta, nose_segments + 1)]
            nose = (cx, 0.0)
        else:
            pts = [(0.0, 0.0)]
            nose = None
        pts += [(x_flank_top, -w2), (top, -w2), (top, w2), (x_flank_top, w2)]
        return cls(pts, (0.0, 0.0), nose, r, "Threading parameters")

    @classmethod
    def from_drill(cls, diameter, tip_angle, length=None):
        """
        A twist drill's section in the XZ plane, from CAM's Drill shape
        parameters: point of tip_angle at the origin, body of diameter to
        length (default 3 diameters past the point) toward +Z.
        """
        r = float(diameter) / 2.0
        point = r / math.tan(math.radians(tip_angle) / 2.0) if 0 < tip_angle < 180 else 0.0
        top = float(length) if length else point + 3 * float(diameter)
        pts = [(0.0, 0.0), (r, point), (r, top), (-r, top), (-r, point)]
        if point == 0.0:
            pts = [(r, 0.0), (r, top), (-r, top), (-r, 0.0)]
        return cls(pts, (0.0, 0.0), None, 0.0, "Drill parameters")

    def mirrored(self):
        """
        The outline mirrored in X about the control point: an external tool
        used inside (boring, internal threading), with its body toward the
        axis. The TurningAddon bores by mirroring external turning, so the
        same insert's reference point mirrors with it.
        """
        m = ToolOutline.__new__(ToolOutline)
        m.points = self.points * np.array([-1.0, 1.0])
        if _polygon_area(m.points) < 0:
            m.points = m.points[::-1]
        m.control_in_shape = self.control_in_shape
        m.nose_radius = self.nose_radius
        m.nose_centre = None if self.nose_centre is None else (-self.nose_centre[0], self.nose_centre[1])
        m.source = self.source + ", mirrored for internal use"
        m.internal = not getattr(self, "internal", False)
        m._build_envelope(self._env_z[1] - self._env_z[0] if len(self._env_z) > 1 else 0.005)
        return m

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
