# SPDX-License-Identifier: LGPL-2.1-or-later
"""
Milling, indexed or rotary, in the workpiece frame.

The turning stock (one radius per angle and axial position about the
rotary axis) can't hold milled features: a pocket in a flat face has walls
parallel to the tool axis, not radial, so a ray from the axis goes through
material, the pocket and material again. Milling uses a signed-distance
grid instead: phi at grid points is the distance to the remaining
material's surface, positive inside. Cutting is CSG subtraction,
phi = min(phi, distance outside the cutter), exact at every tool position;
the surface is rebuilt at phi = 0 with sub-voxel interpolation (surface.py).

Frames (CAM's own, Path/Post/Processor.py and TiltedWorkPlane.py): an op on
a work plane stores its path in the plane's frame and carries the plane as
its Placement, so world = Placement * local and the tool axis is
Placement.Rotation * Z. A table rotary axis turns the workpiece:
machine = R(axis, A) * world (Path/Base/Generator/rotation.py
compute_rotation_matrix), so a position with an A word is in the workpiece
frame at R(axis, -A) * machine. Indexed ops have no A words; the machine's A
is the one that turns the plane's normal to +Z (needed_angle()).
"""

import math
from dataclasses import dataclass

import numpy as np

from .kinematics import Contact
from .stock import RotaryAxis

FLAT, BALL = "flat", "ball"
AXIS_INDEX = {"A": 3, "B": 4, "C": 5}


def rotation_matrix(axis, angle_deg):
    """Rotation by angle (degrees, right-handed) about a unit axis."""
    k = np.asarray(axis, dtype=float)
    k = k / np.linalg.norm(k)
    a = math.radians(angle_deg)
    K = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    return np.eye(3) + math.sin(a) * K + (1 - math.cos(a)) * (K @ K)


@dataclass
class MillTool:
    """A milling cutter: flat or ball end, radius, cutting length (from the tip up the tool axis)."""

    kind: str
    radius: float
    length: float
    source: str = ""

    @classmethod
    def from_cam(cls, shape_id, diameter, length, source=""):
        """From a CAM toolbit: ShapeID 'ballend' is a ball, anything else is treated as flat."""
        sid = str(shape_id or "").lower()
        kind = BALL if sid in ("ballend", "ball", "taperedballnose") else FLAT
        note = "" if sid in ("endmill", "ballend", "") else " (%s taken as a flat end mill)" % sid
        return cls(kind, float(diameter) / 2.0, float(length), source + note)

    def distance(self, pts, tip, axis):
        """Signed distance from pts (n, 3) to the cutter (negative inside)."""
        v = pts - tip
        t = v @ axis
        if self.kind == BALL:
            r = self.radius
            tt = np.clip(t - r, 0.0, max(self.length - r, 0.0))
            closest = tip + (r + tt)[:, None] * axis
            return np.linalg.norm(pts - closest, axis=1) - r
        radial = np.linalg.norm(v - t[:, None] * axis, axis=1)
        q1 = radial - self.radius
        q2 = np.maximum(-t, t - self.length)
        outside = np.hypot(np.maximum(q1, 0.0), np.maximum(q2, 0.0))
        return outside + np.minimum(np.maximum(q1, q2), 0.0)


class VoxelStock:
    """
    Signed distance (positive inside material) on a grid of points
    lo + (i, j, k) * h, covering the stock with a margin of empty points so
    its surface closes.
    """

    def __init__(self, lo, hi, h, axis=None):
        self.h = float(h)
        self.axis = axis or RotaryAxis(direction=(1.0, 0.0, 0.0))
        lo = np.asarray(lo, dtype=float) - 2 * self.h
        hi = np.asarray(hi, dtype=float) + 2 * self.h
        self.shape = tuple(int(math.ceil((b - a) / self.h)) + 1 for a, b in zip(lo, hi))
        self.lo = lo
        self.axes = [lo[i] + self.h * np.arange(self.shape[i]) for i in range(3)]
        self.phi = np.full(self.shape, -self.h * 4, dtype=np.float32)
        self.phi0 = self.phi.copy()

    @classmethod
    def from_box(cls, lo, hi, h, axis=None):
        st = cls(lo, hi, h, axis)
        c = (np.asarray(lo, float) + np.asarray(hi, float)) / 2
        half = (np.asarray(hi, float) - np.asarray(lo, float)) / 2
        X, Y, Z = np.meshgrid(*st.axes, indexing="ij")
        q = np.stack([np.abs(X - c[0]) - half[0], np.abs(Y - c[1]) - half[1], np.abs(Z - c[2]) - half[2]], -1)
        outside = np.linalg.norm(np.maximum(q, 0.0), axis=-1)
        inside = np.minimum(q.max(axis=-1), 0.0)
        st.phi = (-(outside + inside)).astype(np.float32)
        st.phi0 = st.phi.copy()
        st._bbox = st.material_bbox()
        return st

    @classmethod
    def from_sdf(cls, lo, hi, h, sdf_fn, axis=None):
        """Any stock: sdf_fn(grid) returns phi (positive inside) on the grid's points."""
        st = cls(lo, hi, h, axis)
        st.phi = np.asarray(sdf_fn(st), dtype=np.float32)
        st.phi0 = st.phi.copy()
        st._bbox = st.material_bbox()
        return st

    def index_box(self, lo, hi):
        """Index slices of the grid points inside the box [lo, hi], or None."""
        sl = []
        for i in range(3):
            a = int(math.floor((lo[i] - self.lo[i]) / self.h))
            b = int(math.ceil((hi[i] - self.lo[i]) / self.h)) + 1
            a, b = max(a, 0), min(b, self.shape[i])
            if a >= b:
                return None
            sl.append(slice(a, b))
        return tuple(sl)

    def points(self, sl):
        X, Y, Z = np.meshgrid(self.axes[0][sl[0]], self.axes[1][sl[1]], self.axes[2][sl[2]], indexing="ij")
        return np.stack([X.ravel(), Y.ravel(), Z.ravel()], axis=1)

    def material_bbox(self):
        idx = np.nonzero(self.phi > 0)
        if not len(idx[0]):
            return None
        return (np.array([self.axes[i][idx[i].min()] for i in range(3)]),
                np.array([self.axes[i][idx[i].max()] for i in range(3)]))

    def volume(self, phi=None):
        """Material volume: each grid point's cell counted by its distance (a point on a face, half)."""
        phi = self.phi if phi is None else phi
        return float(np.clip(phi / self.h + 0.5, 0.0, 1.0).sum()) * self.h ** 3

    def removed(self):
        return self.volume(self.phi0) - self.volume()

    def snapshot(self):
        return self.phi.copy()

    def restore(self, snap):
        self.phi[:] = snap


class MillingKinematics:
    """
    A cutter whose tip is the programmed point and whose axis is the op's
    frame Z, in the workpiece frame. frame: (R 3x3, t 3) of the op's
    Placement (world = R * local + t). The rotary axis' word (A, B or C) in
    a position turns the workpiece: world positions are rotated by -angle
    about the axis into the workpiece frame.
    """

    name = "milling"

    def __init__(self, axis=None, frame=None, letter="A"):
        self.axis = axis or RotaryAxis(direction=(1.0, 0.0, 0.0))
        R, t = frame if frame is not None else (np.eye(3), np.zeros(3))
        self.R = np.asarray(R, dtype=float)
        self.t = np.asarray(t, dtype=float)
        self.letter = letter
        self.index = AXIS_INDEX.get(letter, 3)

    def to_workpiece(self, pos):
        """(tip, unit tool axis) in the workpiece frame for a machine position (X Y Z A B C)."""
        p = self.R @ np.asarray(pos[:3], dtype=float) + self.t
        d = self.R[:, 2].copy()
        a = float(pos[self.index]) if len(pos) > self.index else 0.0
        if a:
            M = rotation_matrix(self.axis.direction, -a)
            o = np.asarray(self.axis.origin)
            p = M @ (p - o) + o
            d = M @ d
        return p, d / np.linalg.norm(d)

    def _region(self, stock, tool, tips, d):
        ends = tips + tool.length * d
        lo = np.minimum(tips.min(axis=0), ends.min(axis=0)) - tool.radius
        hi = np.maximum(tips.max(axis=0), ends.max(axis=0)) + tool.radius
        return lo, hi

    def may_touch(self, stock, tool, pts):
        """Whether any of the positions pts could reach material (a quick box test)."""
        bb = getattr(stock, "_bbox", None)   # the initial stock's: material only ever goes
        if bb is None:
            return True
        tips = []
        for p in (pts[0], pts[-1]):
            tip, d = self.to_workpiece(p)
            tips.append(tip)
        tips = np.array(tips)
        # positions between the two ends lie in their box, unless the rotary turns between them
        if len(pts) > 2 and abs(pts[-1][self.index] - pts[0][self.index]) > 1e-9:
            return True
        lo, hi = self._region(stock, tool, tips, d)
        return bool(np.all(lo <= bb[1] + stock.h) and np.all(hi >= bb[0] - stock.h))

    def apply(self, stock, tool, pos, max_radius=math.inf):
        tip, d = self.to_workpiece(pos)
        lo, hi = self._region(stock, tool, tip[None, :], d)
        sl = stock.index_box(lo - stock.h, hi + stock.h)
        if sl is None:
            return None
        block = stock.phi[sl]
        if not np.any(block > 0):
            return None
        pts = stock.points(sl)
        cut = tool.distance(pts, tip, d).reshape(block.shape).astype(np.float32)
        inside = (cut < 0) & (block > 0)
        if not inside.any():
            return None
        removed = float(np.max(np.minimum(block[inside], -cut[inside])))
        h = stock.h
        before = np.clip(block / h + 0.5, 0.0, 1.0).sum()
        np.minimum(block, cut, out=block)
        c = Contact(removed, 0.0)
        c.volume = float(before - np.clip(block / h + 0.5, 0.0, 1.0).sum()) * h ** 3
        return c


def placement_frame(position, quaternion):
    """(R, t) of a FreeCAD Placement given as position and quaternion (x, y, z, w)."""
    x, y, z, w = (float(v) for v in quaternion)
    R = np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                  [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                  [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])
    return R, np.asarray(position, dtype=float)


def needed_angle(normal, axis):
    """
    The rotary angle (degrees) that turns a work plane's normal to +Z for a
    table axis along `axis` (machine = R(axis, angle) * world), and how far
    the normal is from what that axis can reach (0 when exact). Matches the
    sign convention of CAM's compute_rotation_matrix.
    """
    k = np.asarray(axis, dtype=float)
    k = k / np.linalg.norm(k)
    n = np.asarray(normal, dtype=float)
    n = n / np.linalg.norm(n)
    # the component along the axis can't be turned away
    off = abs(float(n @ k)) - abs(float(np.array([0.0, 0.0, 1.0]) @ k))
    nz = np.array([0.0, 0.0, 1.0]) - (np.array([0.0, 0.0, 1.0]) @ k) * k
    np_ = n - (n @ k) * k
    if np.linalg.norm(np_) < 1e-12 or np.linalg.norm(nz) < 1e-12:
        return 0.0, abs(off)
    a = np_ / np.linalg.norm(np_)
    b = nz / np.linalg.norm(nz)
    angle = round(math.degrees(math.atan2(float(np.cross(a, b) @ k), float(a @ b))), 9) + 0.0
    reached = rotation_matrix(k, angle) @ n
    return angle, float(np.linalg.norm(reached - np.array([0.0, 0.0, 1.0])))


@dataclass
class MillCompare:
    """The simulated stock against the part, measured on the grid (both distances exact there)."""

    gouge: float            # deepest cut into the part (mm), 0 if none
    gouge_at: tuple
    gouge_volume: float     # mm3
    uncut: float            # most material left beyond the part (mm); at least `band` when clamped
    uncut_at: tuple
    uncut_volume: float
    band: float             # distances beyond this are not measured exactly


def compare_part(stock, part_phi, gouge_tol=0.01, uncut_tol=0.05, band=None):
    """
    Gouge: grid points cut (phi <= 0) that were stock (phi0 > 0) and are
    inside the part by more than gouge_tol. Uncut: points still holding
    material more than uncut_tol outside the part.
    """
    part_phi = np.asarray(part_phi)
    band = float(np.max(np.abs(part_phi))) if band is None else band
    v = stock.h ** 3
    gouge = (stock.phi <= 0) & (stock.phi0 > 0) & (part_phi > gouge_tol)
    uncut = (stock.phi > 0) & (part_phi < -uncut_tol)

    def worst(mask, values):
        if not mask.any():
            return 0.0, None
        i = np.argmax(np.where(mask, values, -np.inf))
        idx = np.unravel_index(i, stock.shape)
        return float(values[idx]), tuple(float(stock.axes[k][idx[k]]) for k in range(3))

    g, g_at = worst(gouge, part_phi)
    u, u_at = worst(uncut, -part_phi)
    return MillCompare(g, g_at, float(gouge.sum()) * v, u, u_at, float(uncut.sum()) * v, band)


def vertex_colours(stock, verts, part_phi=None, gouge_tol=0.01, uncut_tol=0.05):
    """
    RGB per surface vertex: stock colour on the original stock surface, cut
    colour where machined; with the part: red where the surface lies inside
    it (allowing for surface nets chamfering sharp edges by up to ~0.3 of a
    cell), amber where material is left beyond it.
    """
    from .mesh import CLASS_COLOURS, CUT_COLOUR, STOCK_COLOUR
    from .compare import GOUGE, UNCUT
    from .surface import sample
    col = np.tile(np.asarray(STOCK_COLOUR), (len(verts), 1))
    if not len(verts):
        return col
    machined = sample(stock.phi0, stock.lo, stock.h, verts, outside=-1.0) > 0.25 * stock.h
    col[machined] = CUT_COLOUR
    if part_phi is not None:
        dev = sample(part_phi, stock.lo, stock.h, verts, outside=0.0)
        col[dev < -uncut_tol] = CLASS_COLOURS[UNCUT]
        col[dev > gouge_tol + 0.3 * stock.h] = CLASS_COLOURS[GOUGE]
    return col


def format_milling(sim, cmp=None, ops_info=(), max_lines=8):
    """Report for a milling simulation: ops and their rotary angles, the part check, events, diagnostics."""
    out = ["Simulation %s: %d moves, %d samples." % (
        "complete" if sim.done else "%.0f%% simulated" % (100 * sim.progress), len(sim.moves), sim.total_samples)]
    if ops_info:
        out += ["", "Operations (tool axis in the workpiece, rotary angle that brings it to +Z):"]
        for info in ops_info:
            out.append("  " + info)
    if cmp is not None:
        out += ["", "Comparison with the part (on the %.2f mm grid):" % sim.stock.h]
        if cmp.gouge > 0:
            out.append("  Worst gouge: %.3f mm into the part at (%.2f, %.2f, %.2f); %.1f mm3"
                       % ((cmp.gouge,) + cmp.gouge_at + (cmp.gouge_volume,)))
        else:
            out.append("  Worst gouge: none")
        if cmp.uncut > 0:
            out.append("  Most left: %s%.3f mm beyond the part at (%.2f, %.2f, %.2f); %.1f mm3 left in all"
                       % ((">= " if cmp.uncut >= cmp.band - 1e-6 else "", min(cmp.uncut, cmp.band))
                          + cmp.uncut_at + (cmp.uncut_volume,)))
        else:
            out.append("  Material left beyond the part: none")
    out.append("")
    if sim.events:
        out.append("Events:")
        for e in sim.events[:max_lines * 2]:
            out.append("  %s #%d: %s" % (e.op_name, e.index, e.message))
    else:
        out.append("Events: none (no rapid cuts).")
    if sim.diagnostics:
        out += ["", "Path diagnostics:"]
        seen = {}
        for d in sim.diagnostics:
            seen.setdefault((sim.ops[d.op].name, d.message), []).append(d.index)
        for (op, msg), idx in seen.items():
            out.append("  %s: %s (%s)" % (op, msg, ", ".join("#%d" % i for i in idx[:6])))
    return "\n".join(out)
