# SPDX-License-Identifier: LGPL-2.1-or-later
"""
Rotary stock model: a cylindrical height field R(theta, s).

R[i, j] is the remaining outer radius at angle theta_i and axial position
s_j about a rotary axis, Rin[i, j] the inner radius (a bore; 0 where the
stock is solid to the axis). Material lies between Rin and R; a cell with
R <= Rin is empty (cut through, or faced away).

- Turning lowers R for every theta (the workpiece spins), so an
  axisymmetric stock uses n_theta = 1.
- Rotary milling (later phase) lowers R only inside the cutter footprint
  at one angle; that needs n_theta > 1. See kinematics.RotaryMillingKinematics.
"""

import math
from dataclasses import dataclass, field

import numpy as np


@dataclass
class RotaryAxis:
    """A rotary axis: a line through origin along direction (unit vector)."""

    origin: tuple = (0.0, 0.0, 0.0)
    direction: tuple = (0.0, 0.0, 1.0)
    name: str = ""
    limits: tuple = (-math.inf, math.inf)
    extra: dict = field(default_factory=dict)

    def __post_init__(self):
        d = np.asarray(self.direction, dtype=float)
        n = np.linalg.norm(d)
        if n < 1e-12:
            raise ValueError("rotary axis direction is zero")
        self.direction = tuple(float(v) for v in d / n)
        self.origin = tuple(float(v) for v in self.origin)

    @classmethod
    def from_machine_axis(cls, name, data):
        """
        From one entry of a FreeCAD machine file (.fcm) 'machine.axes', e.g.
        {"type": "rotary", "joint": [[0,0,0],[1,0,0]], "limits": {...}}.
        """
        joint = data.get("joint") or [[0, 0, 0], [0, 0, 1]]
        lim = data.get("limits") or {}
        extra = {k: v for k, v in data.items() if k not in ("joint", "limits")}
        return cls(tuple(joint[0]), tuple(joint[1]), name,
                   (float(lim.get("min", -math.inf)), float(lim.get("max", math.inf))), extra)


def rotary_axes_from_machine(machine_json):
    """All rotary axes of a parsed .fcm machine file, by name."""
    axes = (machine_json.get("machine") or machine_json).get("axes") or {}
    return {n: RotaryAxis.from_machine_axis(n, a) for n, a in axes.items() if a.get("type") == "rotary"}


class RotaryStock:
    """Cylindrical height field about `axis`, columns at s = s_min + (j + 0.5) * ds."""

    def __init__(self, s_min, s_max, ds, radius=0.0, n_theta=1, axis=None):
        if s_max <= s_min:
            raise ValueError("stock has no length along its axis")
        self.axis = axis or RotaryAxis()
        self.ds = float(ds)
        n = max(1, int(math.ceil((s_max - s_min) / ds)))
        self.s_min = float(s_min)
        self.s_max = self.s_min + n * self.ds
        self.s = self.s_min + (np.arange(n) + 0.5) * self.ds
        self._set_shape(np.full((int(n_theta), n), float(radius)), np.zeros((int(n_theta), n)))
        self.R0 = self.R.copy()
        self.Rin0 = self.Rin.copy()

    @classmethod
    def from_profile(cls, profile_fn, s_min, s_max, ds, n_theta=1, axis=None, inner_fn=None):
        """Axisymmetric stock with outer radius profile_fn(s) and inner radius inner_fn(s) (default 0)."""
        st = cls(s_min, s_max, ds, 0.0, n_theta, axis)
        st.R[:] = np.maximum(np.asarray(profile_fn(st.s), dtype=float), 0.0)[None, :]
        if inner_fn is not None:
            st.Rin[:] = np.maximum(np.asarray(inner_fn(st.s), dtype=float), 0.0)[None, :]
        st.R0 = st.R.copy()
        st.Rin0 = st.Rin.copy()
        return st

    def add_hole(self, radius, z_lo, z_hi):
        """A hole of `radius` from z_lo to z_hi in the initial stock (e.g. drilled before the program)."""
        j0, j1 = self.column_range(min(z_lo, z_hi), max(z_lo, z_hi))
        np.maximum(self.Rin[:, j0:j1], float(radius), out=self.Rin[:, j0:j1])
        self.Rin0 = self.Rin.copy()

    @property
    def n_s(self):
        return len(self.s)

    def profile(self):
        """Axisymmetric view of the outside: the largest remaining radius per column."""
        return self.R.max(axis=0)

    def outer_profile(self, tol=1e-6):
        """The outside per column, 0 where the column is empty (no material left)."""
        return np.where(self.material() > tol, self.profile(), 0.0)

    def inner_profile(self):
        """Axisymmetric view of the bore: the smallest bore radius per column."""
        return self.Rin.min(axis=0)

    def material(self):
        """Thickest wall per column (R - Rin, <= 0 where the column is empty)."""
        return (self.R - self.Rin).max(axis=0)

    def has_bore(self):
        return bool(np.any(self.Rin > 0))

    def column_range(self, s_lo, s_hi):
        """Column index range [j0, j1) with s inside [s_lo, s_hi]."""
        j0 = int(np.searchsorted(self.s, s_lo, "left"))
        j1 = int(np.searchsorted(self.s, s_hi, "right"))
        return j0, j1

    def lower_all_angles(self, j0, j1, radius):
        """Turning: lower R to `radius` (array for columns j0..j1) at every angle."""
        np.minimum(self.R[:, j0:j1], np.maximum(radius, 0.0)[None, :], out=self.R[:, j0:j1])

    def raise_bore_all_angles(self, j0, j1, radius):
        """Boring: widen the bore to `radius` (array for columns j0..j1) at every angle."""
        np.maximum(self.Rin[:, j0:j1], np.maximum(radius, 0.0)[None, :], out=self.Rin[:, j0:j1])

    def lower_at_angles(self, theta_idx, j_idx, radius):
        """Rotary milling extension point: lower R only at the given cells."""
        self.R[theta_idx, j_idx] = np.minimum(self.R[theta_idx, j_idx], np.maximum(radius, 0.0))

    def set_angles(self, n_theta):
        """
        Give an axisymmetric stock n_theta angles (needed once a cut depends
        on the spindle angle, e.g. threading). Only from n_theta = 1.
        """
        if n_theta == self.n_theta:
            return
        if self.n_theta != 1:
            raise ValueError("can only add angles to an axisymmetric stock")
        self._set_shape(np.repeat(self.R, int(n_theta), axis=0), np.repeat(self.Rin, int(n_theta), axis=0))

    def _set_shape(self, R, Rin):
        self.R = R
        self.Rin = Rin
        self.n_theta = R.shape[0]
        self.theta = np.arange(self.n_theta) * (2 * math.pi / self.n_theta)

    def snapshot(self):
        return np.stack([self.R, self.Rin])

    def restore(self, snap):
        if snap.shape[1:] == self.R.shape:
            self.R[:] = snap[0]
            self.Rin[:] = snap[1]
        else:
            self._set_shape(snap[0].copy(), snap[1].copy())
