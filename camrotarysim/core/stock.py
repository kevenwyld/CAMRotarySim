# SPDX-License-Identifier: LGPL-2.1-or-later
"""
Rotary stock model: a cylindrical height field R(theta, s).

R[i, j] is the remaining radius at angle theta_i and axial position s_j
about a rotary axis. The stock is assumed solid from the axis out to R
(no bores yet).

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
        self.n_theta = int(n_theta)
        self.theta = np.arange(self.n_theta) * (2 * math.pi / self.n_theta)
        self.R = np.full((self.n_theta, n), float(radius))
        self.R0 = self.R.copy()

    @classmethod
    def from_profile(cls, profile_fn, s_min, s_max, ds, n_theta=1, axis=None):
        """Axisymmetric stock whose radius at s is profile_fn(s_array)."""
        st = cls(s_min, s_max, ds, 0.0, n_theta, axis)
        r = np.maximum(np.asarray(profile_fn(st.s), dtype=float), 0.0)
        st.R[:] = r[None, :]
        st.R0 = st.R.copy()
        return st

    @property
    def n_s(self):
        return len(self.s)

    def profile(self):
        """Axisymmetric view: the largest remaining radius per column."""
        return self.R.max(axis=0)

    def column_range(self, s_lo, s_hi):
        """Column index range [j0, j1) with s inside [s_lo, s_hi]."""
        j0 = int(np.searchsorted(self.s, s_lo, "left"))
        j1 = int(np.searchsorted(self.s, s_hi, "right"))
        return j0, j1

    def lower_all_angles(self, j0, j1, radius):
        """Turning: lower R to `radius` (array for columns j0..j1) at every angle."""
        np.minimum(self.R[:, j0:j1], np.maximum(radius, 0.0)[None, :], out=self.R[:, j0:j1])

    def lower_at_angles(self, theta_idx, j_idx, radius):
        """Rotary milling extension point: lower R only at the given cells."""
        self.R[theta_idx, j_idx] = np.minimum(self.R[theta_idx, j_idx], np.maximum(radius, 0.0))

    def snapshot(self):
        return self.R.copy()

    def restore(self, snap):
        self.R[:] = snap
