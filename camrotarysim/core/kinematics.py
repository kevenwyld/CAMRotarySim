# SPDX-License-Identifier: LGPL-2.1-or-later
"""
Kinematics: how a machine position removes material from a RotaryStock.

A kinematics object maps one sampled machine position (X Y Z A B C) and a
tool to a change of the stock height field, and reports contact metrics.
Turning is implemented. Rotary milling is a designed extension point.
"""

import math

import numpy as np

from .stock import RotaryAxis


class Contact:
    """What one tool position did to the stock."""

    __slots__ = ("removed", "buried", "j0", "j1", "low")

    def __init__(self, removed=0.0, buried=0.0, j0=0, j1=0, low=None):
        self.removed = removed  # largest radial depth removed by this sample at any column (mm)
        self.j0, self.j1, self.low = j0, j1, low  # columns touched and the radius the tool left
        self.buried = buried    # stock above the insert's top within its span (mm); > 0 = holder collision


class TurningKinematics:
    """
    Lathe: the workpiece spins about the stock axis, the tool moves in the
    XZ plane at Y = 0 with X as a radius. The stock axis must be parallel to
    Z; its origin offsets X and Z.

    At each tool position the stock is lowered, at every angle, to the
    lowest radius the tool outline reaches in each axial column.
    """

    name = "turning"

    def __init__(self, axis=None, collision_width=0.25):
        self.axis = axis or RotaryAxis()
        # stock above the insert must span at least this axial width (mm) to
        # count as a holder collision; thinner wafers break off
        self.collision_width = float(collision_width)
        d = self.axis.direction
        if abs(abs(d[2]) - 1.0) > 1e-9:
            raise NotImplementedError("turning needs the stock axis parallel to Z")
        self.sign = 1.0 if d[2] > 0 else -1.0

    def tool_frame(self, pos):
        """(radius, axial) of the tool control point."""
        ox, _oy, oz = self.axis.origin
        return pos[0] - ox, (pos[2] - oz) * self.sign

    def apply(self, stock, tool, pos, max_radius=math.inf):
        x, z = self.tool_frame(pos)
        if x + tool.xmin >= max_radius:
            return None  # tool entirely outside the stock's largest radius
        if self.sign > 0:
            j0, j1 = stock.column_range(z + tool.zmin, z + tool.zmax)
            if j0 >= j1:
                return None
            dz = stock.s[j0:j1] - z
        else:
            # axis reversed: the outline is mirrored along the axis
            j0, j1 = stock.column_range(z - tool.zmax, z - tool.zmin)
            if j0 >= j1:
                return None
            dz = z - stock.s[j0:j1]
        low = x + tool.lower(dz)
        current = stock.R[:, j0:j1].max(axis=0)
        removed = float(np.max(current - low, initial=0.0))
        if removed <= 0.0:
            return None
        # stock above the top of the whole insert, within its axial span: the
        # insert is deeper than its own height, so the holder would hit
        top = x + tool.xmax
        excess = np.where(current > low, current - top, 0.0)
        k = max(1, int(round(self.collision_width / stock.ds)))
        if len(excess) >= k > 1:
            excess = np.lib.stride_tricks.sliding_window_view(excess, k).min(axis=1)
        buried = float(np.max(excess, initial=0.0))
        stock.lower_all_angles(j0, j1, low)
        return Contact(removed, buried, j0, j1, low)

    def apply_at_angle(self, stock, tool, pos, i):
        """
        Lower R only at angle index i: the tool passing angle theta_i of the
        spinning workpiece during a spindle-synchronized (G33) move.
        """
        x, z = self.tool_frame(pos)
        if self.sign > 0:
            j0, j1 = stock.column_range(z + tool.zmin, z + tool.zmax)
            dz = stock.s[j0:j1] - z
        else:
            j0, j1 = stock.column_range(z - tool.zmax, z - tool.zmin)
            dz = z - stock.s[j0:j1]
        if j0 >= j1:
            return 0.0
        low = np.maximum(x + tool.lower(dz), 0.0)
        row = stock.R[i, j0:j1]
        removed = float(np.max(row - low, initial=0.0))
        if removed > 0.0:
            np.minimum(row, low, out=row)
        return removed


class RotaryMillingKinematics:
    """
    Extension point for rotary (4th-axis) milling, not implemented yet.

    Design:
    - the rotary word (A, B or C) gives the workpiece angle theta about
      `axis` (a RotaryAxis, e.g. from the Job's machine file);
    - to_workpiece() rotates the tool position (and tool axis) by -theta
      about that axis, giving the cutter in the workpiece frame;
    - apply() then lowers R(theta, s) only inside the cutter footprint at
      that angle (stock.lower_at_angles), using a RotaryStock with
      n_theta > 1 and a milling cutter profile (radius vs height).
    Indexed moves (rotary word changes alone, no cutting) and continuous
    moves (rotary and linear words together) both go through the same
    sampling, since Move.sample() interpolates the rotary words too.
    """

    name = "rotary-milling"

    AXIS_INDEX = {"A": 3, "B": 4, "C": 5}

    def __init__(self, letter, axis):
        if letter not in self.AXIS_INDEX:
            raise ValueError("rotary axis letter must be A, B or C")
        self.letter = letter
        self.index = self.AXIS_INDEX[letter]
        self.axis = axis

    def to_workpiece(self, pos, tool_axis=(0.0, 0.0, 1.0)):
        """Tool point and tool axis in the workpiece frame (rotated by -theta)."""
        theta = -math.radians(pos[self.index])
        k = np.asarray(self.axis.direction)
        o = np.asarray(self.axis.origin)

        def rot(v):
            v = np.asarray(v, dtype=float)
            return (v * math.cos(theta) + np.cross(k, v) * math.sin(theta)
                    + k * np.dot(k, v) * (1 - math.cos(theta)))

        p = rot(np.asarray(pos[:3], dtype=float) - o) + o
        return p, rot(tool_axis)

    def apply(self, stock, tool, pos, max_radius=math.inf):
        raise NotImplementedError("rotary milling removal is planned for a later phase")
