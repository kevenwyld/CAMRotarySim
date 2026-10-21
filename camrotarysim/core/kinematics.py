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

    __slots__ = ("removed", "buried", "j0", "j1", "low", "high", "excess")

    def __init__(self, removed=0.0, buried=0.0, j0=0, j1=0, low=None, high=None, excess=None):
        self.removed = removed  # largest radial depth removed by this sample at any column (mm)
        # columns touched and the radius the tool left: low (outside) or high (bore)
        self.j0, self.j1, self.low, self.high = j0, j1, low, high
        self.excess = excess  # per column j0..j1: material beyond the insert's back before the cut (mm)
        self.buried = buried    # stock beyond the insert's back within its span (mm); > 0 = holder/bar collision


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
        # stock beyond the insert's back must span at least this axial width
        # (mm), over a move, to count as a holder or bar collision; thinner
        # wafers break off (applied by Simulation)
        self.collision_width = float(collision_width)
        d = self.axis.direction
        if abs(abs(d[2]) - 1.0) > 1e-9:
            raise NotImplementedError("turning needs the stock axis parallel to Z")
        self.sign = 1.0 if d[2] > 0 else -1.0

    def tool_frame(self, pos):
        """(radius, axial) of the tool control point."""
        ox, _oy, oz = self.axis.origin
        return pos[0] - ox, (pos[2] - oz) * self.sign

    def _columns(self, stock, tool, z):
        if self.sign > 0:
            j0, j1 = stock.column_range(z + tool.zmin, z + tool.zmax)
            return j0, j1, stock.s[j0:j1] - z
        # axis reversed: the outline is mirrored along the axis
        j0, j1 = stock.column_range(z - tool.zmax, z - tool.zmin)
        return j0, j1, z - stock.s[j0:j1]

    def apply(self, stock, tool, pos, max_radius=math.inf):
        """
        Cut at one tool position. Contact.excess holds, per column touched,
        how far material stood beyond the insert's back (above its top, or
        inside its bottom for an internal tool) before the cut: where the
        holder or bar would be. The simulation collects it over a move and
        applies collision_width to the columns collected, so material a move
        ploughs through counts over its whole length, while a thin wafer left
        by an earlier pass doesn't.
        """
        if getattr(tool, "internal", False):
            return self._apply_internal(stock, tool, pos)
        x, z = self.tool_frame(pos)
        if x + tool.xmin >= max_radius:
            return None  # tool entirely outside the stock's largest radius
        j0, j1, dz = self._columns(stock, tool, z)
        if j0 >= j1:
            return None
        low = x + tool.lower(dz)
        current = stock.R[:, j0:j1].max(axis=0)
        bore = stock.Rin[:, j0:j1].min(axis=0)
        # material only between the bore and the outside
        removed = float(np.max(current - np.maximum(low, bore), initial=0.0))
        if removed <= 0.0:
            return None
        # stock above the top of the whole insert, within its axial span: the
        # insert is deeper than its own height, so the holder would hit
        top = x + tool.xmax
        excess = np.where((current > low) & (current > bore), current - np.maximum(top, bore), 0.0)
        stock.lower_all_angles(j0, j1, low)
        return Contact(removed, float(np.max(excess, initial=0.0)), j0, j1, low, excess=excess)

    def _apply_internal(self, stock, tool, pos):
        """
        A tool inside the stock (boring): it widens the bore, at every angle,
        to the highest radius its outline reaches in each column.
        """
        x, z = self.tool_frame(pos)
        j0, j1, dz = self._columns(stock, tool, z)
        if j0 >= j1:
            return None
        high = x + tool.upper(dz)
        outer = stock.R[:, j0:j1].max(axis=0)
        current = stock.Rin[:, j0:j1].min(axis=0)
        removed = float(np.max(np.minimum(high, outer) - current, initial=0.0))
        if removed <= 0.0:
            return None
        # stock inside the bottom of the whole insert, within its span: the
        # insert is deeper than its own height, so the bar would hit
        bottom = x + tool.xmin
        excess = np.where((current < high) & (outer > current), np.minimum(bottom, outer) - current, 0.0)
        stock.raise_bore_all_angles(j0, j1, high)
        return Contact(removed, float(np.max(excess, initial=0.0)), j0, j1, None, high, excess=excess)

    def apply_at_angle(self, stock, tool, pos, i):
        """
        Cut at angle index i only: the tool passing angle theta_i of the
        spinning workpiece during a spindle-synchronized (G33) move.
        """
        x, z = self.tool_frame(pos)
        j0, j1, dz = self._columns(stock, tool, z)
        if j0 >= j1:
            return 0.0
        if getattr(tool, "internal", False):
            high = x + tool.upper(dz)
            row = stock.Rin[i, j0:j1]
            removed = float(np.max(np.minimum(high, stock.R[i, j0:j1]) - row, initial=0.0))
            if removed > 0.0:
                np.maximum(row, high, out=row)
            return removed
        low = np.maximum(x + tool.lower(dz), 0.0)
        row = stock.R[i, j0:j1]
        removed = float(np.max(row - np.maximum(low, stock.Rin[i, j0:j1]), initial=0.0))
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
