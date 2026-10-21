# SPDX-License-Identifier: LGPL-2.1-or-later
"""
Interpretation of FreeCAD Path commands into motion segments.

Input is a sequence of (name, params) pairs, which is what Path.Command
exposes as cmd.Name and cmd.Parameters. No FreeCAD import is needed here.

Conventions (FreeCAD Path / RS274NGC):
- Coordinates are absolute, in mm and degrees.
- Arc centre offsets I, J, K are relative to the arc start point.
- Plane selection: G17 = XY (normal +Z), G18 = ZX (normal +Y),
  G19 = YZ (normal +X). G2 is clockwise and G3 counter-clockwise, seen
  from the positive end of the plane normal looking toward the origin.
  For G18 that is the view with Z to the right and X up.
"""

import math
from dataclasses import dataclass, field
from typing import List, Optional, Sequence, Tuple

import numpy as np

AXES = ("X", "Y", "Z", "A", "B", "C")
LINEAR = (0, 1, 2)
ROTARY = (3, 4, 5)

# (u, v, normal) as indices into (X, Y, Z); (u, v, normal) is right-handed
PLANES = {
    "G17": (0, 1, 2),
    "G18": (2, 0, 1),
    "G19": (1, 2, 0),
}
OFFSET_WORD = {0: "I", 1: "J", 2: "K"}

RAPID = "rapid"
FEED = "feed"
ARC = "arc"


def normalise_name(name):
    """'G01' -> 'G1', 'g0' -> 'G0'. Comments and blanks return ''."""
    n = (name or "").strip().upper()
    if not n or n.startswith("(") or n.startswith(";"):
        return ""
    letter, num = n[0], n[1:]
    try:
        f = float(num)
    except ValueError:
        return n
    if f == int(f):
        return "%s%d" % (letter, int(f))
    return "%s%g" % (letter, f)


@dataclass
class Move:
    """One motion segment in machine coordinates (X Y Z A B C)."""

    kind: str
    start: Tuple[float, ...]
    end: Tuple[float, ...]
    op: int = 0
    index: int = 0
    feed: Optional[float] = None
    plane: Optional[Tuple[int, int, int]] = None
    centre: Optional[Tuple[float, float, float]] = None
    ccw: bool = False
    sweep: float = 0.0  # signed arc sweep in radians (positive = CCW)
    radius: float = 0.0

    @property
    def linear_length(self):
        if self.kind == ARC:
            u, v, _ = self.plane
            planar = abs(self.sweep) * self.radius
            axial = self.end[self.plane[2]] - self.start[self.plane[2]]
            return math.hypot(planar, axial)
        return math.dist(self.start[:3], self.end[:3])

    def sample(self, step, angle_step=1.0):
        """
        Positions along the move at spacing <= step (mm) and rotary
        increments <= angle_step (degrees). Returns an (n, 6) array that
        excludes the start point and includes the end point, n >= 1.
        """
        start = np.asarray(self.start, dtype=float)
        end = np.asarray(self.end, dtype=float)
        rot = np.abs(end[3:] - start[3:]).max()
        n = max(1, int(math.ceil(self.linear_length / step)), int(math.ceil(rot / angle_step)))
        t = np.arange(1, n + 1, dtype=float) / n
        pts = start[None, :] + t[:, None] * (end - start)[None, :]
        if self.kind == ARC:
            u, v, _ = self.plane
            a0 = math.atan2(start[v] - self.centre[v], start[u] - self.centre[u])
            ang = a0 + t * self.sweep
            pts[:, u] = self.centre[u] + self.radius * np.cos(ang)
            pts[:, v] = self.centre[v] + self.radius * np.sin(ang)
            pts[-1, :3] = end[:3]  # land exactly on the programmed end point
        return pts


@dataclass
class Diagnostic:
    kind: str
    op: int
    index: int
    message: str
    value: float = 0.0


@dataclass
class MotionState:
    """Modal state carried from command to command and from op to op."""

    pos: List[Optional[float]] = field(default_factory=lambda: [None, 0.0, None, 0.0, 0.0, 0.0])
    plane: str = "G17"
    motion: Optional[str] = None
    feed: Optional[float] = None


def arc_geometry(start, end, params, plane, ccw):
    """
    Centre, radius and signed sweep for an arc. params holds I/J/K (relative
    to start) or R. Returns (centre_xyz, radius, sweep, radius_error).
    """
    u, v, _n = plane
    centre = list(start[:3])
    if "R" in params and not any(OFFSET_WORD[a] in params for a in (u, v)):
        r = float(params["R"])
        du, dv = end[u] - start[u], end[v] - start[v]
        chord = math.hypot(du, dv)
        if chord < 1e-12 or abs(r) < chord / 2 - 1e-9:
            raise ValueError("R arc with impossible radius")
        h = math.sqrt(max(r * r - chord * chord / 4, 0.0))
        # centre is left of the chord for a CCW arc shorter than 180 deg (R > 0)
        side = 1.0 if ccw else -1.0
        if r < 0:
            side = -side
        mu, mv = start[u] + du / 2, start[v] + dv / 2
        centre[u] = mu - side * h * dv / chord
        centre[v] = mv + side * h * du / chord
    else:
        centre[u] = start[u] + float(params.get(OFFSET_WORD[u], 0.0))
        centre[v] = start[v] + float(params.get(OFFSET_WORD[v], 0.0))
    r0 = math.hypot(start[u] - centre[u], start[v] - centre[v])
    r1 = math.hypot(end[u] - centre[u], end[v] - centre[v])
    a0 = math.atan2(start[v] - centre[v], start[u] - centre[u])
    a1 = math.atan2(end[v] - centre[v], end[u] - centre[u])
    full = math.hypot(end[u] - start[u], end[v] - start[v]) < 1e-9
    if ccw:
        sweep = (a1 - a0) % (2 * math.pi)
        if full or sweep < 1e-12:
            sweep = 2 * math.pi
    else:
        sweep = -((a0 - a1) % (2 * math.pi))
        if full or sweep > -1e-12:
            sweep = -2 * math.pi
    return tuple(centre), r0, sweep, abs(r1 - r0)


class MotionInterpreter:
    """Turns Path commands into Move objects, carrying modal state across ops."""

    IGNORED = {"G20", "G21", "G40", "G41", "G42", "G43", "G49", "G54", "G55", "G56", "G57",
               "G58", "G59", "G90", "G94", "G95", "G96", "G97", "G98", "G99", "G80"}

    def __init__(self, state=None, arc_tol=0.002):
        self.state = state or MotionState()
        self.arc_tol = arc_tol
        self.diagnostics: List[Diagnostic] = []

    def _diag(self, kind, op, index, message, value=0.0):
        self.diagnostics.append(Diagnostic(kind, op, index, message, value))

    def run(self, commands: Sequence, op: int = 0) -> List[Move]:
        moves = []
        st = self.state
        for index, (name, params) in enumerate(commands):
            params = params or {}
            n = normalise_name(name)
            if not n:
                continue
            if n in PLANES:
                st.plane = n
                continue
            if n in ("G7", "G8"):
                self._diag("unsupported", op, index,
                           "%s (diameter/radius mode) is not supported; X is treated as a radius" % n)
                continue
            if n == "G91":
                self._diag("unsupported", op, index, "G91 incremental mode is not supported")
                continue
            if n in self.IGNORED or n[0] in "MTSFN":
                continue
            if n in ("G0", "G1", "G2", "G3"):
                st.motion = n
            elif n.startswith("G") and n not in ("G0", "G1", "G2", "G3"):
                if n in ("G81", "G82", "G83", "G73", "G84", "G85"):
                    self._diag("unsupported", op, index, "canned cycle %s is not simulated" % n)
                else:
                    self._diag("unsupported", op, index, "command %s is ignored" % n)
                continue
            if "F" in params:
                st.feed = float(params["F"])

            start = list(st.pos)
            end = list(st.pos)
            for i, a in enumerate(AXES):
                if a in params:
                    end[i] = float(params[a])
            st.pos = end
            if any(end[i] is None for i in LINEAR):
                continue  # position not yet known (e.g. the first X-only move of a job)
            for i in range(6):
                if start[i] is None:
                    start[i] = end[i]
            start_t, end_t = tuple(start), tuple(end)
            if start_t == end_t and n in ("G0", "G1"):
                continue
            if n != "G0" and not st.feed:
                self._diag("feed", op, index, "%s with zero or missing feed rate (F%s)" % (n, st.feed))
            if n in ("G0", "G1"):
                moves.append(Move(RAPID if n == "G0" else FEED, start_t, end_t, op, index,
                                  None if n == "G0" else st.feed))
                continue
            plane = PLANES[st.plane]
            ccw = n == "G3"
            try:
                centre, radius, sweep, err = arc_geometry(start_t, end_t, params, plane, ccw)
            except ValueError as e:
                self._diag("arc", op, index, str(e))
                moves.append(Move(FEED, start_t, end_t, op, index, st.feed))
                continue
            if err > self.arc_tol:
                self._diag("arc", op, index,
                           "%s start and end radius differ by %.4f mm" % (n, err), err)
            moves.append(Move(ARC, start_t, end_t, op, index, st.feed, plane, centre, ccw, sweep, radius))
        return moves


def parse_gcode_text(text):
    """
    Minimal parser for FreeCAD-style .nc text ('G1 F5.0 X1.0 Z2.0' per line),
    as stored inside .FCStd files. Used by tests and dev tools; the addon
    itself reads Path.Command objects.
    """
    cmds = []
    for line in text.splitlines():
        line = line.split(";")[0].strip()
        if not line or line.startswith("("):
            continue
        words = line.split()
        name, params = words[0], {}
        for w in words[1:]:
            if w.startswith("("):
                break
            params[w[0].upper()] = float(w[1:])
        cmds.append((name, params))
    return cmds
