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
THREAD = "thread"  # G33 spindle-synchronized move


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
    pitch: float = 0.0        # G33 K: distance per spindle revolution along the move
    start_angle: float = 0.0  # G33 D: degrees past the spindle index at the start
    sync_retract: bool = False  # a rapid straight after a G33 (see Simulation)

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
        n = max(1, int(math.ceil(self.linear_length / step - 1e-9)), int(math.ceil(rot / angle_step - 1e-9)))
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
    retract_mode: str = "G98"   # canned cycle return level (G98: old Z, G99: R)
    cycle: dict = field(default_factory=dict)  # sticky canned cycle words and the series' old Z


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
               "G58", "G59", "G90", "G94", "G95", "G96", "G97", "G4"}
    CYCLES = ("G73", "G81", "G82", "G83")
    PECK_CLEARANCE = 0.254   # LinuxCNC's default G73/G83 peck clearance (0.010 in), unless D is given

    def __init__(self, state=None, arc_tol=0.002):
        self.state = state or MotionState()
        self.arc_tol = arc_tol
        self.diagnostics: List[Diagnostic] = []

    def _diag(self, kind, op, index, message, value=0.0):
        self.diagnostics.append(Diagnostic(kind, op, index, message, value))

    def _cycle(self, n, params, op, index):
        """
        Expand a canned drilling cycle (G81, G82, G83, G73) into moves, as
        LinuxCNC's G-code reference describes them: along the selected
        plane's normal (Z in G17); preliminary motion (rapid to R if below
        it, rapid to the cycle's position in the plane, rapid to R); then
        G81/G82 feed to the bottom (G82's dwell moves nothing), G83 pecks of
        Q, rapid out to R after each and back down to the peck clearance
        above where it reached, G73 pecks of Q backing off the clearance
        only; finally a rapid to clear Z: the position before the series
        under G98 if that is above R, else R. R is sticky, and Z and Q
        within a series.
        """
        st = self.state
        u, v, k = PLANES[st.plane]
        cyc = st.cycle
        if not cyc:
            cyc["old"] = st.pos[k]
        for w in ("R", "Q", AXES[k]):
            if w in params:
                cyc[w] = float(params[w])
        if "F" in params:
            st.feed = float(params["F"])
        r, bottom = cyc.get("R"), cyc.get(AXES[k])
        if r is None or bottom is None:
            self._diag("cycle", op, index, "%s without %s: not simulated" % (n, "R" if r is None else AXES[k]))
            return []
        if r < bottom:
            self._diag("cycle", op, index, "%s with R below the bottom (%s): not simulated" % (n, AXES[k]))
            return []
        if n in ("G73", "G83") and cyc.get("Q", 0.0) <= 0:
            self._diag("cycle", op, index, "%s needs a positive Q" % n)
            return []
        if any(st.pos[i] is None for i in LINEAR):
            self._diag("cycle", op, index, "%s before the position is known: not simulated" % n)
            return []
        clear = self.PECK_CLEARANCE if "D" not in params else float(params["D"])
        moves = []

        def go(kind, **axes):  # axes: {"<axis index>": value}
            start = tuple(st.pos)
            end = list(st.pos)
            for i, val in axes.items():
                end[int(i)] = val
            end = tuple(end)
            st.pos = list(end)
            if end != start:
                moves.append(Move(kind, start, end, op, index, None if kind == RAPID else st.feed))

        if st.pos[k] < r:
            go(RAPID, **{str(k): r})
        target = {str(i): float(params[AXES[i]]) for i in (u, v) if AXES[i] in params}
        if target:
            go(RAPID, **target)
        go(RAPID, **{str(k): r})
        if n in ("G81", "G82"):
            go(FEED, **{str(k): bottom})
        else:
            q = cyc["Q"]
            depth = r
            while True:
                depth = max(bottom, depth - q)
                go(FEED, **{str(k): depth})
                if depth <= bottom + 1e-12:
                    break
                if n == "G83":
                    go(RAPID, **{str(k): r})
                    go(RAPID, **{str(k): depth + clear})
                else:
                    go(RAPID, **{str(k): depth + clear})
        old = cyc["old"]
        go(RAPID, **{str(k): old if (st.retract_mode == "G98" and old is not None and old > r) else r})
        return moves

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
            if n in ("G98", "G99"):
                st.retract_mode = n
                continue
            if n == "G80" or (n in ("G0", "G1", "G2", "G3", "G33") and st.cycle):
                st.cycle = {}      # end of a series of canned cycles
                if n == "G80":
                    continue
            if n in self.CYCLES:
                moves.extend(self._cycle(n, params, op, index))
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
            if n in ("G0", "G1", "G2", "G3", "G33"):
                st.motion = n
            elif n.startswith("G"):
                if n == "G76":
                    self._diag("unsupported", op, index, "G76 threading cycle is not simulated yet")
                elif n in ("G81", "G82", "G83", "G73", "G84", "G85"):
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
            if start_t == end_t and n in ("G0", "G1", "G33"):
                continue
            if n == "G33":
                k = float(params.get("K", 0.0))
                if k <= 0:
                    self._diag("thread", op, index, "G33 without a positive K (distance per revolution)")
                    continue
                moves.append(Move(THREAD, start_t, end_t, op, index, None, pitch=k,
                                  start_angle=abs(float(params.get("D", 0.0)))))
                continue
            if n != "G0" and not st.feed:
                self._diag("feed", op, index, "%s with zero or missing feed rate (F%s)" % (n, st.feed))
            if n in ("G0", "G1"):
                m = Move(RAPID if n == "G0" else FEED, start_t, end_t, op, index,
                         None if n == "G0" else st.feed)
                m.sync_retract = n == "G0" and bool(moves) and moves[-1].kind == THREAD
                moves.append(m)
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
