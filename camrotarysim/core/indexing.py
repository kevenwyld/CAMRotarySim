# SPDX-License-Identifier: LGPL-2.1-or-later
"""
Indexing moves between milling ops on different work planes (retract, rotary turn),
as the posted program runs them; the tool position is carried through the machine frame.
"""

import numpy as np

from .milling import MillingKinematics, rotation_matrix
from .motion import LINEAR, MotionInterpreter, MotionState
from .simulation import OpInput


def _to_machine(kin, angle, local):
    p = kin.R @ np.asarray(local, dtype=float) + kin.t
    o = np.asarray(kin.axis.origin)
    return rotation_matrix(kin.axis.direction, angle) @ (p - o) + o


def _to_local(kin, angle, machine):
    o = np.asarray(kin.axis.origin)
    p = rotation_matrix(kin.axis.direction, -angle) @ (np.asarray(machine, dtype=float) - o) + o
    return kin.R.T @ (p - kin.t)


def first_position(commands):
    """The first X, Y and Z words of an op's commands (each may be None)."""
    found = [None, None, None]
    for _name, params in commands:
        for i, a in enumerate("XYZ"):
            if found[i] is None and a in (params or {}):
                found[i] = float(params[a])
        if all(v is not None for v in found):
            break
    return found


def swing_radius(stock, axis):
    """Largest distance of the stock's material from the rotary axis (on the safe side: faces included)."""
    idx = np.nonzero(stock.phi0 > -stock.h)
    if not len(idx[0]):
        return 0.0
    pts = np.stack([stock.axes[k][idx[k]] for k in range(3)], axis=1)
    k = np.asarray(axis.direction)
    v = pts - np.asarray(axis.origin)
    return float(np.max(np.linalg.norm(v - np.outer(v @ k, k), axis=1)))


def with_indexing(ops, retract_z=None, start_angle=0.0):
    """The milling OpInputs with indexing ops inserted where meta["angle"] changes, and each
    op's start position set. retract_z: machine Z before turning, or None to turn in place."""
    out = []
    angle = float(start_angle)
    machine = None                     # tool tip in the machine frame, once known
    for op in ops:
        kin = op.kinematics
        target = float((op.meta or {}).get("angle", 0.0))
        if abs(target - angle) > 1e-9:
            if machine is None:
                # program start: put the tool over the op's first position, clear
                first = first_position(op.commands)
                if retract_z is not None and first[0] is not None and first[1] is not None:
                    local = [first[0], first[1], first[2] if first[2] is not None else 0.0]
                    machine = _to_machine(kin, target, local)
                    machine[2] = retract_z
            cmds = []
            if machine is not None and retract_z is not None and machine[2] < retract_z:
                cmds.append(("G0", {"Z": retract_z}))
                machine = machine.copy()
                machine[2] = retract_z
            cmds.append(("G0", {kin.letter: target}))
            if machine is not None:
                start = list(machine) + [0.0, 0.0, 0.0]
                start[kin.index] = angle
                name = "%s (index %s %+.1f to %+.1f%s)" % (
                    op.name, kin.letter, angle, target, ", retracted" if retract_z is not None else "")
                out.append(OpInput(name, cmds, op.tool, kinematics=MillingKinematics(kin.axis, None, kin.letter),
                                   meta={"angle": 0.0, "letter": kin.letter, "indexing": True}, start=start))
            angle = target
        if machine is not None:
            op.start = list(_to_local(kin, angle, machine)) + [0.0, 0.0, 0.0]
        out.append(op)
        # where the op leaves the tool
        st = MotionState()
        if op.start is not None:
            st.pos = list(op.start)
        else:
            st.pos = [None, None, None, 0.0, 0.0, 0.0]
        moves = MotionInterpreter(st).run(op.commands)
        if moves:
            machine = _to_machine(kin, angle, moves[-1].end[:3])
        elif all(st.pos[i] is not None for i in LINEAR):
            machine = _to_machine(kin, angle, st.pos[:3])
    return out


def describe_retract(pre_rotary, swing, clearance):
    """(retract machine Z or None, note) from the machine's pre-rotary move; a G53 move
    can't be placed without the work offset, so it becomes swing radius + clearance."""
    text = (pre_rotary or "").strip()
    if not text:
        return None, "no pre-rotary move in the machine: the rotary axis turns with the tool where it is"
    z = None
    for line in text.splitlines():
        words = line.upper().replace("G53", " G53 ").split()
        for w in words:
            if w.startswith("Z"):
                try:
                    z = float(w[1:])
                except ValueError:
                    pass
    if "G53" in text.upper() or z is None:
        zc = swing + clearance
        return zc, ("pre-rotary move '%s' is in machine coordinates: shown as a retract to Z %.3f "
                    "(the stock's swing radius %.3f plus %.3f clearance), as the work offset isn't known here"
                    % (text, zc, swing, clearance))
    return z, "pre-rotary move '%s': retract to Z %.3f" % (text, z)


def rotary_moves(sim):
    """The simulation's moves that turn a rotary axis."""
    return [m for m in sim.moves if any(abs(m.end[i] - m.start[i]) > 1e-9 for i in (3, 4, 5))]
