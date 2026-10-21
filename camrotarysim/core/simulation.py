# SPDX-License-Identifier: LGPL-2.1-or-later
"""
Simulation engine: sequences the ops, samples every move, applies the
kinematics to the stock and records events. Supports playback in steps,
seeking (backwards too, via per-move snapshots) and reset.
"""

from dataclasses import dataclass
from typing import List, Sequence

import numpy as np

from .kinematics import TurningKinematics
from .motion import RAPID, Diagnostic, MotionInterpreter, MotionState


@dataclass
class OpInput:
    """One operation to simulate: its commands and its tool."""

    name: str
    commands: Sequence  # (name, params) pairs
    tool: object        # ToolOutline for turning
    kinematics: object = None


@dataclass
class Event:
    kind: str           # 'rapid_cut' | 'holder_collision'
    op: int
    op_name: str
    index: int          # command index within the op
    position: tuple
    value: float        # mm
    message: str


class Simulation:
    def __init__(self, stock, ops: List[OpInput], step=0.05, angle_step=1.0,
                 rapid_tol=0.001, collision_tol=0.001, kinematics=None, parting_min_radius=1.0):
        self.stock = stock
        self.ops = ops
        self.step = float(step)
        self.angle_step = float(angle_step)
        self.rapid_tol = rapid_tol
        self.collision_tol = collision_tol
        self.parting_min_radius = parting_min_radius
        self.default_kinematics = kinematics or TurningKinematics(stock.axis)

        interp = MotionInterpreter(MotionState())
        self.moves = []
        for i, op in enumerate(ops):
            self.moves.extend(interp.run(op.commands, i))
        self.diagnostics: List[Diagnostic] = interp.diagnostics

        self._samples = [m.sample(self.step, self.angle_step) for m in self.moves]
        counts = np.array([len(s) for s in self._samples], dtype=np.int64)
        self._offsets = np.concatenate([[0], np.cumsum(counts)])
        self.total_samples = int(self._offsets[-1])
        self._max_radius = float(stock.R.max())
        self._initial = stock.snapshot()
        self.reset()

    # -------------------------------------------------------------- state
    def reset(self):
        self.stock.restore(self._initial)
        self.cursor = 0              # samples processed so far
        self._move_snaps = {}        # move index -> (stock snapshot, events count) at move end
        self.events: List[Event] = []
        self._move_events = {}
        self.position = self.moves[0].start if self.moves else None

    @property
    def done(self):
        return self.cursor >= self.total_samples

    @property
    def progress(self):
        return self.cursor / self.total_samples if self.total_samples else 1.0

    def move_at(self, sample_index):
        """Index of the move containing global sample `sample_index` (0-based)."""
        return int(np.searchsorted(self._offsets, sample_index, "right") - 1)

    @property
    def current_move(self):
        if not self.moves:
            return None
        i = min(self.move_at(max(self.cursor - 1, 0)), len(self.moves) - 1)
        return self.moves[i]

    # ----------------------------------------------------------- running
    def _kinematics(self, op_index):
        k = self.ops[op_index].kinematics
        return k or self.default_kinematics

    def advance(self, n):
        """Process up to n more samples. Returns the number processed."""
        if self.done or n <= 0:
            return 0
        target = min(self.total_samples, self.cursor + int(n))
        processed = 0
        while self.cursor < target:
            mi = self.move_at(self.cursor)
            move = self.moves[mi]
            first = self.cursor - int(self._offsets[mi])
            last = min(len(self._samples[mi]), first + (target - self.cursor))
            self._run_samples(mi, move, first, last)
            processed += last - first
            self.cursor += last - first
            if last == len(self._samples[mi]):
                self._finish_move(mi)
        return processed

    def advance_to_move_end(self):
        """Single step: finish the current (or next) move."""
        if self.done:
            return 0
        mi = self.move_at(self.cursor)
        return self.advance(int(self._offsets[mi + 1]) - self.cursor)

    def run_to_end(self):
        return self.advance(self.total_samples - self.cursor)

    def seek(self, sample_index):
        """Jump to a global sample index, forwards or backwards."""
        sample_index = max(0, min(int(sample_index), self.total_samples))
        if sample_index < self.cursor:
            mi = self.move_at(sample_index) - 1  # last fully finished move before target
            while mi >= 0 and mi not in self._move_snaps:
                mi -= 1
            if mi >= 0:
                snap, n_events = self._move_snaps[mi]
                self.stock.restore(snap)
                self.events = self.events[:n_events]
                self.cursor = int(self._offsets[mi + 1])
                self._move_snaps = {k: v for k, v in self._move_snaps.items() if k <= mi}
                self._move_events = {}
                self.position = self.moves[mi].end
            else:
                self.reset()
        self.advance(sample_index - self.cursor)

    def _run_samples(self, mi, move, first, last):
        op = self.ops[move.op]
        kin = self._kinematics(move.op)
        pts = self._samples[mi][first:last]
        if first == 0:
            self._move_start = self.stock.profile() if move.kind == RAPID else None
        start_prof = getattr(self, "_move_start", None)
        worst_removed = 0.0
        worst_buried = 0.0
        where_r = where_b = None
        for p in pts:
            c = kin.apply(self.stock, op.tool, p, self._max_radius)
            if c is None:
                continue
            # rapids: depth into the stock as it was when the move began
            depth = c.removed
            if start_prof is not None:
                depth = float(np.max(start_prof[c.j0:c.j1] - c.low, initial=0.0))
            if depth > worst_removed:
                worst_removed, where_r = depth, p
            if c.buried > worst_buried:
                worst_buried, where_b = c.buried, p
        if len(pts):
            self.position = tuple(pts[-1])
        acc = self._move_events.setdefault(mi, {"removed": 0.0, "buried": 0.0, "pr": None, "pb": None})
        if worst_removed > acc["removed"]:
            acc["removed"], acc["pr"] = worst_removed, where_r
        if worst_buried > acc["buried"]:
            acc["buried"], acc["pb"] = worst_buried, where_b

    def _finish_move(self, mi):
        move = self.moves[mi]
        acc = self._move_events.pop(mi, None)
        if acc:
            name = self.ops[move.op].name
            if move.kind == RAPID and acc["removed"] > self.rapid_tol:
                self.events.append(Event(
                    "rapid_cut", move.op, name, move.index, tuple(acc["pr"]), acc["removed"],
                    "G0 cuts %.3f mm into stock" % acc["removed"]))
            if acc["buried"] > self.collision_tol:
                self.events.append(Event(
                    "holder_collision", move.op, name, move.index, tuple(acc["pb"]), acc["buried"],
                    "stock %.3f mm above the insert's top: holder would collide" % acc["buried"]))
        if not any(e.kind == "parted" for e in self.events):
            prof = self.stock.profile()
            gone = prof <= 1e-3
            if gone.any():
                solid = np.nonzero(~gone)[0]
                inner = np.nonzero(gone)[0]
                inner = inner[(inner > solid.min()) & (inner < solid.max())] if len(solid) else inner[:0]
                # both pieces must be substantial (facing to X0 detaches a tiny centre pip)
                inner = [j for j in inner
                         if prof[:j].max() > self.parting_min_radius and prof[j:].max() > self.parting_min_radius]
                if inner:
                    j = int(inner[0])
                    self.events.append(Event(
                        "parted", move.op, self.ops[move.op].name, move.index, tuple(move.end),
                        float(self.stock.s[j]),
                        "stock parted at Z %.3f (material on both sides)" % self.stock.s[j]))
        self._move_snaps[mi] = (self.stock.snapshot(), len(self.events))

    # ------------------------------------------------------------ helpers
    def path_polylines(self, step=None):
        """Per move: (kind, op, points (n, 3)) for path display."""
        step = step or max(self.step * 4, 0.2)
        out = []
        for m in self.moves:
            pts = np.vstack([np.asarray(m.start)[None, :], m.sample(step, 5.0)])[:, :3]
            out.append((m.kind, m.op, pts))
        return out
