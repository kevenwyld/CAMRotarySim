# SPDX-License-Identifier: LGPL-2.1-or-later
"""
Simulation engine: sequences the ops, samples every move, applies the
kinematics to the stock and records events. Supports playback in steps,
seeking (backwards too, via per-move snapshots) and reset.
"""

from dataclasses import dataclass, replace
from typing import List, Sequence

import math

import numpy as np

from .kinematics import TurningKinematics
from .motion import RAPID, THREAD, Diagnostic, MotionInterpreter, MotionState


@dataclass
class OpInput:
    """One operation to simulate: its commands and its tool."""

    name: str
    commands: Sequence  # (name, params) pairs
    tool: object        # ToolOutline for turning
    kinematics: object = None
    # spindle sense for G33: +1 = the workpiece turns counter-clockwise seen
    # from +Z (a right-hand thread when the tool feeds toward -Z), -1 = clockwise
    spindle: int = 1
    meta: dict = None   # op data for reports (e.g. thread properties)
    # an internal op (boring, internal thread): the tool is used inside, its
    # outline mirrored about the control point (see ToolOutline.mirrored)
    internal: bool = False
    # where the tool is when the op begins (X Y Z A B C in the op's own frame), when known:
    # carried between ops in different frames (see indexing.with_indexing)
    start: list = None


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
                 rapid_tol=0.001, collision_tol=0.001, kinematics=None, parting_min_radius=1.0,
                 thread_angles=120, snapshot_budget=96 * 2 ** 20):
        self.stock = stock
        # internal ops get a mirrored tool on a copy: the caller's OpInputs are left alone
        self.ops = [replace(op, tool=op.tool.mirrored())
                    if op.internal and not getattr(op.tool, "internal", False) else op for op in ops]
        self.step = float(step)
        self.angle_step = float(angle_step)
        self.rapid_tol = rapid_tol
        self.collision_tol = collision_tol
        self.parting_min_radius = parting_min_radius
        # turning by default; milling ops bring their own (MillingKinematics, with their frame)
        self.turning = hasattr(stock, "R")
        if kinematics is None and self.turning:
            kinematics = TurningKinematics(stock.axis)
        self.default_kinematics = kinematics

        interp = MotionInterpreter(MotionState())
        self.moves = []
        for i, op in enumerate(ops):
            if op.start is not None:
                interp.state.pos = [float(v) for v in op.start]
            self.moves.extend(interp.run(op.commands, i))
        self.diagnostics: List[Diagnostic] = interp.diagnostics

        self.thread_angles = int(thread_angles)
        self.snapshot_budget = int(snapshot_budget)
        self._samples = [m.sample(self.step, self.angle_step) for m in self.moves]
        self._thread_events = {}
        self._alpha_after = []
        alpha = 0.0
        for mi, m in enumerate(self.moves):
            if m.kind == THREAD:
                alpha = self._spindle_angle(m, m.linear_length)
            self._alpha_after.append(alpha)
        counts = np.array([len(s) for s in self._samples], dtype=np.int64)
        self._offsets = np.concatenate([[0], np.cumsum(counts)])
        self.total_samples = int(self._offsets[-1])
        self._max_radius = float(stock.R.max()) if self.turning else math.inf
        self._initial = stock.snapshot()
        self.reset()

    # -------------------------------------------------------------- state
    def reset(self):
        self.stock.restore(self._initial)
        self.cursor = 0              # samples processed so far
        self._move_snaps = {}        # move index -> (stock snapshot, events count) at move end
        self._snap_stride = 1
        self._snap_bytes = 0
        self.events: List[Event] = []
        self._move_events = {}
        self._move_start = None      # stock profiles when the current move began, and its index
        self._move_start_index = None
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
                self._snap_bytes = sum(v[0].nbytes for v in self._move_snaps.values())
                self._move_events = {}
                self.position = self.moves[mi].end
            else:
                self.reset()
        self.advance(sample_index - self.cursor)

    # ------------------------------------------------------------ threading
    def _spindle_angle(self, move, d):
        """Workpiece angle (rad) a distance d along a G33 move: the move starts on the index."""
        sense = self.ops[move.op].spindle or 1
        return sense * (math.radians(move.start_angle) + 2 * math.pi * d / move.pitch)

    @property
    def spindle_angle(self):
        """Current workpiece angle about the axis (rad), for display."""
        if not self.moves or self.cursor == 0:
            return 0.0
        mi = min(self.move_at(self.cursor - 1), len(self.moves) - 1)
        m = self.moves[mi]
        done = self.cursor - int(self._offsets[mi])
        if m.kind == THREAD and done < len(self._samples[mi]):
            return self._spindle_angle(m, m.linear_length * done / len(self._samples[mi]))
        return self._alpha_after[mi]

    def _events(self, mi, move):
        """
        Where the tool passes each workpiece angle during a G33 move:
        (distances along the move, angle indices), sorted by distance.
        The tool sits at world angle 0, so it is at workpiece angle -alpha.
        """
        ev = self._thread_events.get(mi)
        if ev is not None:
            return ev
        n = self.stock.n_theta
        sense = self.ops[move.op].spindle or 1
        L, K = move.linear_length, move.pitch
        theta = np.arange(n) / n  # in turns
        d0 = K * np.mod(-sense * theta - move.start_angle / 360.0, 1.0)
        reps = int(math.floor(L / K)) + 1
        d = (d0[None, :] + K * np.arange(reps)[:, None]).ravel()
        idx = np.tile(np.arange(n), reps)
        keep = d <= L + 1e-12
        d, idx = d[keep], idx[keep]
        order = np.argsort(d, kind="stable")
        ev = (d[order], idx[order])
        self._thread_events[mi] = ev
        return ev

    def _run_thread(self, mi, move, first, last):
        if self.stock.n_theta == 1:
            self.stock.set_angles(self.thread_angles)
            self._thread_events.clear()
        op = self.ops[move.op]
        kin = self._kinematics(move.op)
        d, idx = self._events(mi, move)
        n = len(self._samples[mi])
        L = move.linear_length
        lo = L * first / n if first else -1.0
        hi = L * last / n
        a, b = np.searchsorted(d, lo, "right"), np.searchsorted(d, hi, "right")
        start = np.asarray(move.start, dtype=float)
        delta = np.asarray(move.end, dtype=float) - start
        for k in range(a, b):
            pos = start + delta * (d[k] / L)
            kin.apply_at_angle(self.stock, op.tool, pos, int(idx[k]))
        self.position = tuple(self._samples[mi][last - 1])

    def thread_moves(self):
        return [m for m in self.moves if m.kind == THREAD]

    # ------------------------------------------------------------ samples
    def _run_samples(self, mi, move, first, last):
        if move.kind == THREAD:
            self._run_thread(mi, move, first, last)
            return
        if move.sync_retract:
            # the rapid out after a G33 takes a fraction of a turn; treated
            # as instant, it cuts nothing (as a turning move it would cut a
            # full ring at the end of the thread on every pass)
            self.position = tuple(self._samples[mi][last - 1])
            return
        op = self.ops[move.op]
        kin = self._kinematics(move.op)
        pts = self._samples[mi][first:last]
        if first == 0 or self._move_start_index != mi:
            # the stock when the move began: a rapid's cut is measured against it
            self._move_start = ((self.stock.profile(), self.stock.inner_profile())
                                if self.turning else None)
            self._move_start_index = mi
        start_prof = self._move_start if move.kind == RAPID and self.turning else None
        worst_removed = 0.0
        where_r = None
        excess = where = None
        swept = 0.0
        for p in self._candidates(kin, op.tool, pts):
            c = kin.apply(self.stock, op.tool, p, self._max_radius)
            if c is None:
                continue
            # rapids: depth into the stock as it was when the move began
            depth = c.removed
            if start_prof is not None:
                outer, inner = start_prof[0][c.j0:c.j1], start_prof[1][c.j0:c.j1]
                if c.high is not None:   # internal tool: how far it went past the bore, into material
                    depth = float(np.max(np.minimum(c.high, outer) - inner, initial=0.0))
                else:
                    depth = float(np.max(outer - np.maximum(c.low, inner), initial=0.0))
            if depth > worst_removed:
                worst_removed, where_r = depth, p
            if c.volume is not None:
                swept += c.volume
            if c.excess is not None and len(c.excess):
                if excess is None:
                    excess = np.zeros(self.stock.n_s)
                    where = [None] * self.stock.n_s
                seg = excess[c.j0:c.j1]
                better = c.excess > seg
                if better.any():
                    np.maximum(seg, c.excess, out=seg)
                    for j in np.nonzero(better)[0]:
                        where[c.j0 + j] = p
        if len(pts):
            self.position = tuple(pts[-1])
        acc = self._move_events.setdefault(mi, {"removed": 0.0, "pr": None, "excess": None, "where": None,
                                                "volume": 0.0})
        acc["volume"] += swept
        if worst_removed > acc["removed"]:
            acc["removed"], acc["pr"] = worst_removed, where_r
        if excess is not None:
            if acc["excess"] is None:
                acc["excess"], acc["where"] = excess, where
            else:
                better = excess > acc["excess"]
                acc["excess"][better] = excess[better]
                for j in np.nonzero(better)[0]:
                    acc["where"][j] = where[j]

    CHUNK = 32

    def _candidates(self, kin, tool, pts):
        """
        The samples that may touch material: chunks of samples are dropped
        when the tool can't reach the stock anywhere in the columns the chunk
        spans (e.g. rapids in and out of a drilled hole), checked in one go.
        """
        may_touch = getattr(kin, "may_touch", None)
        if may_touch is not None:      # milling: a box test per chunk
            for a in range(0, len(pts), self.CHUNK):
                chunk = pts[a:a + self.CHUNK]
                if may_touch(self.stock, tool, chunk):
                    yield from chunk
            return
        tool_frame = getattr(kin, "tool_frame", None)
        if tool_frame is None or len(pts) <= 2:
            yield from pts
            return
        st = self.stock
        internal = getattr(tool, "internal", False)
        sign = getattr(kin, "sign", 1.0)
        for a in range(0, len(pts), self.CHUNK):
            chunk = pts[a:a + self.CHUNK]
            x = chunk[:, 0] - kin.axis.origin[0]
            z = (chunk[:, 2] - kin.axis.origin[2]) * sign
            if sign > 0:
                lo, hi = z.min() + tool.zmin, z.max() + tool.zmax
            else:
                lo, hi = z.min() - tool.zmax, z.max() - tool.zmin
            j0, j1 = st.column_range(lo, hi)
            if j0 >= j1:
                continue
            wall = (st.R[:, j0:j1] - st.Rin[:, j0:j1]) > 0
            if not wall.any():
                continue
            if internal:
                if x.max() + tool.xmax <= st.Rin[:, j0:j1][wall].min():
                    continue
            elif x.min() + tool.xmin >= st.R[:, j0:j1][wall].max():
                continue
            yield from chunk

    def _collision(self, acc):
        """
        Worst material beyond the insert's back over a move, counted only on
        runs of columns at least collision_width long (the move's columns
        collected together): (depth, position) or None.
        """
        if acc.get("excess") is None:
            return None
        ex = acc["excess"]
        kin = self.default_kinematics
        k = max(1, int(round(getattr(kin, "collision_width", 0.25) / self.stock.ds)))
        best, at = 0.0, None
        flagged = ex > self.collision_tol
        d = np.diff(np.concatenate([[0], flagged.astype(np.int8), [0]]))
        for a, b in zip(np.nonzero(d == 1)[0], np.nonzero(d == -1)[0]):
            if b - a >= k:
                j = a + int(np.argmax(ex[a:b]))
                if ex[j] > best:
                    best, at = float(ex[j]), acc["where"][j]
        return (best, at) if at is not None else None

    def _finish_move(self, mi):
        move = self.moves[mi]
        acc = self._move_events.pop(mi, None)
        if acc:
            name = self.ops[move.op].name
            if move.kind == RAPID and acc["removed"] > self.rapid_tol:
                turns = any(abs(move.end[i] - move.start[i]) > 1e-9 for i in (3, 4, 5))
                vol = acc.get("volume", 0.0)
                if turns:
                    msg = "turning the rotary axis sweeps the stock into the tool"
                else:
                    msg = "G0 cuts %.3f mm into stock" % acc["removed"]
                if vol > 0:
                    msg += " (%.1f mm3 cut)" % vol
                self.events.append(Event("rapid_cut", move.op, name, move.index, tuple(acc["pr"]),
                                         vol if turns else acc["removed"], msg))
            hit = self._collision(acc)
            if hit:
                depth, at = hit
                self.events.append(Event(
                    "holder_collision", move.op, name, move.index, tuple(at), depth,
                    ("stock %.3f mm inside the insert's back: the boring bar would collide"
                     if self.ops[move.op].internal
                     else "stock %.3f mm above the insert's top: holder would collide") % depth))
        if self.turning and not any(e.kind == "parted" for e in self.events):
            prof = self.stock.profile()
            gone = self.stock.material() <= 1e-3
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
        self._store_snapshot(mi)

    def _store_snapshot(self, mi):
        """Snapshot at the end of move mi, thinned out to stay within the memory budget."""
        if mi % self._snap_stride:
            return
        snap = self.stock.snapshot()
        self._move_snaps[mi] = (snap, len(self.events))
        self._snap_bytes += snap.nbytes
        while self._snap_bytes > self.snapshot_budget and len(self._move_snaps) > 1:
            self._snap_stride *= 2
            keep = {k: v for k, v in self._move_snaps.items() if k % self._snap_stride == 0}
            if len(keep) == len(self._move_snaps):
                break
            self._move_snaps = keep
            self._snap_bytes = sum(v[0].nbytes for v in keep.values())

    # ------------------------------------------------------------ helpers
    def path_polylines(self, step=None):
        """Per move: (kind, op, points (n, 3)) for path display."""
        step = step or max(self.step * 4, 0.2)
        out = []
        for m in self.moves:
            pts = np.vstack([np.asarray(m.start)[None, :], m.sample(step, 5.0)])[:, :3]
            out.append((m.kind, m.op, pts))
        return out
