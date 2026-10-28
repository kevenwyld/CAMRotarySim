# SPDX-License-Identifier: LGPL-2.1-or-later
"""Plain-text simulation report (used by the task panel and dev tools)."""

from collections import OrderedDict

import numpy as np


def axis_offset(part_pos, part_neg, min_radius=1.0):
    """Estimate of the part's X offset from the spindle axis (mm), from both section halves."""
    m = (part_pos > min_radius) & (part_neg > min_radius)
    if not m.any():
        return None
    return float(np.median(part_pos[m] - part_neg[m]) / 2.0)


def _ranges(out, result, max_lines, labels):
    for kind, label in labels:
        w = result.worst(kind)
        out.append("  %s: %s" % (label, "none" if w is None else "%+.3f mm at Z %.3f" % (w.worst, w.at)))
    for kind, _label in labels:
        rs = [r for r in result.ranges if r.kind == kind]
        if not rs:
            continue
        out.append("  %s ranges (Z from..to, worst):" % kind)
        for r in sorted(rs, key=lambda r: -abs(r.worst))[:max_lines]:
            out.append("    %9.3f .. %9.3f  %+.3f mm" % (r.s0, r.s1, r.worst))
        if len(rs) > max_lines:
            out.append("    ... %d more" % (len(rs) - max_lines))


def format_report(sim, result=None, part_pos=None, part_neg=None, max_lines=8, threads=None, bore=None):
    out = []
    state = "complete" if sim.done else "%.0f%% simulated" % (100 * sim.progress)
    out.append("Simulation %s: %d moves, %d samples." % (state, len(sim.moves), sim.total_samples))

    if result is not None:
        out.append("")
        out.append("Comparison with the part:")
        for kind, label in (("gouge", "Worst gouge"), ("uncut", "Worst uncut (path)"),
                            ("unreachable", "Worst unreachable (tool shape)")):
            w = result.worst(kind)
            if w is None:
                out.append("  %s: none" % label)
            else:
                out.append("  %s: %+.3f mm at Z %.3f" % (label, w.worst, w.at))
        for kind in ("gouge", "uncut", "unreachable"):
            rs = [r for r in result.ranges if r.kind == kind]
            if not rs:
                continue
            out.append("  %s ranges (Z from..to, worst):" % kind)
            for r in sorted(rs, key=lambda r: -abs(r.worst))[:max_lines]:
                out.append("    %9.3f .. %9.3f  %+.3f mm" % (r.s0, r.s1, r.worst))
            if len(rs) > max_lines:
                out.append("    ... %d more" % (len(rs) - max_lines))
        if part_pos is not None and part_neg is not None:
            off = axis_offset(part_pos, part_neg)
            if off is not None and abs(off) > 1e-3:
                out.append("  Note: part axis offset %+.4f mm in X from the spindle axis." % off)

    if bore is not None:
        out.append("")
        out.append("Bore (positive: material left, negative: cut into the part):")
        _ranges(out, bore, max_lines, (("gouge", "Worst gouge"), ("uncut", "Worst uncut (path)"),
                                       ("unreachable", "Worst unreachable (tool shape)")))

    if threads:
        from .thread_check import format_thread
        out.append("")
        out.append("Threads (measured on the stock):")
        for t in threads:
            out += format_thread(t)

    out.append("")
    if sim.events:
        out.append("Events:")
        for e in sim.events[:max_lines * 2]:
            out.append("  %s #%d: %s at X %.3f Z %.3f" % (e.op_name, e.index, e.message, e.position[0], e.position[2]))
        if len(sim.events) > max_lines * 2:
            out.append("  ... %d more" % (len(sim.events) - max_lines * 2))
    else:
        out.append("Events: none (no rapid cuts, collisions or parting).")

    if sim.diagnostics:
        out.append("")
        out.append("Path diagnostics:")
        groups = OrderedDict()
        for d in sim.diagnostics:
            op = sim.ops[d.op].name if d.op < len(sim.ops) else str(d.op)
            groups.setdefault((op, d.kind, d.message), []).append(d.index)
        for (op, _kind, msg), idx in groups.items():
            shown = ", ".join("#%d" % i for i in idx[:6]) + (" ..." if len(idx) > 6 else "")
            out.append("  %s: %s (%s)" % (op, msg, shown))
    return "\n".join(out)
