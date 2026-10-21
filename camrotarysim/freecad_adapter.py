# SPDX-License-Identifier: LGPL-2.1-or-later
"""
FreeCAD side: turns a CAM Job into simulation inputs for the core.

Access patterns follow CAM's own SimulatorGL.py (FreeCAD main):
- Jobs: ActiveDocument.findObjects("Path::FeaturePython", "Job.*")
- ops: job.Operations.OutList, active per Path.Base.Util.opProperty(op, "Active")
- commands: PathScripts.PathUtils.getPathWithPlacement(op).Commands
- tool: Path.Dressup.Utils.toolController(op).Tool (works through dressups)
- model: job.Model.OutList, stock: job.Stock.Shape
"""

import numpy as np

import FreeCAD
import Part

import Path.Base.Util as PathUtil
import Path.Dressup.Utils as PathDressup
from PathScripts import PathUtils

from .core import compare
from .core.simulation import OpInput
from .core.stock import RotaryStock
from .core.tool import REF_IMAGINARY_TIP, REF_ORIGIN, ToolOutline

# Thread op properties passed to the thread check (TurningAddon PathTurnThread)
THREAD_PROPS = ("Pitch", "ThreadDepth", "ThreadDiameter", "ThreadReport", "ThreadStandard", "ThreadSize")

Y_AXIS = FreeCAD.Vector(0, 1, 0)


def find_jobs(doc=None):
    doc = doc or FreeCAD.ActiveDocument
    if doc is None:
        return []
    return doc.findObjects("Path::FeaturePython", "Job.*")


def active_operations(job):
    return [op for op in job.Operations.OutList if PathUtil.opProperty(op, "Active")]


def tool_of(op):
    tc = PathDressup.toolController(op)
    return tc.Tool if tc else None


def is_turning_op(op):
    """TurningAddon op, or any op whose tool has the Isoshape insert parameters."""
    proxy = getattr(op, "Proxy", None)
    if proxy is not None and type(proxy).__module__.startswith("PathTurnScripts"):
        return True
    tool = tool_of(op)
    return tool is not None and all(hasattr(tool, p) for p in ("TipRadius", "EdgeLength", "TipAngle"))


def op_commands(op):
    return [(c.Name, dict(c.Parameters)) for c in PathUtils.getPathWithPlacement(op).Commands]


def _value(q):
    return float(getattr(q, "Value", q))


def edge_dicts(edges, deflection=0.005):
    """Part edges in the XZ plane -> dicts for ToolOutline.from_section / chaining."""
    out = []
    for e in edges:
        pts = [(p.x, p.z) for p in e.discretize(Deflection=deflection)]
        d = {"kind": "line", "points": pts}
        if isinstance(e.Curve, (Part.Circle, Part.ArcOfCircle)):
            c = e.Curve.Center
            d.update(kind="arc", centre=(c.x, c.z), radius=float(e.Curve.Radius))
        out.append(d)
    return out


def tool_reference(tool):
    """
    Which point of the tool shape the path is programmed to: the imaginary
    tip for Isoshape tools, the shape origin for the TurningAddon's other
    shapes (threading, grooving, full radius).
    """
    shape_id = str(getattr(tool, "ShapeID", "") or "").lower()
    if shape_id == "isoshape" or (not shape_id and hasattr(tool, "EdgeLength") and hasattr(tool, "Rotation")):
        return REF_IMAGINARY_TIP
    return REF_ORIGIN


def tool_outline(tool):
    """
    The insert outline from the tool's own solid: section in the XZ plane at
    mid thickness, in the tool shape's coordinates (Placement reset, as
    SimulatorGL does). Falls back to the shape parameters (Isoshape,
    Threading). Returns (ToolOutline, note).
    """
    tip_r = _value(tool.TipRadius) if hasattr(tool, "TipRadius") else None
    reference = tool_reference(tool)
    note = ""
    shape = getattr(tool, "Shape", None)
    if shape is not None and not shape.isNull():
        try:
            shape = shape.copy()
            shape.Placement = FreeCAD.Placement()
            bb = shape.BoundBox
            wires = shape.slice(Y_AXIS, (bb.YMin + bb.YMax) / 2.0)
            edges = edge_dicts([e for w in wires for e in w.Edges])
            if edges:
                return ToolOutline.from_section(edges, tip_r, "tool shape of %s" % tool.Label,
                                                reference=reference), note
            note = "tool shape has no section in the XZ plane"
        except Exception as e:  # fall back below
            note = "tool shape section failed: %s" % e
    if all(hasattr(tool, p) for p in ("EdgeLength", "TipAngle", "TipRadius", "Rotation")):
        t = ToolOutline.from_iso(_value(tool.EdgeLength), _value(tool.TipAngle),
                                 _value(tool.TipRadius), _value(tool.Rotation))
        return t, (note + "; " if note else "") + "using Isoshape parameters"
    if all(hasattr(tool, p) for p in ("ProfileAngle", "TipRadius", "Width")):
        t = ToolOutline.from_threading(_value(tool.ProfileAngle), _value(tool.TipRadius), _value(tool.Width))
        return t, (note + "; " if note else "") + "using Threading parameters"
    raise ValueError("cannot build an outline for tool %s (%s)" % (tool.Label, note or "no shape"))


def section_polylines(shape, y=0.0, deflection=0.005):
    """Wires of shape at plane Y=y as lists of (x, z) points."""
    polylines = []
    for w in shape.slice(Y_AXIS, y):
        for e in w.Edges:
            polylines.append([(p.x, p.z) for p in e.discretize(Deflection=deflection)])
    return polylines


def model_shape(job):
    shapes = [o.Shape for o in job.Model.OutList if hasattr(o, "Shape") and not o.Shape.isNull()]
    if not shapes:
        return None
    return shapes[0] if len(shapes) == 1 else Part.makeCompound(shapes)


def build_stock(job, ds):
    """RotaryStock about Z from the Job's stock solid: outside and bore from its XZ section."""
    shape = job.Stock.Shape
    bb = shape.BoundBox
    polylines = section_polylines(shape)
    stock = RotaryStock.from_profile(lambda s: compare.part_profile(polylines, s), bb.ZMin, bb.ZMax, ds,
                                     inner_fn=lambda s: compare.part_inner_profile(polylines, s))
    if stock.R.max() <= 0:
        raise ValueError("the Job's stock has no section in the XZ plane")
    return stock


def is_internal(op):
    """
    Whether an op cuts inside (boring, internal threads), as the op itself
    decides: TurningAddon ops answer through their proxy (is_internal for
    Rough/Profile with Side = Internal, internal for Turn Thread, which
    follows the selected face unless set); otherwise the Side / ThreadType
    properties.
    """
    proxy = getattr(op, "Proxy", None)
    for name in ("is_internal", "internal"):
        fn = getattr(proxy, name, None)
        if callable(fn):
            try:
                return bool(fn(op))
            except Exception:
                pass
    return getattr(op, "Side", "") == "Internal" or getattr(op, "ThreadType", "") == "Internal"


def hole_from_ops(ops):
    """The starting hole the internal ops bore from: the smallest Min Diameter set on one, or None."""
    sizes = [(_value(op.MinDiameter) / 2, op.Label) for op in ops
             if is_internal(op) and getattr(op, "SetMinDiameter", False) and _value(getattr(op, "MinDiameter", 0)) > 0]
    return min(sizes) if sizes else None


def bore_depth(part_polylines, stock, ds):
    """
    Lowest Z of the part's bore that is open toward the front (+Z), or None.
    A bore running out at the part's back end (a through bore) returns the
    stock's end.
    """
    s = stock.s
    pin = compare.part_inner_profile(part_polylines, s)
    pout = compare.part_profile(part_polylines, s)
    present = np.nonzero(pout > 0)[0]
    if not len(present):
        return None
    j = present[-1]          # the part's front column
    if pin[j] <= 0:
        return None          # no bore open to the front
    while j > present[0] and pin[j - 1] > 0:
        j -= 1
    return stock.s_min if j == present[0] else float(s[j] - ds / 2)


def spindle_sense(op, forward_right_hand=True):
    """
    +1 / -1 for OpInput.spindle from the tool controller's SpindleDir.
    Path commands don't fix the thread hand: forward_right_hand says what a
    Forward spindle cuts when the tool feeds toward -Z (+1: right-hand).
    """
    tc = PathDressup.toolController(op)
    forward = 1 if forward_right_hand else -1
    return -forward if str(getattr(tc, "SpindleDir", "Forward")) == "Reverse" else forward


def op_meta(op):
    meta = {}
    for p in THREAD_PROPS:
        if hasattr(op, p):
            v = getattr(op, p)
            meta[p] = v if isinstance(v, str) else _value(v)
    tc = PathDressup.toolController(op)
    if tc is not None and hasattr(tc, "SpindleDir"):
        meta["SpindleDir"] = str(tc.SpindleDir)
    return meta


def build_inputs(job, ops, ds, forward_right_hand=True, hole="auto"):
    """
    (stock, [OpInput], part polylines, notes) for the selected ops.

    hole: the starting hole internal ops bore from, until drilling is
    simulated: "auto" (the smallest Min Diameter set on an internal op,
    from the stock's front down the part's bore, or through), None, or
    (diameter, depth from the stock's front).
    """
    stock = build_stock(job, ds)
    model = model_shape(job)
    polylines = section_polylines(model) if model is not None else []
    inputs, notes, cache = [], [], {}
    for op in ops:
        tool = tool_of(op)
        if tool is None:
            notes.append("%s has no tool controller; skipped" % op.Label)
            continue
        if tool.Name not in cache:
            outline, note = tool_outline(tool)
            cache[tool.Name] = outline
            notes.append("%s: outline from %s%s" % (tool.Label, outline.source, " (%s)" % note if note else ""))
        inputs.append(OpInput(op.Label, op_commands(op), cache[tool.Name],
                              spindle=spindle_sense(op, forward_right_hand), meta=op_meta(op),
                              internal=is_internal(op)))
        if str(inputs[-1].meta.get("SpindleDir", "")) == "None" and any(
                c[0].upper().startswith("G33") for c in inputs[-1].commands):
            notes.append("%s: spindle direction None on a thread op; simulated as Forward" % op.Label)
    stock.hole = None
    if hole == "auto":
        found = hole_from_ops(ops)
        if found:
            r, label = found
            z_lo = bore_depth(polylines, stock, ds)
            through = z_lo is not None and z_lo <= stock.s_min
            z_lo = stock.s_min if z_lo is None else z_lo
            stock.add_hole(r, z_lo, stock.s_max)
            stock.hole = (2 * r, stock.s_max - z_lo)
            notes.append("starting hole %.3f from %s's Min Diameter, %s" % (
                2 * r, label, "through" if through else "%.3f deep (to the bottom of the model's bore)"
                % (stock.s_max - z_lo)))
    elif hole:
        d, depth = hole
        if d > 0 and depth > 0:
            stock.add_hole(d / 2, stock.s_max - depth, stock.s_max)
            stock.hole = (d, depth)
            notes.append("starting hole %.3f, %.3f deep (set in the panel)" % (d, depth))
    return stock, inputs, polylines, notes
