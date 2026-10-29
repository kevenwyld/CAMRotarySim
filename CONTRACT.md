# Contract with CAM Lathe Turning and FreeCAD

What `camrotarysim/freecad_adapter.py` assumes about the ops, tools and Path that
CAM Lathe Turning produces, and what the tests' stand-in FreeCAD provides. CAM Lathe
Turning keeps the same list in its own `CONTRACT.md`; a rename on either side must be
made in both files, in the adapter and in `tests/fakejob.py`.

## CAM Lathe Turning ops

| What | Used for | Without it |
|---|---|---|
| `type(op.Proxy).__module__` starts with `PathTurnScripts` | `is_turning_op`, and never a milling op | an op still counts as turning when its tool has `TipRadius`, `EdgeLength`, `TipAngle` |
| `op.Proxy.uses_drill_tool` (True on Turn Drill) | drilling op: internal, cuts at X 0 | a tool whose `ShapeID` or `ShapeType` is `drill` or `reamer` |
| `op.Proxy.internal(op)` (Turn Thread), `op.Proxy.is_internal(op)` (every op; True for Side = Internal where supported) | internal op: mirrored tool, cuts the bore; either True counts, both False means external | the properties `Side == "Internal"` or `ThreadType == "Internal"` |
| `MinDiameter` (length), `SetMinDiameter` (bool) | Auto starting hole: the smallest Min Diameter set on an internal op | no starting hole from the ops |
| `ThreadDiameter`, `ThreadDepth` (lengths, mm) | expected root (external) or major (internal) diameter, `ThreadDiameter -/+ 2 ThreadDepth` | no root check |
| `ThreadPitchDiaMin`/`Max`, `ThreadMajorDiaMin`/`Max`, `ThreadMinorDiaMin`/`Max` (mm, 0 when the tolerance class has no data) | the thread check's expected ranges and verdict | fallback to `ThreadReport` |
| `ThreadRootDia` (mm, the diameter the deepest pass reaches) | the root check, preferred over the line above | `ThreadDiameter -/+ 2 ThreadDepth` |
| `ThreadReport` (text) | fallback for older ops: `Pitch diameter A-B`, `major diameter to A-B`, `minor diameter to A-B`, metric numbers | numbers ending in `"` (UNC/UNF): noted "inch report: not compared", no verdict |
| `Pitch`, `ThreadStandard`, `ThreadSize`, `DrillReport` | carried in the op's `meta`; not compared (the pitch comes from the G33/G76 K) | nothing |
| Tool controller `SpindleDir` (`Forward`, `Reverse`, `None`) | thread hand (with the panel's Forward setting); `None` on a G33 op is noted and simulated as Forward | Forward |

All lengths are read as `float(getattr(q, "Value", q))`, so a Quantity or a plain float
in mm both work.

## Tool shapes

`ShapeID` (lower case) selects the control point the path is programmed to:

| ShapeID | Control point | Parameter fallback when the solid has no usable XZ section |
|---|---|---|
| `isoshape` (or no ShapeID with `EdgeLength` and `Rotation`) | imaginary tip: nose arc nearest the origin minus (r, r) | `EdgeLength`, `TipAngle`, `TipRadius`, `Rotation` |
| `threading` | shape origin: the V's sharp point | `ProfileAngle`, `TipRadius`, `Width` |
| `grooving`, `fullradius` | shape origin: the edge's lowest point at the chuck-side flank | none |
| `drill`, `reamer` (CAM) | tip at the origin, body toward +Z, cutting at X 0 | `Diameter`, `TipAngle` (118), `Length` |

The outline is the tool solid's section, with the solid's Placement reset, at Y +0.01:
just inside the rake face (Y = 0) of tool models from 0.18.1 on, insert and holder on
+Y. If that gives no edges, or an Isoshape section has no nose of TipRadius, it is taken
at Y -0.01: older models, whose Isoshape insert hangs under Y = 0 (those made with 0.17.4
to 0.18.0 carry their Y offset in the Placement, which is reset). The report notes such
tools; CAM menu > Rebuild Turning Tool Models updates them. Details in `docs/DESIGN.md`.

## Path

- Turning ops: read through `PathUtils.getPathWithPlacement(op).Commands`; absolute mm and
  degrees.
- Milling ops (CAM's own, not CAM Lathe Turning): `op.Path.Commands` as stored, local to the
  op's work plane, placed by `op.Placement` (world = Placement x local, tool axis its Z), as
  CAM's post reads them; `getPathWithPlacement` is not used, as it leaves arcs tilted out of
  XY as they are.
- Turning in G18 with X a radius (G7/G8 and G91 are reported, not applied).
- Isoshape paths go to the imaginary tip; internal ops go to the mirrored tip
  (LibLathe bores by turning in a mirror). Threads are G33 passes (K the pitch) or a
  LinuxCNC G76 cycle.

## FreeCAD and CAM

`FreeCAD` (`ActiveDocument`, `ParamGet`, `Placement`, `Vector`, `Console.Print*`),
`FreeCADGui` (`Control`, `addCommand`, `addIconPath`, `addWorkbenchManipulator`,
`ActiveDocument.ActiveView.getSceneGraph`), `Part` (`Circle`, `ArcOfCircle`,
`makeCompound`; shapes' `slice`, `discretize`, `tessellate`, `copy`, `isNull`,
`BoundBox`), `Path.Base.Util.opProperty`, `Path.Dressup.Utils.toolController`,
`PathScripts.PathUtils.getPathWithPlacement`, `Machine.models.machine.MachineFactory`
(`get_machine`, `list_configuration_files`; optional), `pivy.coin`, and `PySide`
(FreeCAD's shim). Jobs are found by `doc.findObjects("Path::FeaturePython", "Job.*")`
and read through `Operations.OutList`, `Stock.Shape`, `Model.OutList`, `Machine`,
`SetupSheet.ClearanceHeightOffset`.

## The tests' stand-in FreeCAD

`tests/fakefreecad.py` `install()` puts stubs for every module above into
`sys.modules`; `install_pyside_shim()` maps `PySide` to PySide6 the way FreeCAD's
generated shim does. They implement only what the addon calls:

- `Console` records `(kind, text)` in `Console.messages`; `ParamGet` returns in-memory
  groups.
- `Part` shapes are backed by section fixtures from real Jobs (`tests/data`), not OCC:
  `slice` returns the stored section, `discretize` the stored points, `tessellate` the
  stored mesh.
- `opProperty` and `toolController` read `op.Active` and `op.ToolController`;
  `getPathWithPlacement` returns `op.Path` unchanged.
- `MachineFactory` reads the `.fcm` fixtures registered in `MachineFactory.files`.
- `pivy.coin` nodes accept any field; `FreeCADGui.Control` keeps one dialog.

`tests/fakejob.py` builds the Jobs from those fixtures with the attributes listed in
the tables above (proxy classes with a `PathTurnScripts.*` module, `internal()`,
`is_internal()`, `uses_drill_tool`, the thread properties). A test that needs a new
attribute adds it there and to this file.
