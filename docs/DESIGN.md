# CAM Rotary Simulator: design and conventions

Reference for the conventions the simulator relies on and how the code is laid out.
The README is the user guide; CONTRACT.md lists what the adapter assumes about CAM Lathe Turning.

## Module map

    camrotarysim/core/      pure Python + numpy, no FreeCAD imports
      motion.py             Path commands -> moves (G0/G1/G2/G3, G33, canned drilling
                            cycles, G76, G17/18/19, A/B/C carried)
      tool.py               turning insert, threading insert and drill outlines, envelopes
      stock.py              RotaryStock: R and bore Rin per (theta, s); machine-file axes
      kinematics.py         TurningKinematics (outside, bore, one angle for G33)
      simulation.py         sampling, playback, seek (thinned snapshots), events
      compare.py            part and bore profiles, reachable profiles, classification
      thread_check.py       thread zones and measurement against the op
      mesh.py               revolved mesh with bores and per-vertex colours
      report.py             text report (turning)
      milling.py            VoxelStock (distance grid), cutters, MillingKinematics,
                            rotary angles, part check, report (milling)
      surface.py            surface nets, trilinear sampling, mesh -> distance grid
      indexing.py           retract and rotary index moves between work planes
    camrotarysim/freecad_adapter.py   Job -> core inputs
    camrotarysim/gui/       command + workbench manipulator, task panel, Coin3D scene

## Commands and moves (turning)

- Commands are the ops' `Path.Commands` through `getPathWithPlacement`, as CAM's
  simulator reads them: absolute coordinates in mm and degrees, X a radius.
- IJK are relative to the arc start point. G17 = XY (normal +Z), G18 = ZX (normal +Y),
  G19 = YZ (normal +X). G2 is clockwise and G3 counter-clockwise seen from the
  positive end of the plane normal; for G18 that is Z to the right and X up.
- G7/G8 (diameter mode) and G91 are reported, not applied.

## Tool outlines and control points

The outline is a closed polygon in (x, z) relative to the control point, the point
the path is programmed to. It is taken from the op's tool solid, sectioned in the
XZ plane through the rake face at Y = 0 (mid-thickness would be inside a holder drawn
in the same solid), with the Placement's Y offset kept (models made with CAM Lathe
Turning 0.17.4 to 0.18.0 use it to put the rake face at Y 0) and X, Z and rotation
reset: at Y +0.01 (models from 0.17.4 on), else at Y -0.01 (models from before 0.17.4,
Isoshape insert under Y = 0) when +0.01 gives no edges or no Isoshape nose of TipRadius. Without a usable section
it is built from the shape parameters (Isoshape, Threading, Drill).

Control point by shape (CAM Lathe Turning `TurnToolShapes.py`):

- Isoshape: the nose arc nearest the shape origin minus (r, r), the imaginary tip.
  The nose centre is therefore at (X + r, Z + r). This follows LibLathe's
  `nose_compensate` (internal -X frame, flipped by `to_commands(invert_x=True)`), and
  finds the tip whether the shape's origin is at the imaginary tip or the sharp corner.
- Threading, Grooving, Full radius: the shape origin. For threading that is the V's
  sharp point; for grooving and full radius blades the edge's lowest point at the
  chuck-side flank.
- CAM's Drill (and reamers): a revolution about Z with the tip at the origin and the
  body toward +Z; the tip is programmed. On a lathe it cuts along the spindle axis at
  X 0 as an internal tool.

Isoshape conventions (from `isoshape.fcstd`, which binds TipAngle, -Rotation,
EdgeLength and TipRadius, and LibLathe's `get_tool_cutting_angle()`):

- the shape origin is the theoretical sharp corner of the active tip;
- the two edges at the tip leave it at Rotation - TipAngle/2 and Rotation + TipAngle/2,
  measured from +Z toward +X;
- the insert is a rhombus with side EdgeLength between sharp corners, every corner
  rounded by TipRadius.

Both outline sources give the same result for the sample tool (`devtools/run_sample_job.py`).

## Internal ops

Whether an op cuts inside is the op's own answer: drilling ops (Turn Drill, or any op
with a CAM drill) always do; otherwise Turn Thread's `internal()` is asked first, then
`is_internal()` (Rough and Profile with Side = Internal), and either saying so counts;
without a proxy answer, the `Side` or `ThreadType` property.

LibLathe bores by turning in a mirror (r -> R - r), so its paths go to the mirrored
imaginary tip. The simulator mirrors the outline the same way and draws the tool
mirrored. An internal tool widens the bore to the top of its outline.

## Drilling cycles

G81, G82, G83 and G73 are expanded as LinuxCNC's G-code reference describes them: along
the selected plane's normal (Z in G17), with the preliminary rapids and pecks. The peck
clearance is 0.254 mm unless D is given, the final retract follows G98/G99, R is sticky,
and Z and Q within a series. G85 to G89 and cycles in G91 are reported, not simulated.

## Threads

- A G33 after a non-synchronized move waits for the spindle index, then moves K per
  revolution along its drive line, starting D degrees past the index. A G33 straight
  after another continues the same helix (LinuxCNC `tp.c`).
- A G76 cycle is expanded as LinuxCNC's `convert_threading_cycle`: depths |I| + J, then
  |I| + J * n^(1/R) while short of |I| + K, then H + 1 full passes; each pass shifted
  along the drive line by (depth - |I|) tan Q; E/L entry and exit tapers that keep the
  lead along Z; D as for G33.
- The tool stays at world angle 0, so it passes workpiece angle theta when the spindle
  has turned -theta. The rapid out after a G33 is taken as instant and cuts nothing.
- The hand is a setting: Path commands do not fix it.
- Measurement, over the interior whole pitches: major diameter from the crests, minor
  from the roots, pitch diameter as the median radius (where groove and ridge are
  equally wide). If the groove is narrower (or wider) than the ridge even at the crests
  (roots), the pitch line is extrapolated along the straight flanks. Internal threads
  are measured on the bore, where crests and roots swap. Results are compared with the
  Turn Thread op's Thread*Dia properties (mm) and ThreadRootDia. Older
  ops without them fall back to the ThreadReport ranges and ThreadDiameter -/+ 2
  ThreadDepth; an inch report (UNC/UNF) is then not compared rather than misread.

## Stock and part (turning)

The stock is a cylindrical height field about Z: an outer radius R and a bore radius
Rin per angle and axial column (turning uses one angle; threading adds angles). The
profiles of the Job's stock solid and model come from sections at Y = 0; parts are
taken as solids of revolution about Z, and the report shows the part's X offset from
the axis when it exceeds 1 um.

Removal: each move is sampled at the resolution; at each sample the stock is lowered,
at every angle, to the insert's lower envelope in every axial column it covers.

Reachable profile: a 1D morphological opening of the part by the insert's lower
envelope, the best result the insert can produce from outside. Material above it is
"unreachable"; anything else left is the path's doing.

Collisions: stock standing beyond the insert's back (above its top, or inside its
bottom for an internal tool) over at least 0.25 mm of the columns a move crosses.

## Milling

Frames follow CAM's own (`Path/Post/Processor.py`, `TiltedWorkPlane.py`): an op on a
work plane stores its path in the plane's frame and carries the plane as its Placement,
so world = Placement * local and the tool axis is Placement.Rotation * Z. A table rotary
axis turns the workpiece: machine = R(axis, A) * world (`compute_rotation_matrix`), so a
position with an A word is in the workpiece frame at R(axis, -A) * machine. Indexed ops
have no A words; the machine's A is the one that turns the plane's normal to +Z.
`applyPlacementToPath` is not used because it leaves arcs tilted out of XY as they are.

The stock is a signed-distance grid (phi positive inside material). Each cutter position
is subtracted exactly, phi = min(phi, distance outside the cutter); the surface is
rebuilt at phi = 0 by surface nets with sub-grid interpolation, which chamfers sharp
edges by up to a third of the grid. Box stock is exact; other stock comes from its
tessellation. The part check is on the grid against the model's tessellation: gouge
where the model has material, left beyond the model, exact up to three grid spacings.

Indexing between ops on different work planes runs what the posted program runs: the
pre-rotary move, the rotary axis turning as a move (one degree per sample), then the
next op from where the tool is, carried through the machine frame. A G53 pre-rotary
move cannot be placed without the work offset and is shown as a retract to the stock's
swing radius plus the SetupSheet clearance.
