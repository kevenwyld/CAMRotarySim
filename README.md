# CAM Rotary Simulator

| :warning: WARNING: This code is AI generated, use extreme caution.  Meant as a proof of concept only. |

A proof-of-concept FreeCAD addon that simulates material removal and checks
the result against the Job's model. It covers:
- turning, drilling, boring and threading ops of the
  [CAM Lathe Turning](https://github.com/kevenwyld/CAMLatheTurning) addon
  (formerly TurningAddon), on a rotary height field (a radius per angle and
  axial position);
- CAM milling ops, indexed or rotary (4th axis), on a signed-distance grid.

It is a standalone simulator. CAM's new simulator (C++/OpenGL, `CAMSimulator`)
can't be extended from an addon:

- It parses only the words `G T X Y Z I J K R`. It ignores plane selection
  (G17/G18/G19) and takes any arc with an I or J offset as an XY arc, so G18
  arcs are wrong. A/B/C words are dropped, and G33 runs as a plain linear move.
- Its tools must be solids of revolution about their own axis.
- It removes material in image space, so no geometry is left to measure.

The UI follows its conventions: Job combo, operation list with "follows
visibility", play/pause, single step, a slider, speed steps 1/2/5/10/25/50.
It adds a reset, and ×0.25 and ×0.5 for short paths.

https://github.com/user-attachments/assets/ad96ca03-bc3d-4b20-8266-5192bdd13e37

https://github.com/user-attachments/assets/3a9bca94-52dd-4211-a65e-e46daf2ec9d1

## Install

Link or copy this folder into your user Mod directory, for example:

    ln -s /path/to/CAMRotarySim ~/.local/share/FreeCAD/v27-1/Mod/CAMRotarySim

Restart FreeCAD. In the CAM workbench the command **Rotary/Turning Simulator**
appears right after the simulators, in both the CAM menu and the "Tool Commands"
toolbar. Minimum FreeCAD 1.1. Requires numpy, which CAM already uses.

## Use

1. Open a Job with turning or milling operations and start the command.
2. Pick the Job. Turning and milling ops are pre-checked. Turning and milling
   ops can't be simulated together yet, so select one kind.
3. Press play. The stock is drawn in the 3D view and the Job's stock and model
   are hidden while the panel is open (untick the option to keep them).
4. Controls:
   - **Slider:** scrub forwards or backwards.
   - **Step:** run to the end of the current move.
   - **− / +:** change the speed.
   - **Reset:** back to the start.
5. Colours on the stock:

   | Colour | Meaning |
   |---|---|
   | Blue | untouched stock |
   | Green | cut surface |
   | Red | gouge (below the part by more than the gouge tolerance) |
   | Amber | uncut material the tool could have reached (milling: any material left beyond the part) |
   | Purple | uncut material this insert can't reach (turning only) |

6. Threads (G33 passes, e.g. CAM Lathe Turning's Turn Thread) are cut along
   their helix. While a pass runs, the displayed workpiece turns with the
   spindle, so the tool sits in its groove. In the thread's Z range the gouge
   check is replaced by a thread measurement. Its colour is green if the pitch
   diameter is in the op's range, amber if the thread is not cut deep enough,
   and red if it is cut too deep; it is light blue until measured, when
   playback pauses or ends. Two settings control threads:
   - **Thread angles:** the angular resolution of thread zones.
   - **Forward spindle cuts:** the hand a Forward spindle cuts when feeding
     toward −Z. Path commands don't fix the hand; Reverse cuts the other one.
7. Internal ops are simulated inside: drilling (Turn Drill, or any op with a
   CAM drill), boring (Rough and Profile with Side = Internal) and internal
   threads. If nothing selected drills the hole, set the hole they start from
   with **Starting hole**:
   - **Auto:** none if a drilling op is selected. Otherwise the smallest Min
     Diameter set on an internal op, from the stock's front down the model's
     bore, or through the stock if the bore runs out at the back.
   - **None.**
   - **Custom:** a diameter and a depth from the stock's front.

   A stock that already has a bore (a tube, say) keeps it. The bore is drawn
   and checked like the outside.
8. The report lists the worst values, the ranges, the bore, threads, events
   and path diagnostics:
   - **Events:** rapid moves that cut, holder collisions, parting, and (milling)
     a rotary index that sweeps the stock into the tool.
   - **Path diagnostics:** arc radius mismatches, zero feed rates,
     unsupported commands.

## Milling (indexed and rotary)

CAM milling ops (Adaptive, Pocket, Profile and so on) are simulated in the
workpiece frame, on a stock that can be cut from any direction.

**Frames.** Each op's commands are taken as stored, local to its work plane,
and placed by the op's Placement: world = Placement × local, with the tool
axis along Placement × Z. This is how CAM's post reads them. CAM's
`applyPlacementToPath` is not used, because it leaves arcs tilted out of XY
as they are, which is wrong on side work planes. A rotary word (A, B or C) in
a command turns the workpiece: the tool is placed at R(axis, −angle) × world
(CAM's `compute_rotation_matrix`).

**Rotary angle.** For each op the report gives the angle that turns its work
plane's normal to +Z. The 3D view turns the part by that angle, with the tool
upright, as on the machine. Compare these angles with the posted program.

**Indexing.** Between ops whose angles differ, the simulator runs what the
posted program runs: the pre-rotary move, then the rotary axis turning as a
move, then the next op from where the tool is. The tool's position is carried
through the machine frame, since each op's coordinates are in its own plane.
Playback turns the part at 3° per tick × the speed, so the rotation is seen.
The **Retract before indexing** setting has three choices:
- **Auto:** the machine's pre-rotary move. A G53 move is in machine
  coordinates, which can't be placed without the work offset, so it is shown
  as a retract to the stock's swing radius about the axis plus the
  SetupSheet's clearance offset, and the report says so.
- **None:** the axis turns with the tool where it is. If the turning stock
  sweeps through the tool, that is cut and reported.
- **Custom:** a Z.

**Rotary axis.** Taken from the Job's machine through CAM's
`MachineFactory`; without one, A along X through the origin is assumed.

**Stock.** A signed-distance grid (the **Milling grid** setting, 0.5 mm by
default) in place of the turning model, which can't represent walls that
aren't radial.
- Each cutter position is subtracted exactly.
- The surface is drawn at sub-grid accuracy; sharp edges are chamfered by up
  to a third of the grid.
- Box stock is exact; other stock comes from its tessellation.

**Check.** On the grid against the model's tessellation:
- **Gouge:** cut where the model has material.
- **Left:** material beyond the model, measured exactly up to 3 grid spacings.

**Tools.** Flat end mills, and ball end mills (ShapeID `ballend`). Other
shapes are treated as flat, and the report says so. The cutting length is
CuttingEdgeHeight.

**Display.** "Part outline" shows the model as a see-through mesh.

## Inputs and conventions

These were verified against source and the sample Job, not assumed:

- **Commands (turning):** the ops' `Path.Commands`, through
  `getPathWithPlacement`, as CAM's simulator reads them. X is a radius.
  Milling ops use their own local commands instead (see Milling). G2/G3 follow the plane
  selection; G18 is the ZX plane, CW/CCW as seen from +Y (Z right, X up).
  IJK are relative to the start point.
- **Imaginary tip:** paths are programmed to the insert's imaginary tip. The
  nose centre is at (X + r, Z + r). This is from LibLathe `nose_compensate` with
  its internal −X frame, which `to_commands(invert_x=True)` flips.
- **Insert outline:** taken from the op's tool solid, sectioned in the XZ plane
  through the shape's origin (just under Y = 0), where the cutting edge is. A
  holder drawn in the same solid stands off that plane and is left out. A
  boring bar crossing it comes into the outline, which only matters where the
  bar would hit the stock (holders aren't collision-checked yet). An Isoshape
  section without a nose of the tool's TipRadius falls back to the
  parameters. The control point depends on the shape (CAM Lathe Turning
  `TurnToolShapes.py`):
  - **Isoshape:** the nose arc nearest the shape origin, minus (r, r): the
    imaginary tip.
  - **Threading, Grooving, Full radius:** the shape origin. For threading that
    is the V's sharp point, a tip radius below the tip arc's bottom.

  The Isoshape rule finds the imaginary tip wherever the shape's origin is:
  CAM Lathe Turning's current `isoshape.fcstd` puts it at the imaginary tip,
  while older documents keep shapes with it at the sharp corner.

  If there is no usable shape, the outline is built from the Threading,
  Isoshape or Drill parameters. The Isoshape outline follows the sketch of
  `isoshape.fcstd`:
  - The sharp corner of the active tip is the corner the edges leave from.
  - Edges run at Rotation ± TipAngle/2, measured from +Z toward +X.
  - The insert is a rhombus with side EdgeLength between sharp corners, every
    corner rounded by TipRadius.
  These agree with LibLathe's `get_tool_cutting_angle()`. Both sources give
  the same outline for the sample tool; see `devtools/run_sample_job.py`.
- **Drilling:** CAM drills (and reamers) are revolved shapes with the tip at
  the origin; on a lathe they cut along the spindle axis at X 0, as internal
  tools.
  - Canned cycles G81, G82, G83 and G73 are expanded as LinuxCNC's G-code
    reference describes them: along the selected plane's normal (Z in G17),
    with preliminary motion and pecks.
  - G73 and G83 peck clearance is 0.254 mm unless D is given.
  - The final retract follows G98/G99.
  - R is sticky, and Z and Q within a series.
  - The same moves written out as G0/G1 work as any other path.
- **Internal ops:** whether an op cuts inside is the op's own answer:
  - drilling ops (Turn Drill, or any op with a CAM drill) always do;
  - Turn Thread's `internal()`, which follows the selected face, is asked
    first, then `is_internal()` (Rough and Profile with `Side = Internal`).
    Every CAM Lathe Turning op inherits an `is_internal()` that is False without a
    Side, so either saying so counts;
  - otherwise the `Side` or `ThreadType` property.

  LibLathe bores by turning in a mirror (r → R − r), so its paths go to the
  mirrored imaginary tip, the nose's outermost point. The simulator mirrors
  the outline the same way and draws the tool mirrored. An internal tool
  widens the bore to the top of its outline.
- **Threads (G33, G76):** per LinuxCNC's G-code reference and motion planner
  (`tp.c`), a G33 after a non-synchronized move waits for the spindle index,
  then moves K per revolution along its drive line, starting D degrees past
  the index. A G33 straight after another continues the same helix. A G76
  cycle is expanded into its passes as LinuxCNC's interpreter does
  (`convert_threading_cycle`): pass depths from J, K and R, the Q infeed
  shift, H spring passes, E/L entry and exit tapers that keep the lead along
  Z, and D. The tool stays at world angle 0, so it passes
  workpiece angle θ when the spindle has turned −θ.
  - The rapid out after a G33 is taken as instant and cuts nothing.
  - The thread measurement covers the interior whole pitches:
    - major diameter: the crests;
    - minor diameter: the roots;
    - pitch diameter: where groove and ridge are equally wide, i.e. the median
      radius.
  - These are compared with the pitch, major or minor diameter ranges in the
    Turn Thread op's ThreadReport, and the deepest pass with its
    ThreadDiameter and ThreadDepth.
  - Internal threads are measured on the bore, where crests and roots swap.
  - If the groove is narrower (or wider) than the ridge even at the crests
    (roots), the pitch diameter lies outside the material and is extrapolated
    along the straight flanks.
- **Stock and part (turning):** the profile of the Job's stock solid and of its model,
  sectioned at Y = 0. Parts are assumed to be solids of revolution about Z. The
  report shows the part's X offset from the axis if it is more than 1 µm.

## Design

    camrotarysim/core/      pure Python + numpy, no FreeCAD imports
      motion.py             Path commands -> moves (G0/G1/G2/G3, G33, canned drilling
                            cycles, G17/18/19, A/B/C carried)
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

- **Turning removal:** each move is sampled at the resolution. At each sample
  the stock is lowered, at every angle, to the insert's lower envelope in every
  axial column it covers.
- **Milling removal:** each move is sampled (every 0.25 mm, or half the grid
  if finer). At each sample the cutter, placed in the workpiece frame by the
  op's frame and any rotary word, is subtracted from the grid:
  phi = min(phi, distance outside the cutter).
- **Reachable profile:** a 1D morphological opening of the part by the insert's
  lower envelope. It is the best result the insert can produce from outside.
  Material left above it is "unreachable"; anything else left is the path's
  doing.

## Limits

- **Holder, boring bar and chuck:** not modelled. "Reachable" and the
  collision check use the insert outline only. A collision is reported when
  stock stands beyond the insert's back over at least 0.25 mm of the columns a
  move crosses: above its top, or inside its bottom for an internal tool, as
  when boring from solid. A radial plunge clears its own kerf, so plunging
  deeper than the insert height is not reported.
- **Stock shape (turning):** an outside and a bore per angle and position, so
  no undercut pockets in a wall. The axis must be Z.
- **Drilling:** canned cycles G85 to G89 (boring, tapping) are reported, not
  simulated; so are cycles in G91. A drill is taken off-axis only as far as the
  path moves it (lathe drilling is at X 0).
- **Diameter mode:** G7/G8 are reported, not applied.
- **Threads:**
  - The tool is taken as a flat profile in the XZ plane, ignoring the helix's
    lead angle.
  - The thread hand is a setting.
- **Milling:**
  - Turning and milling ops can't be simulated together yet.
  - Shanks and holders are not checked for collisions.
  - Cutters other than flat and ball are approximated.
  - Continuous rotary moves (rotary words while cutting) are supported but
    haven't been tested on a real op.
- **Accuracy:**
  - Turning depends on the Resolution setting (column spacing and sample
    step, default 0.05 mm).
  - Milling depends on the Milling grid (default 0.5 mm). Faces are accurate
    well below the grid; drawn edges are chamfered by up to a third of it.
- **Parting:** reported when a column reaches the axis with substantial
  material on both sides (both pieces over 1 mm radius). The parted piece is
  kept for display and comparison.

## Tests

    python3 -m unittest discover -s tests

- **Core:** pure tests, plus end-to-end runs of the sample Jobs (fixtures
  extracted from `turning_test2.FCStd`, `threading_test.FCStd`,
  `boring_test.FCStd` and `inexed_rotary_test.FCStd`).
- **FreeCAD side:** `tests/test_adapter.py` and `tests/test_gui_headless.py`
  run the adapter, task panel, Coin scene and command registration. They use
  stub FreeCAD modules (`tests/fakefreecad.py`), a fake Job built from the
  fixtures (`tests/fakejob.py`) and real PySide6 (offscreen). The GUI test is
  skipped without PySide6. These check the addon's logic, not FreeCAD's own
  behaviour.
- **Dev tools:**
  - `devtools/run_sample_job.py --out DIR` prints the report and writes plots.
  - `devtools/make_partoff_guess.py` builds the Partoff fixtures with LibLathe.
  - `devtools/make_drill_fixture.py` builds the Turn Drill fixtures by running
    CAM Lathe Turning's own `op_generate_gcode()`.
  - `devtools/run_rotary_job.py --out DIR` runs the indexed rotary Job and
    plots sections through each machined face against the model.
  - `devtools/make_thread_g76_fixture.py` writes the thread Job's thread as
    LibLathe's G33 passes or its G76 cycle.
  - `devtools/run_thread_job.py --out DIR` runs the thread Job and plots an
    axial section against the ISO basic profile, plus an unrolled
    radius-versus-angle map that shows the hand.
