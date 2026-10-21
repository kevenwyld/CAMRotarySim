# CAM Rotary Simulator

A proof-of-concept FreeCAD addon that simulates material removal for the
turning operations of the [TurningAddon](https://github.com/kevenwyld/TurningAddon)
and checks the result against the Job's model. Its stock model is a rotary
height field, so rotary (4th-axis) milling can be added later without a
redesign.

It is a standalone simulator. CAM's new simulator (C++/OpenGL, `CAMSimulator`)
can't be extended from an addon:

- It parses only `G T X Y Z I J K R`, so G18 arcs and A/B/C words are lost.
- Its tools must be solids of revolution about their own axis.
- It removes material in image space, so no geometry is left to measure.

The UI follows its conventions: Job combo, operation list with "follows
visibility", play/pause, single step, speed steps 1/5/10/25/50, reset.

## Install

Link or copy this folder into your user Mod directory, for example:

    ln -s /path/to/CAMRotarySim ~/.local/share/FreeCAD/v27-1/Mod/CAMRotarySim

Restart FreeCAD. In the CAM workbench the command **Rotary/Turning Simulator**
appears right after the simulators, in both the CAM menu and the "Tool Commands"
toolbar. Minimum FreeCAD 1.1. Requires numpy, which CAM already uses.

## Use

1. Open a Job with turning operations and start the command.
2. Pick the Job. Turning ops are pre-checked; milling ops are listed but not
   simulated yet.
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
   | Amber | uncut material the tool could have reached |
   | Purple | uncut material this insert can't reach |

6. The report lists the worst values, the ranges, events and path diagnostics:
   - **Events:** rapid moves that cut, holder collisions, parting.
   - **Path diagnostics:** arc radius mismatches, zero feed rates,
     unsupported commands.

## Inputs and conventions

These were verified against source and the sample Job, not assumed:

- **Commands:** the ops' `Path.Commands`, through `getPathWithPlacement`, as
  CAM's simulator reads them. X is a radius. G2/G3 follow the plane
  selection; G18 is the ZX plane, CW/CCW as seen from +Y (Z right, X up).
  IJK are relative to the start point.
- **Imaginary tip:** paths are programmed to the insert's imaginary tip. The
  nose centre is at (X + r, Z + r). This is from LibLathe `nose_compensate` with
  its internal −X frame, which `to_commands(invert_x=True)` flips.
- **Insert outline:** taken from the op's tool solid, sectioned in the XZ plane at
  mid thickness. The control point is the nose arc nearest the shape origin,
  minus (r, r). If there is no usable shape, it is built from the Isoshape
  parameters with the conventions of `isoshape.fcstd`:
  - The shape origin is the sharp corner of the active tip.
  - Edges run at Rotation ± TipAngle/2, measured from +Z toward +X.
  - The insert is a rhombus with side EdgeLength between sharp corners, every
    corner rounded by TipRadius.
  These agree with LibLathe's `get_tool_cutting_angle()`. Both sources give
  the same outline for the sample tool; see `devtools/run_sample_job.py`.
- **Stock and part:** the profile of the Job's stock solid and of its model,
  sectioned at Y = 0. Parts are assumed to be solids of revolution about Z. The
  report shows the part's X offset from the axis if it is more than 1 µm.

## Design

    camrotarysim/core/      pure Python + numpy, no FreeCAD imports
      motion.py             Path commands -> moves (G0/G1/G2/G3, G17/18/19, A/B/C carried)
      tool.py               insert outline, lower/upper envelopes
      stock.py              RotaryStock: R(theta, s) about a RotaryAxis; machine-file axes
      kinematics.py         TurningKinematics; RotaryMillingKinematics (extension point)
      simulation.py         sampling, playback, seek (per-move snapshots), events
      compare.py            part profile, reachable profile, gouge/uncut classification
      mesh.py               revolved mesh with per-vertex colours
      report.py             text report
    camrotarysim/freecad_adapter.py   Job -> core inputs
    camrotarysim/gui/       command + workbench manipulator, task panel, Coin3D scene

- **Turning removal:** each move is sampled at the resolution. At each sample
  the stock is lowered, at every angle, to the insert's lower envelope in every
  axial column it covers.
- **Rotary milling (planned):**
  - Rotate the tool into the workpiece frame by −θ (`to_workpiece`, implemented
    and tested).
  - Lower R only inside the cutter footprint at that angle, with `n_theta > 1`.
  - Rotary axes can come from the Job's machine file (`rotary_axes_from_machine`).
- **Reachable profile:** a 1D morphological opening of the part by the insert's
  lower envelope. It is the best result the insert can produce from outside.
  Material left above it is "unreachable"; anything else left is the path's
  doing.

## Limits

- **Holder and chuck:** not modelled. "Reachable" and the holder-collision check
  use the insert outline only. A collision is reported when stock stands above
  the whole insert over at least 0.25 mm of its axial span. A radial plunge
  clears its own kerf, so plunging deeper than the insert height is not
  reported.
- **Stock shape:** assumed solid to the axis (no bores). The axis must be Z.
- **Diameter mode:** G7/G8 are reported, not applied.
- **Accuracy:** depends on the resolution (column spacing and sample step,
  default 0.05 mm).
- **Parting:** reported when a column reaches the axis with substantial
  material on both sides (both pieces over 1 mm radius). The parted piece is
  kept for display and comparison.

## Tests

    python3 -m unittest discover -s tests

- **Core:** pure tests, plus an end-to-end run of the sample Job (fixtures
  extracted from `turning_test2.FCStd`).
- **FreeCAD side:** `tests/test_adapter.py` and `tests/test_gui_headless.py`
  run the adapter, task panel, Coin scene and command registration. They use
  stub FreeCAD modules (`tests/fakefreecad.py`), a fake Job built from the
  fixtures (`tests/fakejob.py`) and real PySide6 (offscreen). The GUI test is
  skipped without PySide6. These check the addon's logic, not FreeCAD's own
  behaviour.
- **Dev tools:**
  - `devtools/run_sample_job.py --out DIR` prints the report and writes plots.
  - `devtools/make_partoff_guess.py` builds the Partoff fixtures with LibLathe.
