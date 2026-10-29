# CAM Rotary Simulator

| :warning: WARNING: This code is AI generated, use extreme caution.  Meant as a proof of concept only. |

A proof-of-concept FreeCAD addon that simulates material removal and checks the
result against the Job's model: the turning, drilling, boring and threading ops of
[CAM Lathe Turning](https://github.com/kevenwyld/CAMLatheTurning) on a rotary height
field, and CAM milling ops, indexed or rotary (4th axis), on a signed-distance grid.

It is standalone because CAM's own simulator (`CAMSimulator`) cannot be extended:
- it parses only `G T X Y Z I J K R`, ignores plane selection and treats G33 as a linear move;
- its tools must be solids of revolution about their own axis;
- it removes material in image space, so no geometry is left to measure.

The UI follows CAM's simulator: Job combo, operation list, play/pause, single step, a
slider and speed steps, plus a reset and x0.25 / x0.5 for short paths.

https://github.com/user-attachments/assets/ad96ca03-bc3d-4b20-8266-5192bdd13e37

https://github.com/user-attachments/assets/3a9bca94-52dd-4211-a65e-e46daf2ec9d1

## Install

Link or copy this folder into your user Mod directory, for example:

    ln -s /path/to/CAMRotarySim ~/.local/share/FreeCAD/v27-1/Mod/CAMRotarySim

Restart FreeCAD. In the CAM workbench **Rotary/Turning Simulator** appears right after
the simulators in the CAM menu and the "Tool Commands" toolbar. Minimum FreeCAD 1.1;
requires numpy, which CAM already uses.

## Use

1. Open a Job with turning or milling operations and start the command.
2. Pick the Job. Turning and milling ops are pre-checked; select one kind, as they
   cannot be simulated together yet.
3. Press play. The stock is drawn in the 3D view; the Job's stock and model are hidden
   while the panel is open (untick the option to keep them).
4. Controls: slider to scrub forwards or backwards; Step runs to the end of the current
   move; - / + change the speed; Reset goes back to the start.

Colours on the stock:

| Colour | Meaning |
|---|---|
| Blue | untouched stock |
| Green | cut surface |
| Red | gouge (below the part by more than the gouge tolerance) |
| Amber | uncut material the tool could have reached (milling: any material left beyond the part) |
| Purple | uncut material this insert can't reach (turning only) |

Threads (G33 passes or a G76 cycle):
- cut along their helix; while a pass runs the displayed workpiece turns with the spindle;
- in the thread's Z range a thread measurement replaces the gouge check: green when the
  pitch diameter is in the op's range, amber when not deep enough, red when too deep,
  light blue until measured (when playback pauses or ends);
- **Thread angles** sets the angular resolution; **Forward spindle cuts** sets the hand
  a Forward spindle cuts when feeding toward -Z (Reverse cuts the other one).

Internal ops (drilling, boring with Side = Internal, internal threads) are simulated
inside. **Starting hole** sets the hole they begin from when nothing selected drills it:
- **Auto:** none if a drilling op is selected; otherwise the smallest Min Diameter set on
  an internal op, down the model's bore or through the stock;
- **None**, or **Custom:** a diameter and a depth from the stock's front.

A stock that already has a bore keeps it. The bore is drawn and checked like the outside.

The report lists:
- the worst values and ranges, the bore, threads and events (rapid moves that cut,
  holder collisions, parting, a rotary index that sweeps the stock into the tool);
- path diagnostics: arc radius mismatches, zero feed rates, unsupported commands.

## Milling (indexed and rotary)

CAM milling ops are simulated in the workpiece frame on a stock that can be cut from
any direction. Each op's commands stay local to its work plane and are placed by the
op's Placement, as CAM's post reads them; a rotary word in a command turns the workpiece.

- **Rotary angle:** the report gives, per op, the angle that turns its work plane's
  normal to +Z; the 3D view turns the part by it. Compare with the posted program.
- **Indexing:** between ops at different angles the simulator runs the pre-rotary move,
  the rotary axis turning and the next op from where the tool is. **Retract before
  indexing:** Auto (the machine's pre-rotary move; a G53 move is shown as a retract to
  the stock's swing radius plus the SetupSheet clearance), None (turn in place; a stock
  that sweeps through the tool is cut and reported) or Custom (a Z).
- **Rotary axis:** from the Job's machine through CAM's `MachineFactory`; without one,
  A along X through the origin.
- **Stock:** a signed-distance grid (**Milling grid**, 0.5 mm by default). Each cutter
  position is subtracted exactly; drawn edges are chamfered by up to a third of the grid.
- **Check:** gouge where the model has material; left beyond the model, exact up to
  three grid spacings.
- **Tools:** flat and ball end mills (ShapeID `ballend`); other shapes are treated as
  flat and reported. The cutting length is CuttingEdgeHeight.
- **Display:** "Part outline" shows the model as a see-through mesh.

The conventions behind all of this are in [docs/DESIGN.md](docs/DESIGN.md); what the simulator
assumes about CAM Lathe Turning ops and tools is in [CONTRACT.md](CONTRACT.md).

## Limits

- Holder, boring bar and chuck are not modelled; "reachable" and the collision check
  use the insert outline only. A radial plunge clears its own kerf.
- Turning stock is an outside and a bore per angle and position: no undercut pockets
  in a wall. The axis must be Z.
- Canned cycles G85 to G89 and cycles in G91 are reported, not simulated. G7/G8 are
  reported, not applied.
- Threads: the tool is a flat profile in the XZ plane (no lead angle); the hand is a setting.
- Milling: turning and milling ops cannot be simulated together; shanks and holders are
  not checked; cutters other than flat and ball are approximated; continuous rotary
  moves are untested on a real op.
- Accuracy follows the Resolution (turning, 0.05 mm default) and Milling grid settings.
- Parting is reported when a column reaches the axis with over 1 mm of material on
  both sides; the parted piece is kept for display and comparison.

## Tests

    python3 -m unittest discover -s tests

Core tests are pure; `tests/test_adapter.py` and `tests/test_gui_headless.py` run the
adapter, task panel and Coin scene against stub FreeCAD modules (`tests/fakefreecad.py`),
a fake Job built from fixtures and PySide6 offscreen (skipped without PySide6).

Dev tools (`devtools/`): `run_sample_job.py`, `run_rotary_job.py` and `run_thread_job.py`
run the fixture Jobs and write reports and plots; `make_partoff_guess.py`,
`make_drill_fixture.py` and `make_thread_g76_fixture.py` build the fixtures.
