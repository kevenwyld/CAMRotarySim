# SPDX-License-Identifier: LGPL-2.1-or-later
"""
FreeCAD adapter against stub FreeCAD modules (tests/fakefreecad.py) and a
fake Job built from the sample Job fixtures. Checks the adapter's own logic;
FreeCAD's Part/Path behaviour is only emulated here.
"""

import dataclasses
import os
import sys
import types
import unittest

import numpy as np

HERE = os.path.dirname(__file__)
sys.path.insert(0, os.path.join(HERE, ".."))
sys.path.insert(0, HERE)

import fakefreecad  # noqa: E402
from fakejob import make_boring_doc, make_doc, make_rotary_doc, make_thread_doc  # noqa: E402


class TestAdapter(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        fakefreecad.install()
        sys.modules.pop("camrotarysim.freecad_adapter", None)
        import camrotarysim.freecad_adapter as fa
        cls.fa = fa

    def test_job_and_ops(self):
        doc, job, _stock, _model = make_doc(fakefreecad)
        fa = self.fa
        self.assertEqual(fa.find_jobs(doc), [job])
        ops = fa.active_operations(job)
        self.assertEqual([fa.is_turning_op(o) for o in ops], [True, True, True, False])
        cmds = fa.op_commands(ops[0])
        self.assertEqual(cmds[0], ("G18", {}))

    def test_build_inputs(self):
        doc, job, _stock, _model = make_doc(fakefreecad)
        fa = self.fa
        ops = [o for o in fa.active_operations(job) if fa.is_turning_op(o)]
        stock, inputs, polylines, notes = fa.build_inputs(job, ops, 0.05)
        self.assertAlmostEqual(stock.R.max(), 25.73942, 4)
        self.assertAlmostEqual(stock.s_min, -66.3919, 3)
        self.assertEqual([i.name for i in inputs], ["TurnFace", "TurnRough", "TurnProfile"])
        # one outline shared by the three ops, built from the tool solid (sectioned at mid thickness)
        self.assertIs(inputs[0].tool, inputs[2].tool)
        self.assertEqual(job.Operations.OutList[0].ToolController.Tool.Shape.slices, [])  # copy was sliced
        self.assertIn("tool shape of Lathe Iso Insert", notes[0])
        cx, cz = inputs[0].tool.control_in_shape
        self.assertAlmostEqual(cx, 0.27094, 4)
        self.assertAlmostEqual(cz, 0.03571, 4)
        self.assertTrue(polylines)

    def test_tool_outline_fallback_to_parameters(self):
        tool = types.SimpleNamespace(Label="T", Shape=None, TipRadius=0.4, EdgeLength=5.0, TipAngle=60.0,
                                     Rotation=57.0)
        outline, note = self.fa.tool_outline(tool)
        self.assertIn("Isoshape parameters", note)
        self.assertAlmostEqual(outline.nose_centre[0], 0.4, 6)

    def test_no_outline_possible(self):
        with self.assertRaises(ValueError):
            self.fa.tool_outline(types.SimpleNamespace(Label="EM", Shape=None))

    def test_thread_job_inputs(self):
        doc, job, op, tc = make_thread_doc(fakefreecad)
        fa = self.fa
        self.assertTrue(fa.is_turning_op(op))
        self.assertEqual(fa.tool_reference(tc.Tool), "origin")
        self.assertEqual(fa.tool_reference(make_doc(fakefreecad)[1].Operations.OutList[0].ToolController.Tool),
                         "imaginary_tip")
        stock, inputs, polylines, notes = fa.build_inputs(job, [op], 0.05)
        t = inputs[0].tool
        # threading tools are programmed to the V's sharp point: the shape origin
        self.assertEqual(t.control_in_shape, (0.0, 0.0))
        self.assertAlmostEqual(t.lower(0.0), 0.05, 4)  # the tip arc's bottom is a tip radius above it
        self.assertEqual(inputs[0].spindle, 1)
        self.assertAlmostEqual(inputs[0].meta["ThreadDepth"], 1.12933, 4)
        self.assertIn("Pitch diameter 10.679-10.829", inputs[0].meta["ThreadReport"])
        tc.SpindleDir = "Reverse"
        self.assertEqual(fa.spindle_sense(op), -1)
        self.assertEqual(fa.spindle_sense(op, forward_right_hand=False), 1)
        tc.SpindleDir = "Forward"

    def test_threading_fallback_to_parameters(self):
        q = lambda v: types.SimpleNamespace(Value=v)  # noqa: E731
        tool = types.SimpleNamespace(Label="Th", Shape=None, ShapeID="threading", ProfileAngle=q(60.0),
                                     TipRadius=q(0.05), Width=q(6.0))
        outline, note = self.fa.tool_outline(tool)
        self.assertIn("Threading parameters", note)
        self.assertEqual(outline.control_in_shape, (0.0, 0.0))

    def test_internal_ops_and_starting_hole(self):
        doc, job, ops = make_boring_doc(fakefreecad)
        fa = self.fa
        self.assertEqual([fa.is_internal(o) for o in ops], [False, True, False, True])
        self.assertEqual(fa.hole_from_ops(ops), (5.0, "TurnProfile"))
        stock, inputs, polylines, notes = fa.build_inputs(job, ops, 0.05)
        self.assertEqual([i.internal for i in inputs], [False, True, False, True])
        # auto: Min Diameter 10, through, as the model's bore runs out at its back end
        self.assertEqual(stock.hole, (10.0, 113.0))
        self.assertAlmostEqual(stock.Rin[0, 10], 5.0)
        self.assertTrue(any("starting hole 10.000 from TurnProfile's Min Diameter, through" in n for n in notes))
        stock, *_ = fa.build_inputs(job, ops, 0.05, hole=None)
        self.assertFalse(stock.has_bore())
        stock, *_ = fa.build_inputs(job, ops, 0.05, hole=(8.0, 40.0))
        self.assertEqual(stock.hole, (8.0, 40.0))
        self.assertEqual(stock.Rin[0, stock.column_range(-39.9, -39.9)[0]], 4.0)
        self.assertEqual(stock.Rin[0, stock.column_range(-40.1, -40.1)[0]], 0.0)

    def test_thread_op_answers_through_internal(self):
        # Turn Thread inherits is_internal() (False: no Side) and answers through internal()
        doc, job, ops = make_boring_doc(fakefreecad)
        thread = ops[3]
        self.assertFalse(thread.Proxy.is_internal(thread))
        self.assertTrue(thread.Proxy.internal(thread))
        self.assertTrue(self.fa.is_internal(thread))
        # an external Rough answers False through is_internal(), and its Side isn't consulted again
        self.assertFalse(self.fa.is_internal(ops[2]))

    def test_drill_op(self):
        doc, job, ops = make_boring_doc(fakefreecad, with_drill=True)
        fa = self.fa
        drill = ops[1]
        self.assertTrue(fa.is_drill_op(drill))
        self.assertTrue(fa.is_internal(drill))
        self.assertEqual(fa.tool_reference(drill.ToolController.Tool), "origin")
        stock, inputs, polylines, notes = fa.build_inputs(job, ops, 0.05)
        # Auto: a drilling op is selected, so no starting hole
        self.assertIsNone(stock.hole)
        self.assertFalse(stock.has_bore())
        self.assertTrue(any("no starting hole: drilled by TurnDrill" in n for n in notes))
        self.assertTrue(any("10mm Drill: outline from Drill parameters" in n for n in notes))
        t = inputs[1].tool
        self.assertEqual(t.control_in_shape, (0.0, 0.0))
        self.assertAlmostEqual(t.xmax, 5.0)
        # a CAM drill on an op without the Turn Drill proxy still counts as drilling
        other = types.SimpleNamespace(Proxy=object(), ToolController=drill.ToolController)
        self.assertTrue(fa.is_drill_op(other))
        self.assertFalse(fa.is_drill_op(ops[2]))

    def test_milling_ops_and_frames(self):
        import os
        fakefreecad.sys.modules["Machine.models.machine"].MachineFactory.files = {
            "tormach 4-axis": os.path.join(HERE, "data", "rotary", "tormach_4-axis.fcm")}
        doc, job, ops = make_rotary_doc(fakefreecad)
        fa = self.fa
        self.assertTrue(all(fa.is_milling_op(o) for o in ops))
        self.assertFalse(any(fa.is_turning_op(o) for o in ops))
        letter, axis = fa.machine_rotary_axis(job)
        self.assertEqual((letter, axis.direction, axis.limits), ("A", (1.0, 0.0, 0.0), (-120.0, 120.0)))
        stock, inputs, part_phi, notes, info, tools = fa.build_milling_inputs(job, ops, 1.0)
        self.assertEqual([round(i.meta["angle"], 6) for i in inputs if not i.meta.get("indexing")], [-90.0, 0.0, 90.0])
        # Auto: the machine's pre-rotary move (G53 G0 Z-40, machine coordinates) as a retract clearing the swing
        names = [i.name for i in inputs][:3]
        self.assertEqual(names, ["Adaptive (index A +0.0 to -90.0, retracted)", "Adaptive",
                                 "Adaptive001 (index A -90.0 to +0.0, retracted)"])
        self.assertTrue(any("pre-rotary move 'G53 G0 Z-40' is in machine coordinates" in n for n in notes))
        self.assertGreater(stock.retract, 35.355)
        self.assertEqual(len(tools), len(inputs))
        _s, inputs_none, *_ = fa.build_milling_inputs(job, ops, 1.0, retract=None)
        self.assertEqual(len(inputs_none), 5)       # no initial index without a known tool position
        self.assertIn("Adaptive002: (0.000, 1.000, -0.000), A +90.0", info[2])
        self.assertTrue(any("rotary axis A from machine 'tormach 4-axis'" in n for n in notes))
        self.assertTrue(any("Endmill: flat end, diameter 5.000, cutting length 30.000" in n for n in notes))
        self.assertEqual(part_phi.shape, stock.shape)
        # the ops' own commands, local to their work plane: the Adaptive helix starts at local (0, -1.446)
        tip, d = inputs[1].kinematics.to_workpiece((0.0, -1.44625, 5.0, 0, 0, 0))
        np.testing.assert_allclose(d, [0, -1, 0], atol=1e-9)
        np.testing.assert_allclose(tip, [25.0, -30.0, -1.51050519], atol=1e-6)
        # without the machine: A along X assumed
        fakefreecad.sys.modules["Machine.models.machine"].MachineFactory.files = {}
        self.assertIsNone(fa.machine_rotary_axis(job))

    def test_tool_with_holder(self):
        # holders drawn in the tool's solid (CAM Lathe Turning before 0.18.1): the insert hangs under the
        # rake face, the holder on it; the section at +0.01 is the holder's (no nose), so -0.01 is used
        import json
        with open(os.path.join(HERE, "data", "holders", "tools.json")) as f:
            d = json.load(f)
        p = d["params"]
        t = d["turning"]
        turning = self._iso_tool(t["bbox"], [((-1.0, 0.0), t["insert"]), ((0.0, 12.0), t["holder"])],
                                 "iso turning insert 4", p)
        outline, note = self.fa.tool_outline(turning)
        self.assertIn("an older tool model, cutting plane at Y -0.01", note)
        np.testing.assert_allclose(outline.control_in_shape, (0.0, 0.0), atol=1e-9)   # origin = imaginary tip
        self.assertAlmostEqual(outline.nose_radius, 0.4, 6)
        self.assertLess(outline.xmax, 10.0)                          # the insert, not the 80 mm holder
        self.assertEqual(turning.Shape.slices, [])                   # a copy was sliced
        # the boring bar spans both planes: at +0.01 only the bar, at -0.01 insert and bar
        b = d["boring"]
        boring = self._iso_tool(b["bbox"], [((-1.0, 0.0), b["cutting_plane"]), ((-6.0, -1.0), b["bar_only"]),
                                            ((0.0, 4.0), b["bar_only"])], "iso turning insert boring 4", p)
        outline, note = self.fa.tool_outline(boring)
        self.assertIn("older tool model", note)
        np.testing.assert_allclose(outline.control_in_shape, (0.0, 0.0), atol=1e-9)
        self.assertAlmostEqual(outline.nose_radius, 0.4, 6)
        # neither plane has the nose (only holder there): the parameters, and the note says why
        only_holder = self._iso_tool(t["bbox"], [((-1.0, 12.0), t["holder"])], "holder only", p)
        outline, note = self.fa.tool_outline(only_holder)
        self.assertIn("section at Y 0.01 or -0.01 has no nose of radius 0.4", note)
        self.assertIn("using Isoshape parameters", note)

    def test_tool_models_from_0_18_1(self):
        # the rake face is Y 0 with insert and holder on +Y: the section at +0.01, no note
        import json
        with open(os.path.join(HERE, "data", "holders", "tools.json")) as f:
            d = json.load(f)
        r = d["rake_plane"]
        iso = self._iso_tool(r["bbox"], [((0.0, 1.0), r["insert"])], "isoshape 0.18.1", r["params"])
        outline, note = self.fa.tool_outline(iso)
        self.assertEqual(note, "")
        np.testing.assert_allclose(outline.control_in_shape, (0.0, 0.0), atol=1e-9)
        self.assertAlmostEqual(outline.nose_radius, 0.4, 6)
        self.assertEqual(self.fa.cutting_planes(iso.Shape.BoundBox), [0.01])   # nothing under the rake face
        # with a holder on top of the insert, the insert is still what +0.01 cuts
        t = d["turning"]
        held = self._iso_tool((0.0, 0.0, 0.0, 80.0, 13.0, 16.0),
                              [((0.0, 1.0), t["insert"]), ((1.0, 13.0), t["holder"])],
                              "iso insert in a holder, 0.18.1", d["params"])
        outline, note = self.fa.tool_outline(held)
        self.assertEqual(note, "")
        self.assertLess(outline.xmax, 10.0)
        # a 3 mm threading insert of any version spans +0.01 (before 0.17.4: Y -1.5 to 1.5; now 0 to 3)
        q = lambda v: types.SimpleNamespace(Value=v)  # noqa: E731
        with open(os.path.join(HERE, "data", "thread", "tool_threading_section.json")) as f:
            th = json.load(f)["edges"]
        for ys in ((-1.5, 1.5), (0.0, 3.0)):
            bb = fakefreecad.BoundBox(0.05, ys[0], -3.0, 11.2, ys[1], 3.0)
            tool = types.SimpleNamespace(Label="threading", ShapeID="threading", ProfileAngle=q(60.0),
                                         TipRadius=q(0.05), Width=q(6.0),
                                         Shape=fakefreecad.Shape([], bb, sections=[(ys, th)]))
            outline, note = self.fa.tool_outline(tool)
            self.assertEqual(note, "")
            self.assertEqual(outline.control_in_shape, (0.0, 0.0))
            self.assertEqual(tool.Shape.slices, [])

    def test_tool_models_with_a_y_offset_in_the_placement(self):
        # made with 0.17.4 to 0.18.0 (or rebuilt from those shapes): the geometry as before 0.17.4 (insert
        # under Y 0), the Placement Y +1 puts the rake face at Y 0; FreeCAD shows it there, and so must the
        # simulator, so the offset is kept (real_turning_test2_1.FCStd's VBMT in its holder)
        import json
        with open(os.path.join(HERE, "data", "holders", "tools.json")) as f:
            v = json.load(f)["placed_0_18_0"]
        tool = self._iso_tool(v["raw_bbox"], [((-1.0, 0.0), v["insert"]), ((0.0, 12.7), v["holder"])],
                              "VBMT 16 04 04 in a 3deg holder", v["params"], placement_y=v["placement_y"])
        tool.Shape.mesh = ([(0.0, -1.0, 0.0), (1.0, 12.7, 0.0), (0.0, 0.0, 1.0)], [(0, 1, 2)])
        shape, y = self.fa.turning_tool_shape(tool)
        self.assertEqual(y, 1.0)
        self.assertAlmostEqual(shape.BoundBox.YMin, 0.0)                 # the rake face at Y 0, as shown
        pts, _tris = shape.tessellate(0.02)                               # what the 3D view draws
        self.assertEqual([p.y for p in pts], [0.0, 13.7, 1.0])
        outline, note = self.fa.tool_outline(tool)
        self.assertIn("an older tool model, its rake face set at Y 0 by a Y +1 Placement", note)
        self.assertAlmostEqual(outline.nose_radius, 0.4, 6)
        self.assertLess(outline.xmax, 40.0)      # the 16 mm insert (27 in X), not the 80 mm holder
        self.assertEqual(tool.Shape.slices, [])                           # a copy was sliced
        # X, Z and rotation in a tool's Placement are not applied: the origin stays the programmed point
        tool.Shape.Placement.Base.x = 25.0
        shape, y = self.fa.turning_tool_shape(tool)
        self.assertEqual((shape.Placement.Base.x, shape.Placement.Base.y), (0.0, 1.0))

    def _iso_tool(self, bbox, sections, label, p, placement_y=0.0):
        q = lambda v: types.SimpleNamespace(Value=v)  # noqa: E731
        return types.SimpleNamespace(Label=label, ShapeID="isoshape",
                                     Shape=fakefreecad.Shape([], fakefreecad.BoundBox(*bbox), sections=sections,
                                                             placement_y=placement_y),
                                     TipRadius=q(p["TipRadius"]), EdgeLength=q(p["EdgeLength"]),
                                     TipAngle=q(p["TipAngle"]), Rotation=q(p["Rotation"]))

    def test_side_property_fallback(self):
        op = types.SimpleNamespace(Proxy=object(), Side="Internal")
        self.assertTrue(self.fa.is_internal(op))
        self.assertFalse(self.fa.is_internal(types.SimpleNamespace(Proxy=object())))


class TestThreadData(unittest.TestCase):
    """The thread check's expected diameters: Thread*Dia properties, else the metric or inch ThreadReport."""

    INCH_REPORT = ('UNC 1/2-13 external, class 2A\n'
                   'Turn the major diameter to 0.4876"-0.4985" first: the selected 0.4725" is 0.0151" under\n'
                   'Pitch diameter 0.4435"-0.4485", cut to 0.4460"\n'
                   'Root diameter 0.3836", 0.0444" deep from the selected diameter')

    @classmethod
    def setUpClass(cls):
        fakefreecad.install()
        sys.modules.pop("camrotarysim.freecad_adapter", None)
        import camrotarysim.freecad_adapter as fa
        from camrotarysim.core import thread_check
        from camrotarysim.core.simulation import Simulation
        cls.fa, cls.T = fa, thread_check
        _doc, job, cls.op, _tc = make_thread_doc(fakefreecad)
        stock, inputs, _pl, _notes = fa.build_inputs(job, [cls.op], 0.05)
        sim = Simulation(stock, inputs)
        sim.run_to_end()
        cls.stock, cls.zone = stock, cls.T.thread_zones(sim)[0]

    def measure(self, **props):
        op = types.SimpleNamespace(**dict(vars(self.op), **props))
        return self.T.measure(self.stock, dataclasses.replace(self.zone, meta=self.fa.op_meta(op)), 3.0)

    def test_properties_present(self):
        props = dict(ThreadPitchDiaMin=11.2, ThreadPitchDiaMax=11.3, ThreadMajorDiaMin=11.95,
                     ThreadMajorDiaMax=12.05, ThreadMinorDiaMin=0.0, ThreadMinorDiaMax=0.0,
                     ThreadRootDia=types.SimpleNamespace(Value=9.5))
        r = self.measure(ThreadReport=self.INCH_REPORT, **props)  # the properties are mm whatever the report
        self.assertAlmostEqual(r.pitch_diameter, 11.259, delta=0.01)
        self.assertEqual(r.expected_pitch, (11.2, 11.3))     # not the report's 10.679-10.829
        self.assertEqual(r.expected_major, (11.95, 12.05))
        self.assertIsNone(r.expected_minor)
        self.assertEqual(r.verdict, "ok")
        self.assertEqual(r.expected_root, 9.5)
        self.assertTrue(any("deepest pass reaches diameter 9.744; the op's root is 9.500" in n for n in r.notes))
        self.assertFalse(any("crests" in n for n in r.notes))
        # 0 everywhere: the class has no data, so no verdict and no fallback to the report
        r = self.measure(**{k: 0.0 for k in self.fa.THREAD_DIAMETERS})
        self.assertEqual((r.expected_pitch, r.verdict), (None, ""))
        self.assertAlmostEqual(r.expected_root, 9.744, 3)    # ThreadDiameter - 2 ThreadDepth

    def test_properties_absent_metric_report(self):
        r = self.measure()
        self.assertNotIn("ThreadPitchDiaMin", self.fa.op_meta(self.op))
        self.assertEqual(r.expected_pitch, (10.679, 10.829))
        self.assertEqual(r.expected_major, (11.701, 11.966))
        self.assertEqual(r.verdict, "shallow")
        self.assertNotIn("inch report: not compared", r.notes)

    def test_properties_absent_inch_report(self):
        r = self.measure(ThreadReport=self.INCH_REPORT, ThreadStandard="UNC")
        self.assertIn("inch report: not compared", r.notes)
        self.assertEqual((r.expected_pitch, r.expected_major, r.expected_minor), (None, None, None))
        self.assertEqual(r.verdict, "")
        self.assertIsNotNone(r.pitch_diameter)               # still measured, just not judged
        self.assertTrue(any("inch report" in line for line in self.T.format_thread(r)))


if __name__ == "__main__":
    unittest.main()
