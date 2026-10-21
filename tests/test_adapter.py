# SPDX-License-Identifier: LGPL-2.1-or-later
"""
FreeCAD adapter against stub FreeCAD modules (tests/fakefreecad.py) and a
fake Job built from the sample Job fixtures. Checks the adapter's own logic;
FreeCAD's Part/Path behaviour is only emulated here.
"""

import os
import sys
import types
import unittest

HERE = os.path.dirname(__file__)
sys.path.insert(0, os.path.join(HERE, ".."))
sys.path.insert(0, HERE)

import fakefreecad  # noqa: E402
from fakejob import make_doc, make_thread_doc  # noqa: E402


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


if __name__ == "__main__":
    unittest.main()
