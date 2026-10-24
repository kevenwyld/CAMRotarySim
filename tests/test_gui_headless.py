# SPDX-License-Identifier: LGPL-2.1-or-later
"""
Headless test of the FreeCAD adapter, task panel, Coin scene and command
registration, using stub FreeCAD modules (tests/fakefreecad.py) fed with
the sample Job's geometry, and real PySide6 (offscreen). Skipped without
PySide6.
"""

import importlib
import math

import numpy as np
import os
import sys
import unittest

HERE = os.path.dirname(__file__)
DATA = os.path.join(HERE, "data")
ROOT = os.path.join(HERE, "..")
sys.path.insert(0, ROOT)
sys.path.insert(0, HERE)
os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

try:
    import PySide6  # noqa: F401
    HAVE_QT = True
except ImportError:
    HAVE_QT = False

from fakejob import make_boring_doc, make_doc, make_rotary_doc, make_thread_doc  # noqa: E402


_APP = None


@unittest.skipUnless(HAVE_QT, "PySide6 not installed")
class TestHeadlessGui(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        import fakefreecad
        cls.fake = fakefreecad
        cls.fc, cls.gui = fakefreecad.install()
        from PySide6 import QtWidgets
        global _APP  # keep the application alive until interpreter exit
        _APP = cls.app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
        for m in [m for m in sys.modules if m.startswith("camrotarysim.gui") or m == "camrotarysim.freecad_adapter"]:
            del sys.modules[m]

    def test_command_and_manipulator(self):
        if "InitGui" in sys.modules:
            del sys.modules["InitGui"]
        importlib.import_module("InitGui")
        self.assertIn("CAM_RotarySim", self.gui.commands)
        self.assertTrue(self.gui.icon_paths[-1].endswith(os.path.join("Resources", "icons")))
        self.assertTrue(os.path.exists(os.path.join(self.gui.icon_paths[-1], "CAM_RotarySim.svg")))
        m = self.gui.manipulators[-1]
        self.assertEqual(m.modifyMenuBar(), [{"insert": "CAM_RotarySim", "menuItem": "CAM_Inspect"}])
        self.assertEqual(m.modifyToolBars(), [{"insert": "CAM_RotarySim", "toolItem": "CAM_Inspect"}])
        res = self.gui.commands["CAM_RotarySim"].GetResources()
        self.assertEqual(res["Pixmap"], "CAM_RotarySim")

    def test_task_panel_full_run(self):
        doc, job, stock, model = make_doc(self.fake)
        self.fc.ActiveDocument = doc
        from camrotarysim.gui.taskpanel import TaskPanel
        p = TaskPanel(doc)
        # turning ops checked, the milling op listed but unchecked
        states = [p.list_ops.item(i).checkState().name for i in range(p.list_ops.count())]
        self.assertEqual(states, ["Checked", "Checked", "Checked", "Unchecked"])
        p.on_play()
        self.assertIsNotNone(p.sim, p.report.toPlainText())
        self.assertTrue(p.timer.isActive())
        self.assertFalse(stock.ViewObject.Visibility)   # hidden while simulating
        self.assertFalse(model.ViewObject.Visibility)
        n = 0
        while p.timer.isActive() and n < 100000:
            p.on_tick()
            n += 1
        self.assertTrue(p.sim.done)
        text = p.report.toPlainText()
        self.assertIn("Worst gouge: -0.196 mm", text)
        self.assertIn("outline from tool shape of Lathe Iso Insert", text)
        self.assertIn("G1 with zero or missing feed rate", text)
        # the stock mesh reached the scene, with one colour per vertex
        mesh_coords = p.scene.stock_coords.point.values
        self.assertGreater(len(mesh_coords), 1000)
        self.assertEqual(len(p.scene.stock_mat.diffuseColor.values), len(mesh_coords))
        idx = p.scene.stock_faces.coordIndex.values
        self.assertEqual(max(idx), len(mesh_coords) - 1)
        # tool placed so its control point is at the tool position
        tx, ty, tz = p.scene.tool_xf.translation.value
        cx, cz = p.sim.ops[0].tool.control_in_shape
        self.assertAlmostEqual(tx + cx, p.sim.position[0], 9)
        self.assertAlmostEqual(tz + cz, p.sim.position[2], 9)
        # scrub back, step, reset
        p.on_slider(500)
        self.assertAlmostEqual(p.sim.progress, 0.5, 2)
        before = p.sim.cursor
        p.on_step()
        self.assertGreater(p.sim.cursor, before)
        p.on_reset()
        self.assertEqual(p.sim.cursor, 0)
        p.change_speed(10)
        self.assertEqual(p.lbl_speed.text(), "x50")
        # changing the resolution forces a rebuild
        p.spin_res.setValue(0.1)
        self.assertTrue(p.dirty)
        # close restores visibility and removes the scene
        root = self.gui.ActiveDocument.ActiveView.getSceneGraph()
        self.assertIn(p.scene.root, root.children)
        scene_root = p.scene.root
        p.reject()
        self.assertTrue(stock.ViewObject.Visibility)
        self.assertTrue(model.ViewObject.Visibility)
        self.assertNotIn(scene_root, root.children)
        p.timer.deleteLater()
        p.form.deleteLater()
        self.app.processEvents()

    def test_task_panel_thread_job(self):
        doc, job, op, tc = make_thread_doc(self.fake)
        self.fc.ActiveDocument = doc
        from camrotarysim.gui.taskpanel import TaskPanel
        p = TaskPanel(doc)
        self.assertEqual(p.list_ops.item(0).checkState().name, "Checked")
        p.on_play()
        self.assertIsNotNone(p.sim, p.report.toPlainText())
        angles = []
        n = 0
        while p.timer.isActive() and n < 100000:
            p.on_tick()
            angles.append(p.scene.stock_rot.rotation.value)
            n += 1
        self.assertTrue(p.sim.done)
        # the displayed workpiece turned with the spindle during the passes
        self.assertGreater(len({a[1] for a in angles if a}), 10)
        text = p.report.toPlainText()
        self.assertIn("pitch 11.259", text)
        self.assertIn("not cut deep enough", text)
        self.assertIn("right-hand for a Forward spindle", text)
        self.assertIn("outline from tool shape of threading", text)
        self.assertGreater(len(p.scene.stock_coords.point.values), 10000)
        # changing the hand or the angles forces a rebuild
        p.combo_hand.setCurrentIndex(1)
        self.assertTrue(p.dirty)
        p.reject()
        p.timer.deleteLater()
        p.form.deleteLater()
        self.app.processEvents()

    def test_task_panel_boring_job(self):
        doc, job, ops = make_boring_doc(self.fake)
        self.fc.ActiveDocument = doc
        from camrotarysim.gui.taskpanel import TaskPanel
        p = TaskPanel(doc)
        self.assertEqual(p.combo_hole.currentIndex(), 0)       # Auto
        self.assertFalse(p.spin_hole_d.isEnabled())
        p.on_play()
        self.assertIsNotNone(p.sim, p.report.toPlainText())
        # Auto found the hole: shown in the (greyed) fields
        self.assertAlmostEqual(p.spin_hole_d.value(), 10.0)
        self.assertAlmostEqual(p.spin_hole_depth.value(), 113.0)
        # the internal op's tool is drawn mirrored about its control point
        v_ext, v_int = p.tool_meshes[0][0], p.tool_meshes[1][0]
        np.testing.assert_allclose(v_int[:, 0], -v_ext[:, 0])
        n = 0
        while p.timer.isActive() and n < 200000:
            p.on_tick()
            n += 1
        self.assertTrue(p.sim.done)
        text = p.report.toPlainText()
        self.assertIn("Bore (positive", text)
        self.assertIn("TurnThread: internal", text)
        self.assertIn("not cut deep enough", text)
        self.assertIn("starting hole 10.000 from TurnProfile's Min Diameter, through", text)
        self.assertIn("Events: none", text)
        # a custom hole: the fields come alive and the simulation is rebuilt
        p.combo_hole.setCurrentIndex(2)
        self.assertTrue(p.spin_hole_d.isEnabled())
        self.assertTrue(p.dirty)
        p.spin_hole_d.setValue(0.0)    # no hole: boring hits the bar
        p.on_play()
        while p.timer.isActive():
            p.on_tick()
        self.assertIn("boring bar would collide", p.report.toPlainText())
        p.reject()
        p.timer.deleteLater()
        p.form.deleteLater()
        self.app.processEvents()

    def test_task_panel_drilled_boring_job(self):
        doc, job, ops = make_boring_doc(self.fake, with_drill=True)
        self.fc.ActiveDocument = doc
        from camrotarysim.gui.taskpanel import TaskPanel
        p = TaskPanel(doc)
        self.assertEqual([p.list_ops.item(i).checkState().name for i in range(p.list_ops.count())],
                         ["Checked"] * 5)
        p.on_play()
        self.assertIsNotNone(p.sim, p.report.toPlainText())
        self.assertEqual(p.spin_hole_d.value(), 0.0)    # Auto: no hole, the drill makes it
        while p.timer.isActive():
            p.on_tick()
        text = p.report.toPlainText()
        self.assertIn("no starting hole: drilled by TurnDrill", text)
        self.assertIn("Events: none", text)
        self.assertIn("pitch 19.949", text)
        p.reject()
        p.timer.deleteLater()
        p.form.deleteLater()
        self.app.processEvents()

    def test_task_panel_rotary_job(self):
        self.fake.sys.modules["Machine.models.machine"].MachineFactory.files = {
            "tormach 4-axis": os.path.join(HERE, "data", "rotary", "tormach_4-axis.fcm")}
        doc, job, ops = make_rotary_doc(self.fake)
        self.fc.ActiveDocument = doc
        from camrotarysim.gui.taskpanel import TaskPanel
        p = TaskPanel(doc)
        self.assertEqual([p.list_ops.item(i).checkState().name for i in range(p.list_ops.count())],
                         ["Checked"] * 3)
        p.spin_grid.setValue(1.0)          # coarse, for speed
        from camrotarysim.gui.taskpanel import SPEEDS
        p.change_speed(-10)
        p.change_speed(SPEEDS.index(5))    # x5: rotary moves at 15 degrees per tick
        self.assertEqual(p.lbl_speed.text(), "x5")
        p.on_play()
        self.assertTrue(p.milling)
        self.assertIsNotNone(p.sim, p.report.toPlainText())
        angles = set()
        while p.timer.isActive():
            p.on_tick()
            angles.add(round(p.milling_angle()[0], 6))
        self.assertTrue(p.sim.done)
        # the workpiece turned to each work plane, through the angles between
        self.assertTrue({-90.0, 0.0, 90.0} <= angles)
        self.assertTrue(any(-60 < a < -30 for a in angles) and any(30 < a < 60 for a in angles))
        self.assertGreater(len(angles), 12)
        # a tick never jumps across a rotation
        self.assertTrue(all(abs(b - a) <= 15.0 + 1e-6 for a, b in zip(sorted(angles), sorted(angles)[1:])))
        self.assertGreater(p.spin_retract.value(), 35.355)     # Auto shows the retract it used
        rot = p.scene.work_xf.rotation.value
        self.assertAlmostEqual(rot[1], math.radians(90.0))   # last op: +Y face, A +90
        text = p.report.toPlainText()
        self.assertIn("Adaptive002: (0.000, 1.000, -0.000), A +90.0", text)
        self.assertIn("Worst gouge: none", text)
        self.assertIn("rotary axis A from machine 'tormach 4-axis'", text)
        self.assertIn("pre-rotary move 'G53 G0 Z-40'", text)
        self.assertIn("Events: none", text)
        self.assertGreater(len(p.scene.stock_coords.point.values), 1000)
        self.assertGreater(len(p.scene.partmesh_coords.point.values), 1000)
        # the tool is drawn upright in the machine frame
        np.testing.assert_allclose(p.scene.tool_xf.rotation.value[0][2], (0.0, 0.0, 1.0), atol=1e-9)
        np.testing.assert_allclose(p.scene.tool_xf.rotation.value[0][1], (0.0, 0.0, 1.0), atol=1e-9)
        p.reject()
        p.timer.deleteLater()
        p.form.deleteLater()
        self.app.processEvents()

    def test_slow_speeds(self):
        doc, job, stock, model = make_doc(self.fake)
        self.fc.ActiveDocument = doc
        from camrotarysim.gui.taskpanel import SAMPLES_PER_TICK, SPEEDS, TaskPanel
        p = TaskPanel(doc)
        self.assertTrue(p.build())
        p.change_speed(-10)
        self.assertEqual(p.lbl_speed.text(), "x0.25")
        # x0.25: a quarter of x1's samples per tick
        ticks = [p._tick_samples() for _ in range(8)]
        self.assertEqual(sum(ticks), 8 * SAMPLES_PER_TICK // 4)
        p.change_speed(1)
        self.assertEqual(p.lbl_speed.text(), "x0.5")
        self.assertEqual(sum(p._tick_samples() for _ in range(8)), 8 * SAMPLES_PER_TICK // 2)
        # the speed is saved as a value, so a stored x5 stays x5 whatever the list of steps
        p.change_speed(SPEEDS.index(5) - SPEEDS.index(0.5))
        p.reject()
        self.assertEqual(self.fc.ParamGet("User parameter:BaseApp/Preferences/Mod/CAMRotarySim").GetFloat("Speed", 0.0),
                         5.0)
        p2 = TaskPanel(doc)
        self.assertEqual(p2.lbl_speed.text(), "x5")
        p2.reject()
        for panel in (p, p2):
            panel.timer.deleteLater()
            panel.form.deleteLater()
        self.app.processEvents()

    def test_slow_rotary_index(self):
        self.fake.sys.modules["Machine.models.machine"].MachineFactory.files = {
            "tormach 4-axis": os.path.join(HERE, "data", "rotary", "tormach_4-axis.fcm")}
        doc, job, ops = make_rotary_doc(self.fake)
        self.fc.ActiveDocument = doc
        from camrotarysim.gui.taskpanel import TaskPanel
        p = TaskPanel(doc)
        p.spin_grid.setValue(2.0)
        self.assertTrue(p.build())
        p.change_speed(-10)                # x0.25
        # move to the first index (0 to -90): 90 one-degree samples
        mi = next(i for i, m in enumerate(p.sim.moves) if p._turns(m))
        p.sim.seek(int(p.sim._offsets[mi]))
        done = 0
        for _ in range(40):
            n = p._tick_samples()
            p.sim.advance(n)
            done += n
        self.assertEqual(done, 30)         # 0.75 degree per tick on average
        p.reject()
        p.timer.deleteLater()
        p.form.deleteLater()
        self.app.processEvents()


if __name__ == "__main__":
    unittest.main()
