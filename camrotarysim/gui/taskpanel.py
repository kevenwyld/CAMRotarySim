# SPDX-License-Identifier: LGPL-2.1-or-later
"""
Task panel: choose the Job and operations (as CAM's simulator does), play,
pause, step, change speed, scrub, reset, and read the quality report.
The result is drawn in the active 3D view.
"""

import time

import numpy as np

import FreeCAD
import FreeCADGui
from PySide import QtCore, QtGui

from .. import freecad_adapter as fa
from ..core import compare
from ..core.mesh import column_colours, revolve
from ..core.report import format_report
from ..core.simulation import Simulation
from .view import SimScene, outline_prism

PREFS = "User parameter:BaseApp/Preferences/Mod/CAMRotarySim"
SPEEDS = (1, 5, 10, 25, 50)          # same steps as the CAM simulator's speed control
SAMPLES_PER_TICK = 20                # at speed 1
TICK_MS = 40
MESH_INTERVAL = 0.12                 # s between stock mesh rebuilds during playback


def _icon(obj):
    icon = getattr(getattr(obj, "ViewObject", None), "Icon", None)
    return icon if isinstance(icon, QtGui.QIcon) else QtGui.QIcon()


def translate(text):
    return QtCore.QCoreApplication.translate("CAMRotarySim", text)


class TaskPanel:
    def __init__(self, doc=None):
        self.doc = doc or FreeCAD.ActiveDocument
        self.prefs = FreeCAD.ParamGet(PREFS)
        self.jobs = fa.find_jobs(self.doc)
        self.sim = None
        self.scene = None
        self.dirty = True
        self.hidden = []
        self.reach = None
        self.part_R = None
        self.part_neg = None
        self.tool_meshes = {}
        self.current_tool_op = None
        self.last_mesh = 0.0
        self.speed_index = min(self.prefs.GetInt("SpeedIndex", 1), len(SPEEDS) - 1)
        self.timer = QtCore.QTimer()
        self.timer.setInterval(TICK_MS)
        self.timer.timeout.connect(self.on_tick)
        self.form = self._build_form()
        self.on_job_changed()

    # ------------------------------------------------------------------ UI
    def _build_form(self):
        w = QtGui.QWidget()
        w.setWindowTitle(translate("Rotary/Turning Simulator"))
        lay = QtGui.QVBoxLayout(w)

        self.combo_jobs = QtGui.QComboBox()
        for j in self.jobs:
            self.combo_jobs.addItem(_icon(j), j.Label)
        self.combo_jobs.currentIndexChanged.connect(self.on_job_changed)
        lay.addWidget(QtGui.QLabel(translate("Job")))
        lay.addWidget(self.combo_jobs)

        lay.addWidget(QtGui.QLabel(translate("Operations")))
        self.list_ops = QtGui.QListWidget()
        self.list_ops.itemChanged.connect(self.mark_dirty)
        lay.addWidget(self.list_ops)
        self.follow_vis = QtGui.QCheckBox(translate("Follows visibility"))
        self.follow_vis.setChecked(self.prefs.GetBool("FollowsVisibility", False))
        self.follow_vis.toggled.connect(self.on_follow_visibility)
        lay.addWidget(self.follow_vis)

        grid = QtGui.QGridLayout()
        self.spin_res = self._spin(0.005, 1.0, self.prefs.GetFloat("Resolution", 0.05), 3, 0.01)
        self.spin_gouge = self._spin(0.0, 5.0, self.prefs.GetFloat("GougeTolerance", 0.01), 3, 0.005)
        self.spin_uncut = self._spin(0.0, 5.0, self.prefs.GetFloat("UncutTolerance", 0.05), 3, 0.01)
        self.spin_res.valueChanged.connect(self.mark_dirty)
        self.spin_gouge.valueChanged.connect(self.update_result)
        self.spin_uncut.valueChanged.connect(self.update_result)
        for row, (label, sp) in enumerate(((translate("Resolution"), self.spin_res),
                                           (translate("Gouge tolerance"), self.spin_gouge),
                                           (translate("Uncut tolerance"), self.spin_uncut))):
            grid.addWidget(QtGui.QLabel(label), row, 0)
            grid.addWidget(sp, row, 1)
        lay.addLayout(grid)

        self.slider = QtGui.QSlider(QtCore.Qt.Orientation.Horizontal)
        self.slider.setRange(0, 1000)
        self.slider.sliderMoved.connect(self.on_slider)
        lay.addWidget(self.slider)

        row = QtGui.QHBoxLayout()
        self.btn_reset = QtGui.QPushButton("\u23ee")
        self.btn_play = QtGui.QPushButton("\u25b6")
        self.btn_step = QtGui.QPushButton("\u23ed")
        self.btn_slower = QtGui.QPushButton("\u2212")
        self.btn_faster = QtGui.QPushButton("+")
        self.lbl_speed = QtGui.QLabel()
        self.btn_reset.setToolTip(translate("Reset"))
        self.btn_play.setToolTip(translate("Play / pause"))
        self.btn_step.setToolTip(translate("Single step (one move)"))
        self.btn_slower.setToolTip(translate("Slower"))
        self.btn_faster.setToolTip(translate("Faster"))
        self.btn_reset.clicked.connect(self.on_reset)
        self.btn_play.clicked.connect(self.on_play)
        self.btn_step.clicked.connect(self.on_step)
        self.btn_slower.clicked.connect(lambda: self.change_speed(-1))
        self.btn_faster.clicked.connect(lambda: self.change_speed(1))
        for b in (self.btn_reset, self.btn_play, self.btn_step, self.btn_slower, self.lbl_speed, self.btn_faster):
            row.addWidget(b)
        lay.addLayout(row)
        self.change_speed(0)

        self.lbl_status = QtGui.QLabel()
        lay.addWidget(self.lbl_status)

        row = QtGui.QHBoxLayout()
        self.chk_stock = QtGui.QCheckBox(translate("Stock"))
        self.chk_tool = QtGui.QCheckBox(translate("Tool"))
        self.chk_path = QtGui.QCheckBox(translate("Path"))
        self.chk_part = QtGui.QCheckBox(translate("Part outline"))
        self.chk_dev = QtGui.QCheckBox(translate("Deviation colours"))
        for c, on in ((self.chk_stock, True), (self.chk_tool, True), (self.chk_path, False),
                      (self.chk_part, True), (self.chk_dev, True)):
            c.setChecked(on)
            c.toggled.connect(self.update_visibility)
            row.addWidget(c)
        self.chk_dev.toggled.connect(self.update_stock_mesh)
        lay.addLayout(row)

        self.chk_hide = QtGui.QCheckBox(translate("Hide the Job's stock and model while simulating"))
        self.chk_hide.setChecked(True)
        lay.addWidget(self.chk_hide)

        self.report = QtGui.QPlainTextEdit()
        self.report.setReadOnly(True)
        font = QtGui.QFontDatabase.systemFont(QtGui.QFontDatabase.SystemFont.FixedFont)
        self.report.setFont(font)
        self.report.setMinimumHeight(180)
        lay.addWidget(self.report)
        return w

    @staticmethod
    def _spin(lo, hi, val, dec, step):
        s = QtGui.QDoubleSpinBox()
        s.setRange(lo, hi)
        s.setDecimals(dec)
        s.setSingleStep(step)
        s.setSuffix(" mm")
        s.setValue(val)
        return s

    def getStandardButtons(self):
        return QtGui.QDialogButtonBox.StandardButton.Close

    def reject(self):
        self.cleanup()
        FreeCADGui.Control.closeDialog()
        return True

    def accept(self):
        return self.reject()

    # ------------------------------------------------------------ job/ops
    @property
    def job(self):
        i = self.combo_jobs.currentIndex()
        return self.jobs[i] if 0 <= i < len(self.jobs) else None

    def on_job_changed(self, *_):
        self.list_ops.blockSignals(True)
        self.list_ops.clear()
        job = self.job
        if job is not None:
            ops = fa.active_operations(job)
            allhidden = all(not op.Visibility for op in ops)
            for op in ops:
                item = QtGui.QListWidgetItem(_icon(op), op.Label)
                item.setFlags(item.flags() | QtCore.Qt.ItemFlag.ItemIsUserCheckable)
                turning = fa.is_turning_op(op)
                on = turning and not (self.follow_vis.isChecked() and not op.Visibility and not allhidden)
                item.setCheckState(QtCore.Qt.CheckState.Checked if on else QtCore.Qt.CheckState.Unchecked)
                if not turning:
                    item.setToolTip(translate("Not a turning operation: milling ops are not simulated yet"))
                item.setData(QtCore.Qt.ItemDataRole.UserRole, op.Name)
                self.list_ops.addItem(item)
        self.list_ops.blockSignals(False)
        self.mark_dirty()

    def on_follow_visibility(self, on):
        self.prefs.SetBool("FollowsVisibility", bool(on))
        self.on_job_changed()

    def selected_ops(self):
        out = []
        for i in range(self.list_ops.count()):
            it = self.list_ops.item(i)
            if it.checkState() == QtCore.Qt.CheckState.Checked:
                out.append(self.doc.getObject(it.data(QtCore.Qt.ItemDataRole.UserRole)))
        return out

    def mark_dirty(self, *_):
        self.dirty = True
        self.pause()
        self.lbl_status.setText(translate("Press play to build the simulation."))

    # ---------------------------------------------------------- building
    def build(self):
        job = self.job
        ops = self.selected_ops()
        if job is None or not ops:
            self.report.setPlainText(translate("Select a Job and at least one operation."))
            return False
        self.prefs.SetFloat("Resolution", self.spin_res.value())
        QtGui.QApplication.setOverrideCursor(QtCore.Qt.CursorShape.WaitCursor)
        try:
            ds = self.spin_res.value()
            stock, inputs, polylines, notes = fa.build_inputs(job, ops, ds)
            self.sim = Simulation(stock, inputs, step=ds)
            self.notes = notes
            self.part_R = compare.part_profile(polylines, stock.s) if polylines else None
            self.part_neg = compare.part_profile(polylines, stock.s, -1) if polylines else None
            self.reach = None
            if self.part_R is not None:
                tools = {id(i.tool): i.tool for i in inputs}
                self.reach = np.min([compare.reachable_profile(self.part_R, stock.s, t) for t in tools.values()],
                                    axis=0)
            self.tool_meshes = {}
            for i, (op, inp) in enumerate(zip(ops, inputs)):
                self.tool_meshes[i] = self._tool_mesh(fa.tool_of(op), inp.tool)
            if self.scene is None:
                self.scene = SimScene(FreeCADGui.ActiveDocument.ActiveView)
                self._hide_job(job)
            self.scene.set_path(self.sim.path_polylines())
            self.scene.set_part_section(polylines)
            self.current_tool_op = None
            self.dirty = False
        except Exception as e:
            FreeCAD.Console.PrintError("CAMRotarySim: %s\n" % e)
            self.report.setPlainText(translate("Could not build the simulation:") + "\n%s" % e)
            self.sim = None
            return False
        finally:
            QtGui.QApplication.restoreOverrideCursor()
        self.update_all()
        return True

    @staticmethod
    def _tool_mesh(tool, outline):
        try:
            shape = tool.Shape.copy()
            shape.Placement = FreeCAD.Placement()
            pts, tris = shape.tessellate(0.02)
            if pts and tris:
                return np.array([(p.x, p.y, p.z) for p in pts]), np.array(tris), outline.control_in_shape
        except Exception:
            pass
        v, t = outline_prism(outline)
        return v, t, outline.control_in_shape

    def _hide_job(self, job):
        if not self.chk_hide.isChecked():
            return
        objs = [job.Stock] + list(job.Model.OutList)
        for o in objs:
            vo = getattr(o, "ViewObject", None)
            if vo is not None and vo.Visibility:
                vo.Visibility = False
                self.hidden.append(o)

    def cleanup(self):
        self.timer.stop()
        if self.scene is not None:
            self.scene.remove()
            self.scene = None
        for o in self.hidden:
            try:
                o.ViewObject.Visibility = True
            except Exception:
                pass
        self.hidden = []
        self.prefs.SetFloat("GougeTolerance", self.spin_gouge.value())
        self.prefs.SetFloat("UncutTolerance", self.spin_uncut.value())
        self.prefs.SetInt("SpeedIndex", self.speed_index)

    def ensure_built(self):
        return (not self.dirty and self.sim is not None) or self.build()

    # ---------------------------------------------------------- playback
    def on_play(self):
        if self.timer.isActive():
            self.pause()
            return
        if not self.ensure_built():
            return
        if self.sim.done:
            self.sim.reset()
        self.timer.start()
        self.btn_play.setText("\u23f8")

    def pause(self):
        if self.timer.isActive():
            self.timer.stop()
            self.btn_play.setText("\u25b6")
            self.update_all()

    def on_step(self):
        self.pause()
        if self.ensure_built():
            self.sim.advance_to_move_end()
            self.update_all()

    def on_reset(self):
        self.pause()
        if self.ensure_built():
            self.sim.reset()
            self.update_all()

    def on_slider(self, value):
        if not self.ensure_built():
            return
        self.pause()
        self.sim.seek(int(self.sim.total_samples * value / 1000.0))
        self.update_all(report=False)

    def change_speed(self, delta):
        self.speed_index = max(0, min(len(SPEEDS) - 1, self.speed_index + delta))
        self.lbl_speed.setText("x%d" % SPEEDS[self.speed_index])

    def on_tick(self):
        if self.sim is None:
            self.timer.stop()
            return
        self.sim.advance(SPEEDS[self.speed_index] * SAMPLES_PER_TICK)
        if self.sim.done:
            self.pause()
            return
        self.update_tool()
        self.update_progress()
        now = time.monotonic()
        if now - self.last_mesh > MESH_INTERVAL:
            self.update_stock_mesh()
            self.last_mesh = now

    # ------------------------------------------------------------ display
    def update_all(self, report=True):
        if self.sim is None or self.scene is None:
            return
        self.update_stock_mesh()
        self.update_tool()
        self.update_progress()
        self.update_visibility()
        if report:
            self.update_report()

    def comparison(self):
        if self.sim is None or self.part_R is None:
            return None
        st = self.sim.stock
        return compare.compare(st.profile(), self.part_R, st.s, self.spin_gouge.value(),
                               self.spin_uncut.value(), self.reach)

    def update_stock_mesh(self, *_):
        if self.sim is None or self.scene is None:
            return
        st = self.sim.stock
        res = self.comparison() if self.chk_dev.isChecked() else None
        colours = column_colours(st, res.classes if res is not None else None)
        v, t, c = revolve(st, colours)
        self.scene.set_stock_mesh(v, t, c)

    def update_tool(self):
        sim = self.sim
        if sim is None or self.scene is None or sim.position is None or not sim.moves:
            return
        op = sim.current_move.op
        if op != self.current_tool_op and op in self.tool_meshes:
            v, t, _c = self.tool_meshes[op]
            self.scene.set_tool_mesh(v, t)
            self.current_tool_op = op
        self.scene.set_tool_position(sim.position, self.tool_meshes[op][2])

    def update_progress(self):
        sim = self.sim
        self.slider.blockSignals(True)
        self.slider.setValue(int(round(1000 * sim.progress)))
        self.slider.blockSignals(False)
        m = sim.current_move
        if m is not None:
            self.lbl_status.setText("%s  #%d  X %.3f  Z %.3f" % (
                sim.ops[m.op].name, m.index, sim.position[0], sim.position[2]))

    def update_visibility(self, *_):
        if self.scene is not None:
            self.scene.show(self.chk_stock.isChecked(), self.chk_tool.isChecked(),
                            self.chk_path.isChecked(), self.chk_part.isChecked())

    def update_result(self, *_):
        if self.sim is not None:
            self.update_stock_mesh()
            self.update_report()

    def update_report(self):
        if self.sim is None:
            return
        text = format_report(self.sim, self.comparison(), self.part_R, self.part_neg)
        if getattr(self, "notes", None):
            text += "\n\nTools:\n" + "\n".join("  " + n for n in self.notes)
        text += ("\n\nNotes: 'reachable' and holder checks use the insert outline only; "
                 "the holder and chuck are not modelled.")
        self.report.setPlainText(text)
