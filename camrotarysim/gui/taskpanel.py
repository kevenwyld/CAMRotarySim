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
from ..core import compare, thread_check
from ..core import milling
from ..core.mesh import bore_colours, column_colours, revolve
from ..core.surface import surface_nets
from ..core.report import format_report
from ..core.simulation import Simulation
from .view import SimScene, outline_prism

PREFS = "User parameter:BaseApp/Preferences/Mod/CAMRotarySim"
# the CAM simulator's speed steps (GuiDisplay.cpp), with 0.25 and 0.5 for short paths
SPEEDS = (0.25, 0.5, 1, 2, 5, 10, 25, 50)
SAMPLES_PER_TICK = 20                # at speed 1
ROTARY_DEG_PER_TICK = 3              # at speed 1: rotary moves are animated
TICK_MS = 40
MESH_INTERVAL = 0.12                 # s between stock mesh rebuilds during playback (at least)
HANDS = ("Right-hand", "Left-hand")
HOLES = ("Auto", "None", "Custom")
RETRACTS = ("Auto", "None", "Custom")


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
        self.reach_bore = None
        self.part_in = None
        self.part_R = None
        self.part_neg = None
        self.tool_meshes = {}
        self.current_tool_op = None
        self.last_mesh = 0.0
        self.mesh_time = 0.0
        self.zones = []
        self.threads = []
        self.milling = False
        self.part_phi = None
        self.op_info = []
        # stored as the speed itself, so adding steps doesn't change what a saved setting means
        saved = self.prefs.GetFloat("Speed", 5.0)
        self.speed_index = min(range(len(SPEEDS)), key=lambda i: abs(SPEEDS[i] - saved))
        self._budget = 0.0      # fractional samples carried between ticks (speeds below 1)
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
        self.spin_angles = QtGui.QSpinBox()
        self.spin_angles.setRange(24, 720)
        self.spin_angles.setSingleStep(24)
        self.spin_angles.setValue(self.prefs.GetInt("ThreadAngles", 120))
        self.spin_angles.setToolTip(translate("Angles around the part where threads are cut (more: finer, slower)"))
        self.spin_angles.valueChanged.connect(self.mark_dirty)
        grid.addWidget(QtGui.QLabel(translate("Thread angles")), 3, 0)
        grid.addWidget(self.spin_angles, 3, 1)
        self.combo_hand = QtGui.QComboBox()
        for h in HANDS:
            self.combo_hand.addItem(translate(h))
        self.combo_hand.setCurrentIndex(1 if self.prefs.GetBool("ForwardLeftHand", False) else 0)
        self.combo_hand.setToolTip(translate(
            "The thread a Forward spindle cuts when the tool feeds toward -Z. Path commands don't say: it "
            "depends on how the machine turns and which side the tool is on. Reverse cuts the other hand."))
        self.combo_hand.currentIndexChanged.connect(self.mark_dirty)
        grid.addWidget(QtGui.QLabel(translate("Forward spindle cuts")), 4, 0)
        grid.addWidget(self.combo_hand, 4, 1)
        # until drilling is simulated: the hole internal ops bore from
        self.combo_hole = QtGui.QComboBox()
        for h in HOLES:
            self.combo_hole.addItem(translate(h))
        self.combo_hole.setToolTip(translate(
            "The hole that exists before the program (drilled, or in the stock), for boring and internal "
            "threads. Auto: the Min Diameter set on an internal op, down the model's bore (or through)."))
        self.spin_hole_d = self._spin(0.0, 10000.0, 0.0, 3, 0.5)
        self.spin_hole_depth = self._spin(0.0, 10000.0, 0.0, 3, 1.0)
        self.spin_hole_d.setToolTip(translate("Starting hole diameter"))
        self.spin_hole_depth.setToolTip(translate("Starting hole depth from the stock's front"))
        row_hole = QtGui.QHBoxLayout()
        for widget in (self.combo_hole, self.spin_hole_d, self.spin_hole_depth):
            signal = widget.currentIndexChanged if widget is self.combo_hole else widget.valueChanged
            signal.connect(self.on_hole_changed)
            row_hole.addWidget(widget)
        grid.addWidget(QtGui.QLabel(translate("Starting hole")), 5, 0)
        grid.addLayout(row_hole, 5, 1)
        self.spin_grid = self._spin(0.1, 5.0, self.prefs.GetFloat("MillingGrid", 0.5), 2, 0.1)
        self.spin_grid.setToolTip(translate("Milling: spacing of the stock's distance grid (finer: slower, "
                                            "sharper edges; faces are accurate well below it)"))
        self.spin_grid.valueChanged.connect(self.mark_dirty)
        grid.addWidget(QtGui.QLabel(translate("Milling grid")), 6, 0)
        grid.addWidget(self.spin_grid, 6, 1)
        self.combo_retract = QtGui.QComboBox()
        for r in RETRACTS:
            self.combo_retract.addItem(translate(r))
        self.combo_retract.setToolTip(translate(
            "Before the rotary axis turns between work planes. Auto: the machine's pre-rotary move (a G53 move "
            "is shown clearing the stock's swing). None: turn with the tool where it is, which shows whether "
            "the turning stock hits it. Custom: a Z."))
        self.spin_retract = self._spin(-10000.0, 10000.0, 0.0, 3, 1.0)
        self.spin_retract.setToolTip(translate("Retract Z (machine frame, work coordinates)"))
        row_r = QtGui.QHBoxLayout()
        for widget in (self.combo_retract, self.spin_retract):
            signal = widget.currentIndexChanged if widget is self.combo_retract else widget.valueChanged
            signal.connect(self.on_retract_changed)
            row_r.addWidget(widget)
        grid.addWidget(QtGui.QLabel(translate("Retract before indexing")), 7, 0)
        grid.addLayout(row_r, 7, 1)
        self.on_retract_changed()
        self.on_hole_changed()
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

    def on_hole_changed(self, *_):
        custom = self.combo_hole.currentIndex() == 2
        self.spin_hole_d.setEnabled(custom)
        self.spin_hole_depth.setEnabled(custom)
        if hasattr(self, "lbl_status"):
            self.mark_dirty()

    def on_retract_changed(self, *_):
        self.spin_retract.setEnabled(self.combo_retract.currentIndex() == 2)
        if hasattr(self, "lbl_status"):
            self.mark_dirty()

    def retract_setting(self):
        i = self.combo_retract.currentIndex()
        return "auto" if i == 0 else (None if i == 1 else self.spin_retract.value())

    def hole_setting(self):
        i = self.combo_hole.currentIndex()
        if i == 0:
            return "auto"
        if i == 1:
            return None
        return (self.spin_hole_d.value(), self.spin_hole_depth.value())

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
                turning = fa.is_turning_op(op) or fa.is_milling_op(op)
                on = turning and not (self.follow_vis.isChecked() and not op.Visibility and not allhidden)
                item.setCheckState(QtCore.Qt.CheckState.Checked if on else QtCore.Qt.CheckState.Unchecked)
                if not turning:
                    item.setToolTip(translate("Neither a turning nor a milling operation this simulator knows"))
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
        mill = [op for op in ops if fa.is_milling_op(op)]
        self.milling = bool(mill)
        if mill and len(mill) != len(ops):
            self.report.setPlainText(translate(
                "Turning and milling ops can't be simulated together yet: select one kind."))
            return False
        QtGui.QApplication.setOverrideCursor(QtCore.Qt.CursorShape.WaitCursor)
        if self.milling:
            try:
                return self.build_milling(job, ops)
            finally:
                QtGui.QApplication.restoreOverrideCursor()
        try:
            ds = self.spin_res.value()
            right = self.combo_hand.currentIndex() == 0
            self.prefs.SetBool("ForwardLeftHand", not right)
            self.prefs.SetInt("ThreadAngles", self.spin_angles.value())
            stock, inputs, polylines, notes = fa.build_inputs(job, ops, ds, forward_right_hand=right,
                                                              hole=self.hole_setting())
            if self.combo_hole.currentIndex() == 0:   # show what Auto found
                d, depth = stock.hole if getattr(stock, "hole", None) else (0.0, 0.0)
                for spin, v in ((self.spin_hole_d, d), (self.spin_hole_depth, depth)):
                    spin.blockSignals(True)
                    spin.setValue(v)
                    spin.blockSignals(False)
            self.sim = Simulation(stock, inputs, step=ds, thread_angles=self.spin_angles.value())
            self.zones = thread_check.thread_zones(self.sim)
            self.threads = []
            self.notes = notes
            self.part_R = compare.part_profile(polylines, stock.s) if polylines else None
            self.part_neg = compare.part_profile(polylines, stock.s, -1) if polylines else None
            self.part_in = compare.part_inner_profile(polylines, stock.s) if polylines else None
            self.reach = self.reach_bore = None
            if self.part_R is not None:
                # inputs' tools as the simulation uses them (internal ones mirrored)
                ext = {id(i.tool): i.tool for i in self.sim.ops if not i.internal}
                bor = {id(i.tool): i.tool for i in self.sim.ops if i.internal}
                if ext:
                    self.reach = np.min([compare.reachable_profile(self.part_R, stock.s, t)
                                         for t in ext.values()], axis=0)
                if bor:
                    self.reach_bore = np.max([compare.reachable_bore(self.part_in, self.part_R, stock.s, t)
                                              for t in bor.values()], axis=0)
            self.tool_meshes = {}
            for i, (op, inp) in enumerate(zip(ops, self.sim.ops)):
                self.tool_meshes[i] = self._tool_mesh(fa.tool_of(op), inp.tool, inp.internal)
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

    def build_milling(self, job, ops):
        """Milling ops (indexed or rotary): a distance-grid stock in the workpiece frame."""
        try:
            h = self.spin_grid.value()
            self.prefs.SetFloat("MillingGrid", h)
            stock, inputs, part_phi, notes, info, tools = fa.build_milling_inputs(job, ops, h, self.retract_setting())
            if self.combo_retract.currentIndex() == 0:   # show what Auto found
                self.spin_retract.blockSignals(True)
                self.spin_retract.setValue(stock.retract if stock.retract is not None else 0.0)
                self.spin_retract.blockSignals(False)
            self.sim = Simulation(stock, inputs, step=min(0.25, h / 2))
            self.notes, self.op_info, self.part_phi = notes, info, part_phi
            self.zones, self.threads = [], []
            self.tool_meshes = {}
            for i, (inp, tool) in enumerate(zip(inputs, tools)):
                self.tool_meshes[i] = self._mill_tool_mesh(tool, inp.tool)
            if self.scene is None:
                self.scene = SimScene(FreeCADGui.ActiveDocument.ActiveView)
                self._hide_job(job)
            self.scene.set_path(self._world_path())
            self.scene.set_part_section([])
            model = fa.model_shape(job)
            if model is not None:
                tris = fa.shape_triangles(model)
                v = tris.reshape(-1, 3)
                self.scene.set_part_mesh(v, np.arange(len(v)).reshape(-1, 3))
            self.current_tool_op = None
            self.dirty = False
        except Exception as e:
            FreeCAD.Console.PrintError("CAMRotarySim: %s\n" % e)
            self.report.setPlainText(translate("Could not build the simulation:") + "\n%s" % e)
            self.sim = None
            return False
        self.update_all()
        return True

    def _world_path(self):
        """The path in the workpiece frame (each op through its own frame)."""
        out = []
        for kind, op, pts in self.sim.path_polylines():
            kin = self.sim.ops[op].kinematics
            full = np.zeros((len(pts), 6))
            full[:, :3] = pts
            out.append((kind, op, np.array([kin.to_workpiece(p)[0] for p in full])))
        return out

    @staticmethod
    def _mill_tool_mesh(tool, mill_tool):
        """A milling tool's solid (CAM shapes: tip at the origin, axis +Z), or a cylinder."""
        try:
            shape = tool.Shape.copy()
            shape.Placement = FreeCAD.Placement()
            pts, tris = shape.tessellate(0.05)
            if pts and tris:
                return np.array([(p.x, p.y, p.z) for p in pts]), np.array(tris), (0.0, 0.0)
        except Exception:
            pass
        n = 24
        a = np.linspace(0, 2 * np.pi, n, endpoint=False)
        r, L = mill_tool.radius, mill_tool.length
        v = np.array([(r * np.cos(t), r * np.sin(t), z) for z in (0.0, L) for t in a] + [(0, 0, 0), (0, 0, L)])
        i = np.arange(n)
        j = (i + 1) % n
        tris = np.vstack([np.stack([i, j, n + j], 1), np.stack([i, n + j, n + i], 1),
                          np.stack([np.full(n, 2 * n), j, i], 1), np.stack([np.full(n, 2 * n + 1), n + i, n + j], 1)])
        return v, tris, (0.0, 0.0)

    def milling_angle(self):
        """The rotary angle the machine is at: the current op's (its plane's) plus any rotary word."""
        m = self.sim.current_move
        if m is None:
            return 0.0, None
        inp = self.sim.ops[m.op]
        kin = inp.kinematics
        word = float(self.sim.position[kin.index]) if self.sim.position is not None else 0.0
        return float(inp.meta.get("angle", 0.0)) + word, kin

    @staticmethod
    def _tool_mesh(tool, outline, internal=False):
        """The tool's solid (or a prism of its outline), mirrored about the control point when used inside."""
        cx = outline.control_in_shape[0]
        try:
            shape = tool.Shape.copy()
            shape.Placement = FreeCAD.Placement()
            pts, tris = shape.tessellate(0.02)
            if pts and tris:
                v = np.array([(p.x, p.y, p.z) for p in pts])
                if internal:
                    v[:, 0] = 2 * cx - v[:, 0]
                return v, np.array(tris), outline.control_in_shape
        except Exception:
            pass
        v, t = outline_prism(outline)   # the outline is already mirrored for internal use
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
        self.prefs.SetFloat("Speed", float(SPEEDS[self.speed_index]))

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

    def _tick_samples(self):
        """Samples for one tick: SAMPLES_PER_TICK x speed, or ROTARY_DEG_PER_TICK x speed during a
        rotary index (a tick stops where one begins); fractions carry over for speeds below 1."""
        speed = SPEEDS[self.speed_index]
        sim = self.sim
        rate = speed * SAMPLES_PER_TICK
        mi = sim.move_at(sim.cursor) if sim.moves and not sim.done else None
        turning = mi is not None and self._turns(sim.moves[mi])
        if turning:
            rate = min(rate, speed * ROTARY_DEG_PER_TICK)
        self._budget += rate
        n = int(self._budget)
        if n < 1:
            return 0
        self._budget -= n
        if mi is None:
            return n
        offsets = sim._offsets
        if turning:
            return min(n, int(offsets[mi + 1]) - sim.cursor)
        j = mi + 1
        while j < len(sim.moves) and int(offsets[j]) < sim.cursor + n:
            if self._turns(sim.moves[j]):
                self._budget = 0.0
                return max(1, int(offsets[j]) - sim.cursor)
            j += 1
        return n

    @staticmethod
    def _turns(move):
        return any(abs(move.end[i] - move.start[i]) > 1e-9 for i in (3, 4, 5))

    def change_speed(self, delta):
        self.speed_index = max(0, min(len(SPEEDS) - 1, self.speed_index + delta))
        self._budget = 0.0
        self.lbl_speed.setText("x%g" % SPEEDS[self.speed_index])

    def on_tick(self):
        if self.sim is None:
            self.timer.stop()
            return
        self.sim.advance(self._tick_samples())
        if self.sim.done:
            self.pause()
            return
        self.update_tool()
        self.update_progress()
        now = time.monotonic()
        if now - self.last_mesh > max(MESH_INTERVAL, 3 * self.mesh_time):
            self.update_stock_mesh(final=False)
            self.last_mesh = time.monotonic()
            self.mesh_time = self.last_mesh - now

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
        return compare.compare(st.outer_profile(), self.part_R, st.s, self.spin_gouge.value(), self.spin_uncut.value(),
                               self.reach, [(z.z0, z.z1) for z in self.zones if not z.internal])

    def bore_comparison(self):
        """The bore against the part's, when either has one."""
        if self.sim is None or self.part_R is None:
            return None
        st = self.sim.stock
        if not (st.has_bore() or np.any(self.part_in > 0)):
            return None
        return compare.compare_bore(st.inner_profile(), self.part_in, self.part_R, st.s, self.spin_gouge.value(),
                                    self.spin_uncut.value(), self.reach_bore,
                                    [(z.z0, z.z1) for z in self.zones if z.internal])

    def measure_threads(self):
        st = self.sim.stock
        self.threads = []
        for z in self.zones:
            tool = self.sim.ops[z.op].tool
            self.threads.append(thread_check.measure(st, z, max(-tool.zmin, tool.zmax), self.spin_gouge.value()))
        return self.threads

    def update_stock_mesh(self, *_, final=True):
        if self.sim is None or self.scene is None:
            return
        st = self.sim.stock
        dev = self.chk_dev.isChecked()
        if self.milling:
            v, t = surface_nets(st.phi, st.lo, st.h)
            c = milling.vertex_colours(st, v, self.part_phi if dev else None, self.spin_gouge.value(),
                                       self.spin_uncut.value())
            self.scene.set_stock_mesh(v, t, c)
            return
        res = self.comparison() if dev else None
        bore = self.bore_comparison() if dev else None
        outer_v, inner_v = (), ()
        if res is not None and final and self.zones:
            measured = self.measure_threads()
            outer_v = [((t.zone.z0, t.zone.z1), t.verdict) for t in measured if not t.zone.internal]
            inner_v = [((t.zone.z0, t.zone.z1), t.verdict) for t in measured if t.zone.internal]
        colours = column_colours(st, res.classes if res is not None else None, thread_verdicts=outer_v)
        inner = bore_colours(st, bore.classes if bore is not None else None, thread_verdicts=inner_v)
        v, t, c = revolve(st, colours, inner_colours=inner)
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
        if self.milling:
            angle, kin = self.milling_angle()
            tip, d = kin.to_workpiece(np.asarray(sim.position, dtype=float))
            ax = kin.axis
            M = milling.rotation_matrix(ax.direction, angle)
            o = np.asarray(ax.origin)
            self.scene.set_workpiece_rotation(ax.direction, ax.origin, angle)
            self.scene.set_tool_pose(M @ (tip - o) + o, M @ d)
            return
        self.scene.set_tool_position(sim.position, self.tool_meshes[op][2])
        self.scene.set_spindle_angle(sim.spindle_angle)

    def update_progress(self):
        sim = self.sim
        self.slider.blockSignals(True)
        self.slider.setValue(int(round(1000 * sim.progress)))
        self.slider.blockSignals(False)
        m = sim.current_move
        if m is not None and self.milling:
            angle, kin = self.milling_angle()
            p = sim.position
            self.lbl_status.setText("%s  #%d  X %.3f  Y %.3f  Z %.3f  %s %.1f" % (
                sim.ops[m.op].name, m.index, p[0], p[1], p[2], kin.letter, angle))
        elif m is not None:
            self.lbl_status.setText("%s  #%d  X %.3f  Z %.3f" % (
                sim.ops[m.op].name, m.index, sim.position[0], sim.position[2]))

    def update_visibility(self, *_):
        if self.scene is not None:
            part = self.chk_part.isChecked()
            mill = bool(getattr(self, "milling", False))
            self.scene.show(self.chk_stock.isChecked(), self.chk_tool.isChecked(),
                            self.chk_path.isChecked(), part and not mill, part and mill)

    def update_result(self, *_):
        if self.sim is not None:
            self.update_stock_mesh()
            self.update_report()

    def update_report(self):
        if self.sim is None:
            return
        if self.milling:
            cmp = None
            if self.part_phi is not None:
                cmp = milling.compare_part(self.sim.stock, self.part_phi, self.spin_gouge.value(),
                                           self.spin_uncut.value())
            text = milling.format_milling(self.sim, cmp, self.op_info)
            text += "\n\nSetup:\n" + "\n".join("  " + n for n in self.notes)
            text += ("\n\nNotes: the angles are what each op's work plane needs, solved here; compare "
                     "them with the posted program. Edges are drawn chamfered by up to a third of the grid; "
                     "the check is on the grid, where it isn't.")
            self.report.setPlainText(text)
            return
        threads = self.measure_threads() if self.zones else None
        text = format_report(self.sim, self.comparison(), self.part_R, self.part_neg, threads=threads,
                             bore=self.bore_comparison())
        if getattr(self, "notes", None):
            text += "\n\nTools:\n" + "\n".join("  " + n for n in self.notes)
        text += ("\n\nNotes: 'reachable' and holder checks use the insert outline only; "
                 "the holder and chuck are not modelled.")
        if self.zones:
            text += (" Threads: %s for a Forward spindle feeding toward -Z (setting above); the rapid out "
                     "after each G33 is taken as instant." % self.combo_hand.currentText().lower())
        self.report.setPlainText(text)
