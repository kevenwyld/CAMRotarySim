# SPDX-License-Identifier: LGPL-2.1-or-later
"""The CAM_RotarySim command and its menu/toolbar placement."""

import FreeCAD
import FreeCADGui
from PySide import QtCore

COMMAND = "CAM_RotarySim"
# CAM's simulator list in the CAM menu and the "Tool Commands" toolbar is
# followed directly by CAM_Inspect, whichever simulator the preferences put
# first (FreeCAD main, Mod/CAM/InitGui.py), so inserting before it places
# this command right after the simulators.
ANCHOR = "CAM_Inspect"


class CommandRotarySim:
    def GetResources(self):
        return {
            "Pixmap": "CAM_RotarySim",
            "MenuText": QtCore.QT_TRANSLATE_NOOP("CAM_RotarySim", "Rotary/Turning Simulator"),
            "ToolTip": QtCore.QT_TRANSLATE_NOOP(
                "CAM_RotarySim",
                "Simulates turning operations on the Job's stock and checks the result against the model"),
        }

    def IsActive(self):
        doc = FreeCAD.ActiveDocument
        return doc is not None and any(o.Name.startswith("Job") for o in doc.Objects)

    def Activated(self):
        if FreeCADGui.Control.activeDialog():
            FreeCAD.Console.PrintWarning("CAMRotarySim: close the open task dialog first\n")
            return
        from .taskpanel import TaskPanel
        FreeCADGui.Control.showDialog(TaskPanel())


class Manipulator:
    """
    Adds the command to the CAM menu and toolbar. FreeCAD rebuilds menus on
    every workbench switch and re-applies manipulators each time, so the
    entries persist; in other workbenches the anchor is absent and the
    entries are no-ops.
    """

    def modifyMenuBar(self):
        return [{"insert": COMMAND, "menuItem": ANCHOR}]

    def modifyToolBars(self):
        return [{"insert": COMMAND, "toolItem": ANCHOR}]


def register():
    FreeCADGui.addCommand(COMMAND, CommandRotarySim())
    FreeCADGui.addWorkbenchManipulator(Manipulator())
