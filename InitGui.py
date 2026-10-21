# SPDX-License-Identifier: LGPL-2.1-or-later
"""CAM Rotary Simulator addon: GUI start-up."""

import os

import FreeCADGui

import camrotarysim

ICONS = os.path.join(os.path.dirname(camrotarysim.__file__), "..", "Resources", "icons")
FreeCADGui.addIconPath(os.path.normpath(ICONS))

from camrotarysim.gui import command  # noqa: E402

command.register()
