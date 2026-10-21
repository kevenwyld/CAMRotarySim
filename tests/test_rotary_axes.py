# SPDX-License-Identifier: LGPL-2.1-or-later
import json
import os
import sys
import unittest

import numpy as np

HERE = os.path.dirname(__file__)
DATA = os.path.join(HERE, "data")
sys.path.insert(0, os.path.join(HERE, ".."))

from camrotarysim.core.kinematics import RotaryMillingKinematics  # noqa: E402
from camrotarysim.core.stock import RotaryAxis, rotary_axes_from_machine  # noqa: E402


def _read(path):
    with open(path) as f:
        return f.read()


class TestRotaryAxes(unittest.TestCase):
    def test_machine_rotary_axis(self):
        m = json.loads(_read(os.path.join(DATA, "tormach_4-axis.fcm")))
        axes = rotary_axes_from_machine(m)
        self.assertEqual(list(axes), ["A"])
        self.assertEqual(axes["A"].direction, (1.0, 0.0, 0.0))
        self.assertEqual(axes["A"].limits, (-120.0, 120.0))
        lathe = json.loads(_read(os.path.join(DATA, "lathe_test_setup.fcm")))
        self.assertEqual(rotary_axes_from_machine(lathe), {})

    def test_rotary_milling_transform(self):
        kin = RotaryMillingKinematics("A", RotaryAxis(direction=(1, 0, 0)))
        p, tool_axis = kin.to_workpiece((0.0, 0.0, 10.0, 90.0, 0.0, 0.0))
        # A=+90 turns the workpiece; in the workpiece frame the tool point rotates by -90 about X
        np.testing.assert_allclose(p, (0.0, 10.0, 0.0), atol=1e-9)
        np.testing.assert_allclose(tool_axis, (0.0, 1.0, 0.0), atol=1e-9)
        with self.assertRaises(NotImplementedError):
            kin.apply(None, None, (0,) * 6)


if __name__ == "__main__":
    unittest.main()
