# SPDX-License-Identifier: LGPL-2.1-or-later
import json
import os
import sys
import unittest


HERE = os.path.dirname(__file__)
DATA = os.path.join(HERE, "data")
sys.path.insert(0, os.path.join(HERE, ".."))

from camrotarysim.core.stock import rotary_axes_from_machine  # noqa: E402


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


if __name__ == "__main__":
    unittest.main()
