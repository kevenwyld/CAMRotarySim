# SPDX-License-Identifier: LGPL-2.1-or-later
import os
import sys
import unittest


HERE = os.path.dirname(__file__)
DATA = os.path.join(HERE, "data")
sys.path.insert(0, os.path.join(HERE, ".."))


class TestCoreIsPure(unittest.TestCase):
    def test_no_freecad_imports_in_core(self):
        import re
        core = os.path.join(HERE, "..", "camrotarysim", "core")
        bad = re.compile(r"^\s*(import|from)\s+(FreeCAD|FreeCADGui|Part|Path|PathScripts|pivy|PySide)\b", re.M)
        for fn in os.listdir(core):
            if fn.endswith(".py"):
                with open(os.path.join(core, fn)) as f:
                    self.assertIsNone(bad.search(f.read()), fn)


if __name__ == "__main__":
    unittest.main()
