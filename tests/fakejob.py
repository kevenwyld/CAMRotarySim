# SPDX-License-Identifier: LGPL-2.1-or-later
"""A fake CAM Job built from the sample Job fixtures (tests/data), for the stub FreeCAD modules."""

import json
import os
import types

from camrotarysim.core.motion import parse_gcode_text

DATA = os.path.join(os.path.dirname(__file__), "data")


class Cmd:
    def __init__(self, name, params):
        self.Name, self.Parameters = name, params


class VO:
    def __init__(self, visible=True):
        self.Visibility = visible
        self.Icon = None


def make_doc(fake):
    def load(fn):
        with open(os.path.join(DATA, fn)) as f:
            return json.load(f)

    edges = lambda fn: load(fn)["edges"]  # noqa: E731
    sd = load("stock.json")
    r = sd["radius"]
    stock = types.SimpleNamespace(Name="Stock", Label="Stock", ViewObject=VO(),
                                  Shape=fake.Shape(edges("stock.json"),
                                                   fake.BoundBox(-r, -r, sd["z_min"], r, r, sd["z_max"])))
    model = types.SimpleNamespace(Name="Clone", Label="Clone", ViewObject=VO(),
                                  Shape=fake.Shape(edges("part_section.json"),
                                                   fake.BoundBox(-25.3, -25.3, -66.4, 25.3, 25.3, -1)))
    tool_shape = fake.Shape(edges("tool_T37_section.json"), fake.BoundBox(0.27, -1.0, 0.036, 6.99, 0.0, 4.68),
                            section_y=-0.5, mesh=([(0, 0, 0), (1, 0, 0), (0, 0, 1)], [(0, 1, 2)]))
    tool = types.SimpleNamespace(Name="Tool", Label="Lathe Iso Insert", Shape=tool_shape,
                                 TipRadius=types.SimpleNamespace(Value=0.4), EdgeLength=5.0,
                                 TipAngle=60.0, Rotation=57.0)
    tc = types.SimpleNamespace(Tool=tool)
    proxy_cls = type("ObjectTurnFace", (), {"__module__": "PathTurnScripts.PathTurnFace"})
    ops = []
    for n in ("TurnFace", "TurnRough", "TurnProfile"):
        with open(os.path.join(DATA, n + ".nc")) as f:
            cmds = [Cmd(c, p) for c, p in parse_gcode_text(f.read())]
        ops.append(types.SimpleNamespace(Name=n, Label=n, Active=True, Visibility=True, ViewObject=VO(),
                                         Proxy=proxy_cls(), ToolController=tc,
                                         Path=types.SimpleNamespace(Commands=cmds)))
    mill = types.SimpleNamespace(Name="Profile", Label="Profile", Active=True, Visibility=True, ViewObject=VO(),
                                 Proxy=object(), ToolController=types.SimpleNamespace(Tool=types.SimpleNamespace(
                                     Name="EM", Label="Endmill")), Path=types.SimpleNamespace(Commands=[]))
    job = types.SimpleNamespace(Name="Job", Label="Job", ViewObject=VO(), Stock=stock,
                                Model=types.SimpleNamespace(OutList=[model]),
                                Operations=types.SimpleNamespace(OutList=ops + [mill]))
    objs = {o.Name: o for o in [job, stock, model] + ops + [mill]}
    doc = types.SimpleNamespace(Objects=list(objs.values()),
                                findObjects=lambda t, n: [job],
                                getObject=lambda name: objs.get(name))
    return doc, job, stock, model
