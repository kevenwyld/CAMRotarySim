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
    tool = types.SimpleNamespace(Name="Tool", Label="Lathe Iso Insert", Shape=tool_shape, ShapeID="isoshape",
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


def make_thread_doc(fake):
    """The thread Job (threading_test.FCStd): one Turn Thread op, M12x1.75, on a 12.000 stock."""
    def load(fn):
        with open(os.path.join(DATA, "thread", fn)) as f:
            return json.load(f)

    q = lambda v: types.SimpleNamespace(Value=v)  # noqa: E731
    sd = load("stock.json")
    r = sd["radius"]
    stock = types.SimpleNamespace(Name="Stock", Label="Stock", ViewObject=VO(),
                                  Shape=fake.Shape(sd["edges"], fake.BoundBox(-r, -r, sd["z_min"], r, r, sd["z_max"])))
    model = types.SimpleNamespace(Name="Clone", Label="Clone", ViewObject=VO(),
                                  Shape=fake.Shape(load("part_section.json")["edges"],
                                                   fake.BoundBox(-6.0, -6.0, -56.2, 6.0, 6.0, 0.0)))
    td = load("tool_threading_section.json")
    tool = types.SimpleNamespace(Name="threading", Label="threading", ShapeID="threading",
                                 Shape=fake.Shape(td["edges"], fake.BoundBox(0.05, -1.5, -3.0, 11.2, 1.5, 3.0),
                                                  section_y=0.0, mesh=([(0, 0, 0), (1, 0, 0), (0, 0, 1)], [(0, 1, 2)])),
                                 ProfileAngle=q(60.0), TipRadius=q(0.05), Width=q(6.0))
    tc = types.SimpleNamespace(Tool=tool, SpindleDir="Forward")
    meta = load("op.json")
    with open(os.path.join(DATA, "thread", "TurnThread.nc")) as f:
        cmds = [Cmd(c, p) for c, p in parse_gcode_text(f.read())]
    proxy_cls = type("ObjectTurnThread", (), {"__module__": "PathTurnScripts.PathTurnThread"})
    op = types.SimpleNamespace(Name="TurnThread", Label="TurnThread", Active=True, Visibility=True, ViewObject=VO(),
                               Proxy=proxy_cls(), ToolController=tc, Path=types.SimpleNamespace(Commands=cmds),
                               Pitch=q(meta["Pitch"]), ThreadDepth=q(meta["ThreadDepth"]),
                               ThreadDiameter=q(meta["ThreadDiameter"]), ThreadReport=meta["ThreadReport"])
    job = types.SimpleNamespace(Name="Job", Label="Job", ViewObject=VO(), Stock=stock,
                                Model=types.SimpleNamespace(OutList=[model]),
                                Operations=types.SimpleNamespace(OutList=[op]))
    objs = {o.Name: o for o in (job, stock, model, op)}
    doc = types.SimpleNamespace(Objects=list(objs.values()), findObjects=lambda t, n: [job],
                                getObject=lambda name: objs.get(name))
    return doc, job, op, tc


class TurnBaseOp:
    """Like PathTurnBase.ObjectOp: is_internal() is Side = Internal on ops that support it."""
    __module__ = "PathTurnScripts.PathTurnBase"
    supports_internal = False

    def is_internal(self, obj):
        return self.supports_internal and getattr(obj, "Side", "External") == "Internal"


def make_boring_doc(fake, min_diameter=10.0, with_drill=False):
    """
    The boring Job (boring_test.FCStd): a tube (OD 29.994, bore 19.994) from a
    r 15 cylinder; TurnFace, TurnProfile (Side Internal, Min Diameter 10),
    TurnRough, and an internal M22x2.5 TurnThread. with_drill: a Turn Drill
    (10 mm CAM drill, G83 through) after TurnFace; its tool has no shape here,
    so its outline comes from the Drill parameters.
    """
    def load(fn):
        with open(os.path.join(DATA, "boring", fn)) as f:
            return json.load(f)

    q = lambda v: types.SimpleNamespace(Value=v)  # noqa: E731
    sd = load("stock.json")
    r = sd["radius"]
    stock = types.SimpleNamespace(Name="Stock", Label="Stock", ViewObject=VO(),
                                  Shape=fake.Shape(sd["edges"], fake.BoundBox(-r, -r, sd["z_min"], r, r, sd["z_max"])))
    model = types.SimpleNamespace(Name="Clone", Label="Clone", ViewObject=VO(),
                                  Shape=fake.Shape(load("part_section.json")["edges"],
                                                   fake.BoundBox(-15.0, -15.0, -113.0, 15.0, 15.0, -3.0)))
    mesh = ([(0, 0, 0), (1, 0, 0), (0, 0, 1)], [(0, 1, 2)])
    iso = types.SimpleNamespace(Name="Lathe_Iso_Insert", Label="Lathe Iso Insert", ShapeID="isoshape",
                                Shape=fake.Shape(load("tool_iso_section.json")["edges"],
                                                 fake.BoundBox(0.0, -1.0, 0.0, 5.79, 0.0, 5.79), section_y=-0.5,
                                                 mesh=mesh),
                                TipRadius=q(0.4), EdgeLength=q(5.0), TipAngle=q(60.0), Rotation=q(45.0))
    thr = types.SimpleNamespace(Name="threading", Label="threading", ShapeID="threading",
                                Shape=fake.Shape(load("tool_threading_section.json")["edges"],
                                                 fake.BoundBox(0.05, -1.5, -3.0, 11.2, 1.5, 3.0), section_y=0.0,
                                                 mesh=mesh),
                                ProfileAngle=q(60.0), TipRadius=q(0.05), Width=q(6.0))
    tcs = {"iso": types.SimpleNamespace(Tool=iso, SpindleDir="Forward"),
           "threading": types.SimpleNamespace(Tool=thr, SpindleDir="Forward")}
    spec = load("ops.json")
    ops = []
    for n in spec["order"]:
        o = spec["ops"][n]
        internal = o["internal"]
        # as in the TurningAddon: every op inherits is_internal() from the base,
        # False without Side support (Turn Thread included); Turn Thread has internal()
        if n == "TurnThread":
            proxy = type("ObjectTurnThread", (TurnBaseOp,), {"__module__": "PathTurnScripts.PathTurnThread",
                                                             "internal": lambda self, obj, v=internal: v})()
        else:
            proxy = type("ObjectTurn", (TurnBaseOp,), {"__module__": "PathTurnScripts.PathTurn" + n[4:],
                                                       "supports_internal": True})()
        with open(os.path.join(DATA, "boring", n + ".nc")) as f:
            cmds = [Cmd(c, p) for c, p in parse_gcode_text(f.read())]
        op = types.SimpleNamespace(Name=n, Label=n, Active=True, Visibility=True, ViewObject=VO(), Proxy=proxy,
                                   ToolController=tcs[o["tool"]], Path=types.SimpleNamespace(Commands=cmds))
        if "SetMinDiameter" in o:
            op.SetMinDiameter = o["SetMinDiameter"]
            op.MinDiameter = q(min_diameter)
        if "Side" in o:
            op.Side = o["Side"]
        for k in ("Pitch", "ThreadDepth", "ThreadDiameter"):
            if k in o:
                setattr(op, k, q(o[k]))
        if "ThreadReport" in o:
            op.ThreadReport = o["ThreadReport"]
        ops.append(op)
        if with_drill and n == "TurnFace":
            drill = types.SimpleNamespace(Name="Drill", Label="10mm Drill", ShapeID="drill", ShapeType="Drill",
                                          Shape=None, Diameter=q(10.0), TipAngle=q(118.0), Length=q(150.0))
            proxy = type("ObjectTurnDrill", (TurnBaseOp,), {"__module__": "PathTurnScripts.PathTurnDrill",
                                                            "uses_drill_tool": True})()
            with open(os.path.join(DATA, "drill", "TurnDrill_G83.nc")) as f:
                cmds = [Cmd(c, p) for c, p in parse_gcode_text(f.read())]
            ops.append(types.SimpleNamespace(Name="TurnDrill", Label="TurnDrill", Active=True, Visibility=True,
                                             ViewObject=VO(), Proxy=proxy,
                                             ToolController=types.SimpleNamespace(Tool=drill, SpindleDir="Forward"),
                                             Path=types.SimpleNamespace(Commands=cmds)))
    job = types.SimpleNamespace(Name="Job", Label="Job", ViewObject=VO(), Stock=stock,
                                Model=types.SimpleNamespace(OutList=[model]),
                                Operations=types.SimpleNamespace(OutList=ops))
    objs = {o.Name: o for o in [job, stock, model] + ops}
    doc = types.SimpleNamespace(Objects=list(objs.values()), findObjects=lambda t, n: [job],
                                getObject=lambda name: objs.get(name))
    return doc, job, ops
