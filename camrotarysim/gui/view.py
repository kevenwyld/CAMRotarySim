# SPDX-License-Identifier: LGPL-2.1-or-later
"""
Coin3D scene for the simulation, added to the active 3D view's scene graph:
stock mesh (vertex colours), tool (tessellated tool solid or outline
prism), tool path and the part's section outline.
"""

import math

import numpy as np
from pivy import coin

from ..core.motion import RAPID

RAPID_COLOUR = (0.9, 0.3, 0.3)
FEED_COLOUR = (0.2, 0.6, 1.0)
PART_COLOUR = (0.1, 0.1, 0.1)
TOOL_COLOUR = (0.85, 0.65, 0.2)
PART_GHOST_COLOUR = (0.3, 0.3, 0.35)


def _switch(child, on=True):
    sw = coin.SoSwitch()
    sw.addChild(child)
    sw.whichChild = coin.SO_SWITCH_ALL if on else coin.SO_SWITCH_NONE
    return sw


def _set_on(sw, on):
    sw.whichChild = coin.SO_SWITCH_ALL if on else coin.SO_SWITCH_NONE


def _faces_index(tris):
    idx = np.full((len(tris), 4), -1, dtype=np.int64)
    idx[:, :3] = tris
    return idx.ravel().tolist()


class SimScene:
    def __init__(self, view):
        self.view = view
        self.root = coin.SoSeparator()
        self.root.setName("CAMRotarySim")

        # stock
        sep = coin.SoSeparator()
        hints = coin.SoShapeHints()
        hints.vertexOrdering = coin.SoShapeHints.COUNTERCLOCKWISE
        hints.shapeType = coin.SoShapeHints.SOLID
        hints.creaseAngle = 0.6
        self.stock_mat = coin.SoMaterial()
        # the workpiece turns with the spindle during G33, so the tool (at
        # world angle 0) sits in the groove it is cutting
        self.stock_rot = coin.SoRotation()
        bind = coin.SoMaterialBinding()
        bind.value = coin.SoMaterialBinding.PER_VERTEX_INDEXED
        self.stock_coords = coin.SoCoordinate3()
        self.stock_faces = coin.SoIndexedFaceSet()
        for n in (hints, self.stock_rot, self.stock_mat, bind, self.stock_coords, self.stock_faces):
            sep.addChild(n)
        self.stock_sw = _switch(sep)

        # tool
        sep = coin.SoSeparator()
        self.tool_xf = coin.SoTransform()
        mat = coin.SoMaterial()
        mat.diffuseColor = TOOL_COLOUR
        self.tool_coords = coin.SoCoordinate3()
        self.tool_faces = coin.SoIndexedFaceSet()
        th = coin.SoShapeHints()
        th.vertexOrdering = coin.SoShapeHints.UNKNOWN_ORDERING
        for n in (self.tool_xf, mat, th, self.tool_coords, self.tool_faces):
            sep.addChild(n)
        self.tool_sw = _switch(sep)

        # path and part section
        self.path_sw, self.path_nodes = self._lines_node(1.0)
        self.part_sw, self.part_nodes = self._lines_node(2.0)
        # the part as a see-through mesh (milling)
        sep = coin.SoSeparator()
        mat = coin.SoMaterial()
        mat.diffuseColor = PART_GHOST_COLOUR
        mat.transparency = 0.75
        self.partmesh_coords = coin.SoCoordinate3()
        self.partmesh_faces = coin.SoIndexedFaceSet()
        for n in (mat, self.partmesh_coords, self.partmesh_faces):
            sep.addChild(n)
        self.partmesh_sw = _switch(sep, False)
        # the workpiece (stock, path, part) turns by the rotary angle about its axis (milling)
        self.work = coin.SoSeparator()
        self.work_xf = coin.SoTransform()
        self.work.addChild(self.work_xf)
        for sw in (self.stock_sw, self.path_sw, self.part_sw, self.partmesh_sw):
            self.work.addChild(sw)
        self.root.addChild(self.work)
        self.root.addChild(self.tool_sw)
        view.getSceneGraph().addChild(self.root)

    @staticmethod
    def _lines_node(width):
        sep = coin.SoSeparator()
        style = coin.SoDrawStyle()
        style.lineWidth = width
        light = coin.SoLightModel()
        light.model = coin.SoLightModel.BASE_COLOR
        mat = coin.SoMaterial()
        bind = coin.SoMaterialBinding()
        bind.value = coin.SoMaterialBinding.PER_PART
        coords = coin.SoCoordinate3()
        lines = coin.SoLineSet()
        for n in (style, light, mat, bind, coords, lines):
            sep.addChild(n)
        return _switch(sep), (mat, coords, lines)

    # ---------------------------------------------------------------- update
    def set_stock_mesh(self, verts, tris, colours):
        self.stock_coords.point.setNum(len(verts))
        self.stock_coords.point.setValues(0, len(verts), verts.tolist())
        self.stock_mat.diffuseColor.setNum(len(colours))
        self.stock_mat.diffuseColor.setValues(0, len(colours), colours.tolist())
        idx = _faces_index(tris)
        self.stock_faces.coordIndex.setNum(len(idx))
        self.stock_faces.coordIndex.setValues(0, len(idx), idx)
        self.stock_faces.materialIndex.setNum(len(idx))
        self.stock_faces.materialIndex.setValues(0, len(idx), idx)

    def set_spindle_angle(self, angle):
        """Rotate the displayed workpiece by angle (rad) about the Z axis."""
        self.stock_rot.rotation.setValue(coin.SbVec3f(0.0, 0.0, 1.0), float(angle))

    def set_workpiece_rotation(self, axis, origin, angle_deg):
        """Turn the workpiece (stock, path, part) by angle about the rotary axis through origin."""
        self.work_xf.rotation.setValue(coin.SbVec3f(*map(float, axis)), math.radians(float(angle_deg)))
        self.work_xf.center.setValue(*map(float, origin))

    def set_tool_pose(self, tip, tool_axis, control_in_shape=(0.0, 0.0)):
        """Tool (a CAM shape: tip at its origin, axis +Z) at tip, along tool_axis (machine frame)."""
        rot = coin.SbRotation(coin.SbVec3f(0.0, 0.0, 1.0), coin.SbVec3f(*map(float, tool_axis)))
        self.tool_xf.rotation.setValue(rot)
        self.tool_xf.translation.setValue(*map(float, tip))

    def set_part_mesh(self, verts, tris):
        self.partmesh_coords.point.setNum(len(verts))
        self.partmesh_coords.point.setValues(0, len(verts), np.asarray(verts).tolist())
        idx = _faces_index(np.asarray(tris))
        self.partmesh_faces.coordIndex.setNum(len(idx))
        self.partmesh_faces.coordIndex.setValues(0, len(idx), idx)

    def set_tool_mesh(self, verts, tris):
        """Tool geometry in tool-shape coordinates (control point at control_in_shape)."""
        self.tool_coords.point.setNum(len(verts))
        self.tool_coords.point.setValues(0, len(verts), [tuple(map(float, v)) for v in verts])
        idx = _faces_index(np.asarray(tris))
        self.tool_faces.coordIndex.setNum(len(idx))
        self.tool_faces.coordIndex.setValues(0, len(idx), idx)

    def set_tool_position(self, pos, control_in_shape):
        cx, cz = control_in_shape
        self.tool_xf.translation.setValue(float(pos[0] - cx), float(pos[1]), float(pos[2] - cz))

    def _set_lines(self, nodes, polylines, colours):
        mat, coords, lines = nodes
        pts = [tuple(map(float, p)) for pl in polylines for p in pl]
        coords.point.setNum(len(pts))
        coords.point.setValues(0, len(pts), pts)
        counts = [len(pl) for pl in polylines]
        lines.numVertices.setNum(len(counts))
        lines.numVertices.setValues(0, len(counts), counts)
        mat.diffuseColor.setNum(len(colours))
        mat.diffuseColor.setValues(0, len(colours), colours)

    def set_path(self, path_polylines):
        """path_polylines: Simulation.path_polylines() -> [(kind, op, pts (n, 3))]."""
        pls = [p for _k, _o, p in path_polylines if len(p) >= 2]
        cols = [RAPID_COLOUR if k == RAPID else FEED_COLOUR for k, _o, p in path_polylines if len(p) >= 2]
        self._set_lines(self.path_nodes, pls, cols)

    def set_part_section(self, polylines):
        pls = [[(x, 0.0, z) for x, z in pl] for pl in polylines if len(pl) >= 2]
        self._set_lines(self.part_nodes, pls, [PART_COLOUR] * len(pls))

    def show(self, stock=None, tool=None, path=None, part=None, part_mesh=None):
        for sw, on in ((self.stock_sw, stock), (self.tool_sw, tool), (self.path_sw, path), (self.part_sw, part),
                       (self.partmesh_sw, part_mesh)):
            if on is not None:
                _set_on(sw, on)

    def remove(self):
        try:
            self.view.getSceneGraph().removeChild(self.root)
        except Exception:
            pass


def outline_prism(outline, thickness=1.0):
    """Tool display fallback: the outline extruded from Y=-thickness to 0, in tool-shape coordinates."""
    p = outline.points + np.asarray(outline.control_in_shape)
    n = len(p)
    verts = [(x, 0.0, z) for x, z in p] + [(x, -thickness, z) for x, z in p]
    tris = []
    for i in range(1, n - 1):  # caps as fans (fine for the convex-ish inserts)
        tris.append((0, i, i + 1))
        tris.append((n, n + i + 1, n + i))
    for i in range(n):
        j = (i + 1) % n
        tris.append((i, j, n + j))
        tris.append((i, n + j, n + i))
    return np.asarray(verts), np.asarray(tris)
