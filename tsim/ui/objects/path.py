"""Path visualization object creation functions."""

from __future__ import annotations

from itertools import islice
from typing import List

from panda3d.core import (Geom, GeomNode, GeomTristrips, GeomVertexData,
                          GeomVertexFormat, GeomVertexWriter, NodePath,
                          TransparencyAttrib)

from tsim.model.geometry import Point, sec
from tsim.model.network.lane import LANE_WIDTH
from tsim.ui.objects.way import LEVEL_HEIGHT
from tsim.utils.color import interpolate_rgb
from tsim.utils.iterators import window_iter

VERTEX_FORMAT = GeomVertexFormat.get_v3n3c4()
HEIGHT = LEVEL_HEIGHT + 1.0
SHIFT = LANE_WIDTH
START_COLOR = (0.2, 0.2, 1.0, 0.8)
END_COLOR = (1.0, 0.0, 0.6, 0.8)


def create(parent: NodePath, points: List[Point]) -> NodePath:
    """Create node for given path and attach it to the parent."""
    geom = _generate_mesh(points)
    node = GeomNode('path')
    node.add_geom(geom)
    node.adjust_draw_mask(0x00000000, 0x00010000, 0xfffeffff)
    node_path = parent.attach_new_node(node)
    node_path.set_transparency(TransparencyAttrib.M_alpha)
    return node_path


def _generate_mesh(points: List[Point]) -> Geom:
    """Generate mesh for a Path."""
    def calc_color(param):
        return interpolate_rgb(START_COLOR, END_COLOR, param)

    if len(points) < 2:
        return Geom()

    vertex_data = GeomVertexData('path', VERTEX_FORMAT, Geom.UH_static)
    vertex_data.set_num_rows(2 * len(points) + 1)
    vertex_writer = GeomVertexWriter(vertex_data, 'vertex')
    normal_writer = GeomVertexWriter(vertex_data, 'normal')
    color_writer = GeomVertexWriter(vertex_data, 'color')

    length = sum(p1.distance(p2) for p1, p2 in window_iter(points))
    vector = points[1] - points[0]
    distance = vector.norm()
    position = distance / length
    vector = vector.normalized()

    width_vector = LANE_WIDTH * 0.5 * vector.rotated_left()
    for vertex in (points[0] + width_vector, points[0] - width_vector):
        vertex_writer.add_data3f(vertex.x, vertex.y, HEIGHT)
        normal_writer.add_data3f(0.0, 0.0, 1.0)
        color_writer.add_data4f(*START_COLOR)

    last_vector = vector
    for point, next_ in zip(islice(points, 1, None), islice(points, 2, None)):
        vector = next_ - point
        distance = vector.norm()
        vector = vector.normalized()
        try:
            bisector = (last_vector + vector).normalized()
            width_vector = (sec(bisector, vector) * 0.5 * LANE_WIDTH
                            * bisector.rotated_left())
        except ZeroDivisionError:
            width_vector = vector.rotated_right() * 0.5 * LANE_WIDTH
        color = calc_color(position)
        for vertex in (point + width_vector, point - width_vector):
            vertex_writer.add_data3f(vertex.x, vertex.y, HEIGHT)
            normal_writer.add_data3f(0.0, 0.0, 1.0)
            color_writer.add_data4f(*color)
        position = position + distance / length
        last_vector = vector

    point = points[-1]
    width_vector = 0.5 * LANE_WIDTH * last_vector.rotated_left()

    distance = LANE_WIDTH if distance > LANE_WIDTH else distance / 2
    vector = -last_vector * distance

    for vertex in (point + width_vector + vector,
                   point - width_vector + vector,
                   point):
        vertex_writer.add_data3f(vertex.x, vertex.y, HEIGHT)
        normal_writer.add_data3f(0.0, 0.0, 1.0)
        color_writer.add_data4f(*END_COLOR)

    primitive = GeomTristrips(Geom.UH_static)
    primitive.add_consecutive_vertices(0, 2 * len(points) + 1)
    primitive.close_primitive()
    geom = Geom(vertex_data)
    geom.add_primitive(primitive)
    return geom
