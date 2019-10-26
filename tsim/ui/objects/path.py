"""Path visualization object creation functions."""

from __future__ import annotations

from itertools import islice

from panda3d.core import (Geom, GeomNode, GeomTristrips, GeomVertexData,
                          GeomVertexFormat, GeomVertexWriter, NodePath)

from tsim.model.geometry import sec
from tsim.model.network.path import Path
from tsim.model.network.way import LANE_WIDTH
from tsim.ui.objects.way import LEVEL_HEIGHT

VERTEX_FORMAT = GeomVertexFormat.get_v3n3c4()
HEIGHT = LEVEL_HEIGHT + 1.0


def create(parent: NodePath, path: Path) -> NodePath:
    """Create node for given path and attach it to the parent."""
    geom = _generate_mesh(path)
    node = GeomNode('path')
    node.add_geom(geom)
    node.adjust_draw_mask(0x00000000, 0x00010000, 0xfffeffff)
    node_path = parent.attach_new_node(node)
    return node_path


def _generate_mesh(path: Path) -> Geom:
    """Generate mesh for a Path."""
    points = list(path.points())

    if len(points) < 2:
        return Geom()

    vertex_data = GeomVertexData('path', VERTEX_FORMAT, Geom.UH_static)
    vertex_data.set_num_rows(2 * len(points) + 1)
    vertex_writer = GeomVertexWriter(vertex_data, 'vertex')
    normal_writer = GeomVertexWriter(vertex_data, 'normal')
    color_writer = GeomVertexWriter(vertex_data, 'color')

    length = path.length
    vector = points[1] - points[0]
    distance = vector.norm()
    position = distance / length
    vector = vector.normalized()

    color = [0.0, 0.0, 1.0, 1.0]

    width_vector = LANE_WIDTH * 0.5 * vector.rotated_left()
    for vertex in (points[0] + width_vector, points[0] - width_vector):
        vertex_writer.add_data3f(vertex.x, vertex.y, HEIGHT)
        normal_writer.add_data3f(0.0, 0.0, 1.0)
        color_writer.add_data4f(*color)

    last_vector = vector
    for point, next_ in zip(islice(points, 1, None), islice(points, 2, None)):
        vector = next_ - point
        distance = vector.norm()
        vector = vector.normalized()
        bisector = (last_vector + vector).normalized()
        width_vector = (sec(bisector, vector) * 0.5 * LANE_WIDTH
                        * bisector.rotated_left())
        for vertex in (point + width_vector, point - width_vector):
            vertex_writer.add_data3f(vertex.x, vertex.y, HEIGHT)
            normal_writer.add_data3f(0.0, 0.0, 1.0)
            color[0] = position ** 2
            color[1] = (1 - position) ** 2
            color_writer.add_data4f(*color)
        position = position + distance / length
        last_vector = vector

    point = points[-1]
    width_vector = 0.5 * LANE_WIDTH * last_vector.rotated_left()
    color[0] = 1.0
    color[1] = 0.0

    distance = LANE_WIDTH if distance > LANE_WIDTH else distance / 2
    vector = -last_vector * distance

    for vertex in (point + width_vector + vector,
                   point - width_vector + vector,
                   point):
        vertex_writer.add_data3f(vertex.x, vertex.y, HEIGHT)
        normal_writer.add_data3f(0.0, 0.0, 1.0)
        color_writer.add_data4f(*color)

    primitive = GeomTristrips(Geom.UH_static)
    primitive.add_consecutive_vertices(0, 2 * len(points) + 1)
    primitive.close_primitive()
    geom = Geom(vertex_data)
    geom.add_primitive(primitive)
    return geom
