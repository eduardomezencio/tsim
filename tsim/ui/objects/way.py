"""Way object creation functions."""

from __future__ import annotations

from panda3d.core import (ConfigVariableDouble, Geom, GeomNode, GeomTristrips,
                          GeomVertexData, GeomVertexFormat, GeomVertexWriter,
                          NodePath)

from tsim.model.geometry import sec
from tsim.model.network.way import LANE_WIDTH, OrientedWay, Way
from tsim.ui import textures

LEVEL_HEIGHT = ConfigVariableDouble('level-height').get_value()
VERTEX_FORMAT = GeomVertexFormat.get_v3n3t2()


def create(parent: NodePath, way: Way) -> NodePath:
    """Create node for given way and attach it to the parent."""
    geom = _generate_mesh(way)
    node = GeomNode(str(way.id))
    node.add_geom(geom)
    node.adjust_draw_mask(0x00000000, 0x00010000, 0xfffeffff)
    node_path = parent.attach_new_node(node)
    node_path.set_texture(textures.get('road'), 1)
    return node_path


def _generate_mesh(way: Way) -> Geom:
    """Generate mesh for a Way."""
    vertex_data = GeomVertexData(str(way.id), VERTEX_FORMAT, Geom.UH_static)
    vertex_data.set_num_rows(4 + 2 * len(way.waypoints))
    vertex_writer = GeomVertexWriter(vertex_data, 'vertex')
    normal_writer = GeomVertexWriter(vertex_data, 'normal')
    texcoord_writer = GeomVertexWriter(vertex_data, 'texcoord')

    total = way.length
    start_z = way.start.level * LEVEL_HEIGHT
    end_z = way.end.level * LEVEL_HEIGHT

    if total <= 0.0:
        return None

    half_width = LANE_WIDTH * way.total_lane_count / 2
    lanes_float = float(way.total_lane_count)

    vector = way.direction_from_node(way.start,
                                     Way.Endpoint.START).normalized()
    point = way.start.position + vector * way.start.geometry.distance(
        OrientedWay.build(way, Way.Endpoint.START))
    width_vector = half_width * vector.rotated_left()
    for vertex in (point + width_vector, point - width_vector):
        vertex_writer.add_data3f(vertex.x, vertex.y, start_z)
        normal_writer.add_data3f(0.0, 0.0, 1.0)
    texture_v = 0.0
    texcoord_writer.add_data2f(0.0, texture_v)
    texcoord_writer.add_data2f(lanes_float, texture_v)

    vectors = way.vectors()
    last_vector = next(vectors)
    acc_len = abs(last_vector)
    for point, vector in zip(way.points(skip=1), vectors):
        bisector = last_vector.normalized() + vector.normalized()
        width_vector = (sec(bisector, vector) * half_width
                        * bisector.rotated_left().normalized())
        height = start_z + (end_z - start_z) * acc_len / total
        for vertex in (point + width_vector, point - width_vector):
            vertex_writer.add_data3f(vertex.x, vertex.y, height)
            normal_writer.add_data3f(0.0, 0.0, 1.0)
        texture_v += abs(vector) / LANE_WIDTH
        texcoord_writer.add_data2f(0.0, texture_v)
        texcoord_writer.add_data2f(lanes_float, texture_v)
        acc_len += abs(vector)
        last_vector = vector

    vector = way.direction_from_node(way.end, Way.Endpoint.END)
    texture_v += abs(vector) / LANE_WIDTH
    vector = vector.normalized()
    point = way.end.position + vector * way.end.geometry.distance(
        OrientedWay.build(way, Way.Endpoint.END))
    width_vector = half_width * vector.rotated_right()
    for vertex in (point + width_vector, point - width_vector):
        vertex_writer.add_data3f(vertex.x, vertex.y, end_z)
        normal_writer.add_data3f(0.0, 0.0, 1.0)
    texcoord_writer.add_data2f(0.0, texture_v)
    texcoord_writer.add_data2f(lanes_float, texture_v)

    primitive = GeomTristrips(Geom.UH_static)
    primitive.add_consecutive_vertices(0, 4 + 2 * len(way.waypoints))
    primitive.close_primitive()
    geom = Geom(vertex_data)
    geom.add_primitive(primitive)
    return geom
