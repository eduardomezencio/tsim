"""Way object creation functions."""

from __future__ import annotations

from itertools import accumulate

from panda3d.core import (Geom, GeomNode, GeomTristrips, GeomVertexData,
                          GeomVertexFormat, GeomVertexWriter, NodePath)

from tsim.model.network.way import LANE_WIDTH, Way
from tsim.ui import textures

LEVEL_HEIGHT = 5.0
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
    geometry = way.geometry
    rows = 2 * len(geometry.segments) + 2

    vertex_data = GeomVertexData(str(way.id), VERTEX_FORMAT, Geom.UH_static)
    vertex_data.set_num_rows(rows)
    vertex_writer = GeomVertexWriter(vertex_data, 'vertex')
    normal_writer = GeomVertexWriter(vertex_data, 'normal')
    texcoord_writer = GeomVertexWriter(vertex_data, 'texcoord')

    start_z = way.start.level * LEVEL_HEIGHT
    end_z = way.end.level * LEVEL_HEIGHT
    lanes_float = float(way.total_lane_count)

    segment = geometry.segments[0]
    for vertex in (segment.start_left, segment.start_right):
        vertex_writer.add_data3f(vertex.x, vertex.y, start_z)
        normal_writer.add_data3f(0.0, 0.0, 1.0)
    texture_v = 0.0
    texcoord_writer.add_data2f(0.0, texture_v)
    texcoord_writer.add_data2f(lanes_float, texture_v)

    lengths = [s.length() for s in geometry.segments]
    total_len = sum(lengths)

    for segment, acc_len in zip(geometry.segments, accumulate(lengths)):
        height = start_z + (end_z - start_z) * acc_len / total_len
        for vertex in (segment.end_left, segment.end_right):
            vertex_writer.add_data3f(vertex.x, vertex.y, height)
            normal_writer.add_data3f(0.0, 0.0, 1.0)
        texture_v = acc_len / LANE_WIDTH
        texcoord_writer.add_data2f(0.0, texture_v)
        texcoord_writer.add_data2f(lanes_float, texture_v)

    primitive = GeomTristrips(Geom.UH_static)
    primitive.add_consecutive_vertices(0, rows)
    primitive.close_primitive()
    geom = Geom(vertex_data)
    geom.add_primitive(primitive)
    return geom
