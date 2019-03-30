"""Visualization of network objects."""

from dataclasses import astuple
from itertools import chain

from panda3d.core import (ConfigVariableColor, Geom, GeomNode, GeomTriangles,
                          GeomTristrips, GeomVertexData, GeomVertexFormat,
                          GeomVertexWriter, NodePath)

from tsim.model.entity import EntityIndex
from tsim.model.geometry import sec
from tsim.model.network import Node, Way
from tsim.model.network_extra import LANE_WIDTH
from tsim.ui import textures

LEVEL_HEIGHT = 5.0
WAY_COLOR = ConfigVariableColor('way-color')


def create_and_attach_nodes(index: EntityIndex, parent: NodePath):
    """Create and add network nodes to the scene."""
    nodes = ((f'node_{k}', v) for k, v in index.entities.items()
             if isinstance(v, Node))
    for name, node in nodes:
        node_ = build_node_geom_node(name, node)
        if node_ is not None:
            node_path = parent.attach_new_node(node_)
            node_path.set_texture(textures.get('intersection'))
            # node_path.set_color(WAY_COLOR)
            node_path.set_pos(*astuple(node.position),
                              LEVEL_HEIGHT * node.level)


def create_and_attach_ways(index: EntityIndex, parent: NodePath):
    """Create and add network ways to the scene."""
    ways = ((f'way_{k}', v) for k, v in index.entities.items()
            if isinstance(v, Way))
    for name, way in ways:
        node = build_way_geom_node(name, way)
        if node is not None:
            node_path = parent.attach_new_node(node)
            node_path.set_texture(textures.get('road'), 1)
            # node_path.set_color(WAY_COLOR)


def build_node_geom_node(name: str, node: Node) -> GeomNode:
    """Build a GeomNode for a network Node object."""
    size = len(node.geometry.points)
    vertex_format = GeomVertexFormat.get_v3t2()
    vertex_data = GeomVertexData(name, vertex_format, Geom.UH_static)
    vertex_data.set_num_rows(size)
    vertex_writer = GeomVertexWriter(vertex_data, 'vertex')
    texcoord_writer = GeomVertexWriter(vertex_data, 'texcoord')

    indexes = [[j for j in range(i - 1, i + 2)]
               for i in range(0, size, 2)]
    triangles = [[node.geometry.points[j] for j in t] for t in indexes]

    for point in node.geometry.points:
        vertex_writer.add_data3f(point.x, point.y, 0.0)
        texcoord_writer.add_data2f(point.x / LANE_WIDTH, point.y / LANE_WIDTH)

    indexes = [i for i, t in zip(indexes, triangles)
               if not t[0].close_to(t[1]) and not t[1].close_to(t[2])]

    geom = Geom(vertex_data)

    primitive = GeomTriangles(Geom.UH_static)
    for index in chain.from_iterable(indexes):
        primitive.add_vertex(index % size)
    primitive.close_primitive()
    geom.add_primitive(primitive)

    if size > 4:
        primitive = GeomTristrips(Geom.UH_static)
        for index in (i if i % 2 else -i - 1 for i in range(size // 2)):
            primitive.add_vertex(index % size)
        primitive.close_primitive()
        geom.add_primitive(primitive)

    node = GeomNode(f'{name}_node')
    node.add_geom(geom)
    return node


def build_way_geom_node(name: str, way: Way) -> GeomNode:
    """Build a GeomNode for a network Way object."""
    vertex_format = GeomVertexFormat.get_v3t2()
    vertex_data = GeomVertexData(name, vertex_format, Geom.UH_static)
    vertex_data.set_num_rows(4 + 2 * len(way.waypoints))
    vertex_writer = GeomVertexWriter(vertex_data, 'vertex')
    texcoord_writer = GeomVertexWriter(vertex_data, 'texcoord')

    total = way.length
    start_z = way.start.level * LEVEL_HEIGHT
    end_z = way.end.level * LEVEL_HEIGHT

    if total <= 0.0:
        return None

    half_width = LANE_WIDTH * way.total_lanes / 2
    lanes_float = float(way.total_lanes)

    vector = way.direction_from_node(way.start,
                                     Way.Endpoint.START).normalized()
    normal = vector.rotated_left()
    point = way.start.position + vector * way.start.geometry.distance(way)
    width_vector = half_width * normal
    for vertex in (point + width_vector, point - width_vector):
        vertex_writer.add_data3f(vertex.x, vertex.y, start_z)
    texture_v = 0.0
    texcoord_writer.add_data2f(0.0, texture_v)
    texcoord_writer.add_data2f(lanes_float, texture_v)

    vectors = way.vectors()
    last_vector = next(vectors)
    acc_len = abs(last_vector)
    for point, vector in zip(way.points(skip=1), vectors):
        bisector = last_vector.normalized() + vector.normalized()
        normal = bisector.rotated_left().normalized()
        width_vector = sec(bisector, vector) * half_width * normal
        height = start_z + (end_z - start_z) * acc_len / total
        for vertex in (point - width_vector, point + width_vector):
            vertex_writer.add_data3f(vertex.x, vertex.y, height)
        texture_v += abs(vector) / LANE_WIDTH
        texcoord_writer.add_data2f(0.0, texture_v)
        texcoord_writer.add_data2f(lanes_float, texture_v)
        acc_len += abs(vector)
        last_vector = vector

    vector = way.direction_from_node(way.end,
                                     Way.Endpoint.END)
    texture_v += abs(vector) / LANE_WIDTH
    vector = vector.normalized()
    normal = vector.rotated_right()
    point = way.end.position + vector * way.end.geometry.distance(way)
    width_vector = half_width * normal
    for vertex in (point + width_vector, point - width_vector):
        vertex_writer.add_data3f(vertex.x, vertex.y, end_z)
    texcoord_writer.add_data2f(0.0, texture_v)
    texcoord_writer.add_data2f(lanes_float, texture_v)

    primitive = GeomTristrips(Geom.UH_static)
    primitive.add_consecutive_vertices(0, 4 + 2 * len(way.waypoints))
    primitive.close_primitive()
    geom = Geom(vertex_data)
    geom.add_primitive(primitive)
    node = GeomNode(f'{name}_node')
    node.add_geom(geom)
    return node
