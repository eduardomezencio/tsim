"""Visualization of network objects."""

from dataclasses import astuple
from itertools import zip_longest
from math import cos, pi, sin

from panda3d.core import (ConfigVariableColor, Geom, GeomNode, GeomTristrips,
                          GeomVertexData, GeomVertexFormat, GeomVertexWriter,
                          LODNode, NodePath)

from tsim.model.entity import EntityIndex
from tsim.model.geometry import sec
from tsim.model.network import Node, Way

LEVEL_HEIGHT = 5.0
LANE_WIDTH = 2.5


def create_and_attach_nodes(index: EntityIndex, parent: NodePath):
    """Create and add network nodes to the scene."""
    geoms = (build_node_geom(16), build_node_geom(8))
    nodes = (v for v in index.entities.values() if isinstance(v, Node))
    for node in nodes:
        geom_nodes = [GeomNode('node{node.id}_{i}_node') for i in range(2)]
        for geom, geom_node in zip(geoms, geom_nodes):
            geom_node.add_geom(geom)
        node_paths = [NodePath(g) for g in geom_nodes]
        lod = LODNode(f'node{node.id}_lod')
        lod_np = NodePath(lod)
        lod_np.reparent_to(parent)
        levels = [(250.0, 0.0), (1000.0, 250.0)]
        for bounds, node_path in zip(levels, node_paths):
            lod.add_switch(*bounds)
            node_path.set_color(ConfigVariableColor('way-color'))
            node_path.reparent_to(lod_np)
        lod_np.set_pos(*astuple(node.position), LEVEL_HEIGHT * node.level)


def create_and_attach_ways(index: EntityIndex, parent: NodePath):
    """Create and add network ways to the scene."""
    ways = ((f'way_{k}', v) for k, v in index.entities.items()
            if isinstance(v, Way))
    for name, way in ways:
        node = build_way_geom_node(name, way)
        if node is not None:
            node_path = parent.attach_new_node(node)
            node_path.set_color(ConfigVariableColor('way-color'))


def build_node_geom(vertex_count: int = 16) -> Geom:
    """Build the geom object for the network nodes."""
    def vertex_order(num):
        for i in range(num // 2):
            yield (num - i) % num
            yield i + 1

    vertex_format = GeomVertexFormat.get_v3()
    vertex_data = GeomVertexData('node', vertex_format, Geom.UH_static)
    vertex_data.set_num_rows(vertex_count)
    vertex_writer = GeomVertexWriter(vertex_data, 'vertex')
    for i in range(vertex_count):
        angle = i * 2 * pi / vertex_count
        vertex_writer.add_data3f(LANE_WIDTH * cos(angle),
                                 LANE_WIDTH * sin(angle), 0.0)
    primitive = GeomTristrips(Geom.UH_static)
    for i in vertex_order(vertex_count):
        primitive.add_vertex(i)
    primitive.close_primitive()
    geom = Geom(vertex_data)
    geom.add_primitive(primitive)
    return geom


def build_way_geom_node(name: str, way: Way) -> GeomNode:
    """Build a GeomNode for a network Way object."""
    vertex_format = GeomVertexFormat.get_v3()
    vertex_data = GeomVertexData(name, vertex_format, Geom.UH_static)
    vertex_data.set_num_rows(4 + 2 * len(way.waypoints))
    vertex_writer = GeomVertexWriter(vertex_data, 'vertex')

    total = way.length
    start_z = way.start.level * LEVEL_HEIGHT
    delta_z = way.end.level * LEVEL_HEIGHT - start_z

    if total <= 0.0:
        return None

    acc_len = 0.0
    last_vector = next(way.vectors())
    for point, vector in zip_longest(way.points(), way.vectors()):
        if vector is None:
            vector = last_vector
        bisector = last_vector.normalized() + vector.normalized()
        normal = bisector.rotated_right().normalized()
        width_vector = (sec(bisector, vector) * LANE_WIDTH * normal
                        * way.total_lanes)
        height = start_z + delta_z * acc_len / total
        for vertex in (point - width_vector, point + width_vector):
            vertex_writer.add_data3f(vertex.x, vertex.y, height)
        acc_len += abs(vector)
        last_vector = vector

    primitive = GeomTristrips(Geom.UH_static)
    primitive.add_consecutive_vertices(0, 4 + 2 * len(way.waypoints))
    primitive.close_primitive()
    geom = Geom(vertex_data)
    geom.add_primitive(primitive)
    node = GeomNode(f'{name}_node')
    node.add_geom(geom)
    return node
