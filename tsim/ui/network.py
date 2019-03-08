"""Visualization of network objects."""

from math import cos, pi, sin

from panda3d.core import (Geom, GeomNode, GeomTristrips, GeomVertexData,
                          GeomVertexFormat, GeomVertexWriter)

from tsim.model.network import Way


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
        vertex_writer.add_data3f(2.0 * cos(angle), 2.0 * sin(angle), 0.0)
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
    for point, normal in zip(way.points(), way.normals()):
        for vertex in (point + 2.0 * normal, point - 2.0 * normal):
            vertex_writer.add_data3f(vertex.x, vertex.y, 0.0)
    primitive = GeomTristrips(Geom.UH_static)
    primitive.add_consecutive_vertices(0, 4 + 2 * len(way.waypoints))
    primitive.close_primitive()
    geom = Geom(vertex_data)
    geom.add_primitive(primitive)
    node = GeomNode(f'{name}_node')
    node.add_geom(geom)
    return node
