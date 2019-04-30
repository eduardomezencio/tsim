"""Node object creation functions."""

from itertools import chain

from panda3d.core import (Geom, GeomNode, GeomTriangles, GeomTristrips,
                          GeomVertexData, GeomVertexFormat, GeomVertexWriter,
                          NodePath)

from tsim.model.network import LANE_WIDTH, Node
from tsim.ui import textures
from tsim.ui.constants import LEVEL_HEIGHT

VERTEX_FORMAT = GeomVertexFormat.get_v3t2()


def generate_mesh(node: Node) -> Geom:
    """Generate mesh for a Node."""
    size = len(node.geometry.points)
    vertex_data = GeomVertexData(str(node.id), VERTEX_FORMAT, Geom.UH_static)
    vertex_data.set_num_rows(size)
    vertex_writer = GeomVertexWriter(vertex_data, 'vertex')
    texcoord_writer = GeomVertexWriter(vertex_data, 'texcoord')

    indexes = [[j for j in range(i - 1, i + 2)]
               for i in range(0, size, 2)]
    triangles = [[node.geometry.points[j] for j in t] for t in indexes]

    for point in node.geometry.points:
        point = point + node.position
        vertex_writer.add_data3f(point.x, point.y, LEVEL_HEIGHT * node.level)
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

    return geom


def create(parent: NodePath, node: Node) -> NodePath:
    """Create node for given node and attach it to the parent."""
    geom = generate_mesh(node)
    node_ = GeomNode(str(node.id))
    node_.add_geom(geom)
    node_path = parent.attach_new_node(node_)
    node_path.set_texture(textures.get('intersection'))
    return node_path
