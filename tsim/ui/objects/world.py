"""World object creation functions.

Just a parent node for the other objects, with the ground plane mesh.
"""

from __future__ import annotations

from itertools import product

from panda3d.core import (DecalEffect, Geom, GeomNode, GeomTristrips,
                          GeomVertexData, GeomVertexFormat, GeomVertexWriter,
                          NodePath)

from tsim.ui import textures

VERTEX_FORMAT = GeomVertexFormat.get_v3n3t2()


def generate_mesh(radius: float, count: int) -> Geom:
    """Generate mesh for the ground plane."""
    vertex_data = GeomVertexData('ground', VERTEX_FORMAT, Geom.UH_static)
    vertex_data.set_num_rows(count ** 2)
    vertex_writer = GeomVertexWriter(vertex_data, 'vertex')
    normal_writer = GeomVertexWriter(vertex_data, 'normal')
    texcoord_writer = GeomVertexWriter(vertex_data, 'texcoord')

    step = 2 * radius / count
    for i, j in product(range(count + 1), repeat=2):
        vertex_writer.add_data3f(i * step - radius, j * step - radius, 0.0)
        normal_writer.add_data3f(0.0, 0.0, 1.0)
        texcoord_writer.add_data2f(i * 512, j * 512)

    geom = Geom(vertex_data)
    primitive = GeomTristrips(Geom.UH_static)
    for j in range(count):
        rows = range(count + 1) if j % 2 else reversed(range(count + 1))
        for i in rows:
            primitive.add_vertex((j + (j + 1) % 2) * (count + 1) + i)
            primitive.add_vertex((j + j % 2) * (count + 1) + i)
    primitive.close_primitive()
    geom.add_primitive(primitive)
    return geom


def create(parent: NodePath, radius: float, count: int) -> NodePath:
    """Create and add the ground plane to the scene."""
    geom = generate_mesh(radius, count)
    node = GeomNode('world')
    node.add_geom(geom)

    node_path = parent.attach_new_node(node)
    node_path.set_texture(textures.get('ground'))
    node_path.set_effect(DecalEffect.make())
    return node_path
