"""Grid class implementation."""

from math import floor

from panda3d.core import (Geom, GeomLines, GeomNode, GeomVertexData,
                          GeomVertexFormat, GeomVertexWriter, NodePath,
                          TransparencyAttrib)


class Grid:
    """A grid of lines for visual reference."""

    node_path: NodePath
    spacing: float
    size: float

    def __init__(self, spacing: float, size: float):
        self.spacing = spacing
        self.size = size
        self._create_geom()

    def _create_geom(self):
        radius = floor(self.size / (2 * self.spacing))
        diameter = (2 * radius + 1)
        vertex_count = diameter * 4
        start = -radius * self.spacing

        vertex_format = GeomVertexFormat.get_v3c4()
        vertex_data = GeomVertexData('grid', vertex_format, Geom.UH_static)
        vertex_data.set_num_rows(vertex_count)
        vertex_writer = GeomVertexWriter(vertex_data, 'vertex')
        color_writer = GeomVertexWriter(vertex_data, 'color')

        for i in range(diameter):
            vertex_writer.add_data3f(start + i * self.spacing, start, 0.0)
            vertex_writer.add_data3f(start + i * self.spacing, -start, 0.0)
            vertex_writer.add_data3f(start, start + i * self.spacing, 0.0)
            vertex_writer.add_data3f(-start, start + i * self.spacing, 0.0)
            for _ in range(4):
                color_writer.add_data4f(0.3, 0.3, 0.4, 0.5)

        primitive = GeomLines(Geom.UH_static)
        primitive.add_consecutive_vertices(0, vertex_count)
        primitive.close_primitive()
        geom = Geom(vertex_data)
        geom.add_primitive(primitive)
        self.geom = geom

    def attach_node(self, name: str, parent: NodePath):
        """Creates a grid node and attach it to the given node."""
        node = GeomNode(name)
        node.add_geom(self.geom)
        node_path = parent.attach_new_node(node)
        node_path.set_transparency(TransparencyAttrib.M_alpha)
        self.node_path = node_path
        return node_path
