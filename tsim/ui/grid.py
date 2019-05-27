"""Grid class implementation."""

from itertools import product
from math import floor

from panda3d.core import (ConfigVariableColor, Geom, GeomLinestrips, GeomNode,
                          GeomVertexData, GeomVertexFormat, GeomVertexWriter,
                          NodePath, TransparencyAttrib)

from tsim.model.geometry import Vector


class Grid:
    """A grid of lines for visual reference."""

    focus: NodePath
    node_path: NodePath
    spacing: float
    size: float

    def __init__(self, spacing: float, size: float, parent: NodePath,
                 focus: NodePath):
        self.spacing = spacing
        self.size = size
        self.focus = focus
        self._create_geom()

        node = GeomNode('grid')
        node.add_geom(self.geom)
        node.adjust_draw_mask(0x00000000, 0x00010000, 0xfffeffff)
        self.node_path = parent.attach_new_node(node)
        self.node_path.set_z(0.1)
        self.node_path.set_transparency(TransparencyAttrib.M_alpha)
        self.node_path.set_shader_off()

    def update(self):
        """Update callback."""
        x, y = self.focus.get_x(), self.focus.get_y()
        self.node_path.set_x(x - x % self.spacing)
        self.node_path.set_y(y - y % self.spacing)

    def _create_geom(self):
        color = ConfigVariableColor('grid-color')

        radius = floor(self.size / (2 * self.spacing))
        diameter = (2 * radius + 1)
        start = -radius * self.spacing

        vertex_format = GeomVertexFormat.get_v3c4()
        vertex_data = GeomVertexData('grid', vertex_format, Geom.UH_static)
        vertex_data.set_num_rows(diameter * 4)
        vertex_writer = GeomVertexWriter(vertex_data, 'vertex')
        color_writer = GeomVertexWriter(vertex_data, 'color')

        for i, j in product(range(diameter), repeat=2):
            vertex_writer.add_data3f(start + i * self.spacing,
                                     start + j * self.spacing, 0.0)
            alpha = 0.5 - (Vector(i - radius, j - radius).norm() / radius)
            color_writer.add_data4f(color[0], color[1], color[2], alpha)

        primitive = GeomLinestrips(Geom.UH_static)
        for vertex in vertex_indexes(diameter):
            primitive.add_vertex(vertex)
        primitive.close_primitive()
        self.geom = Geom(vertex_data)
        self.geom.add_primitive(primitive)


def vertex_indexes(diameter):
    """Get indexes for linestrip with given diameter."""
    yield from (x + y * diameter for x, y in vertex_coords(diameter))


def vertex_coords(diameter):
    """Get coordinates of vertexes for linestrip with given diameter."""
    for x in range(0, diameter - 2, 2):
        i, j = 0, 1
        for y in range(diameter):
            yield from ((x + i, y), (x + j, y))
            i, j = j, i
        i, j = 1, 2
        for y in reversed(range(1, diameter - 1)):
            yield from ((x + i, y), (x + j, y))
            i, j = j, i
