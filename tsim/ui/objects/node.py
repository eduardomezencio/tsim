"""Node object creation functions."""

from __future__ import annotations

from math import pi

from itertools import chain, cycle

import aggdraw
from panda3d.core import (CardMaker, Geom, GeomNode, GeomTriangles,
                          GeomTristrips, GeomVertexData, GeomVertexFormat,
                          GeomVertexWriter, NodePath, TransparencyAttrib)
from PIL import Image

from tsim.model.geometry import Vector
from tsim.model.network.intersection import ConflictPointType
from tsim.model.network.node import Node
from tsim.model.network.way import LANE_WIDTH
from tsim.ui import textures
from tsim.ui.objects.way import LEVEL_HEIGHT
from tsim.ui.textures import create_texture

CARD_MAKER = CardMaker('lane_connections_card_maker')
CARD_MAKER.set_frame((-16, 16, -16, 16))
COLORS = ('crimson', 'orange', 'gold', 'limegreen',
          'turquoise', 'deepskyblue', 'blueviolet', 'hotpink')
RESOLUTION = 1024
MIDDLE = Vector(RESOLUTION // 2, -RESOLUTION // 2)
PPM = RESOLUTION // 32
VERTEX_FORMAT = GeomVertexFormat.get_v3n3t2()


def create(parent: NodePath, node: Node) -> NodePath:
    """Create node for given node and attach it to the parent."""
    geom = _generate_mesh(node)
    node_ = GeomNode(str(node.id))
    node_.add_geom(geom)
    node_.adjust_draw_mask(0x00000000, 0x00010000, 0xfffeffff)
    node_path = parent.attach_new_node(node_)
    node_path.set_texture(textures.get('intersection'))
    return node_path


def create_lane_connections_card(node: Node, parent: NodePath) -> NodePath:
    """Create textured mesh showing lane connectins and attach it."""
    image = _create_lane_connections_image(node)
    texture = create_texture(image)

    card = parent.attach_new_node(CARD_MAKER.generate())
    card.set_pos((*node.position, 0.25))
    card.look_at(card, (0.0, 0.0, -1.0))
    card.set_texture(texture)
    card.set_transparency(TransparencyAttrib.M_alpha)
    card.set_shader_off()
    return card


def _generate_mesh(node: Node) -> Geom:
    """Generate mesh for a Node."""
    size = len(node.geometry.polygon)
    vertex_data = GeomVertexData(str(node.id), VERTEX_FORMAT, Geom.UH_static)
    vertex_data.set_num_rows(size)
    vertex_writer = GeomVertexWriter(vertex_data, 'vertex')
    normal_writer = GeomVertexWriter(vertex_data, 'normal')
    texcoord_writer = GeomVertexWriter(vertex_data, 'texcoord')

    indexes = [range(i - 1, i + 2) for i in range(0, size, 2)]
    triangles = [[node.geometry.polygon[j] for j in t] for t in indexes]

    for point in node.geometry.polygon:
        vertex_writer.add_data3f(point.x, point.y, LEVEL_HEIGHT * node.level)
        normal_writer.add_data3f(0.0, 0.0, 1.0)
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


def _create_lane_connections_image(node: Node) -> Image:
    """Generate image showing lane connections on a node."""
    image = Image.new('RGBA', (RESOLUTION, RESOLUTION))
    draw = aggdraw.Draw(image)

    colors = dict(zip(node.oriented_ways, cycle(COLORS)))

    for lanes, points in node.intersection.iterate_connections_curve_points():
        start, crossing, end = map(Vector.y_flipped,
                                   (p * PPM + MIDDLE for p in points))
        vector = lanes[1].way.direction_from_node(node, lanes[1].endpoint)
        vector = vector.normalized().rotated(pi * 1.125).y_flipped() * 32.0
        path = aggdraw.Path()
        path.moveto(*start)
        path.curveto(*start, *crossing, *end)
        path.lineto(*(end + vector))
        draw.path(path, aggdraw.Pen(colors[lanes[0].oriented_way.flipped()],
                                    0.25 * PPM, 224))

    pen = aggdraw.Pen('black', 1, 192)
    brush = {ConflictPointType.DIVERGE: aggdraw.Brush('green', 192),
             ConflictPointType.MERGE: aggdraw.Brush('yellow', 192),
             ConflictPointType.CROSSING: aggdraw.Brush('white', 192)}

    cpoints = set(p for _, p in
                  chain.from_iterable(c.conflict_points for c in
                                      node.intersection.curves.values()))
    for cpoint in cpoints:
        point = (cpoint.point * PPM + MIDDLE).y_flipped()
        draw.ellipse(point.enclosing_rect(0.25 * PPM), pen, brush[cpoint.type])

    pen = aggdraw.Pen('black', 1, 32)
    for cpoint in cpoints:
        for neighbor in cpoint.neighbors:
            points = ((p.point * PPM + MIDDLE).y_flipped()
                      for p in (cpoint, neighbor))
            draw.line(tuple(chain.from_iterable(points)), pen)

    draw.flush()
    # image.show()
    return image
