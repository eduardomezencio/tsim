"""Node object creation functions."""

from itertools import chain, cycle, product

from panda3d.core import (CardMaker, Geom, GeomNode, GeomTriangles,
                          GeomTristrips, GeomVertexData, GeomVertexFormat,
                          GeomVertexWriter, NodePath, TransparencyAttrib)
from PIL import Image
import aggdraw

from tsim.model.geometry import Vector, line_intersection_safe
from tsim.model.network import LANE_WIDTH, Lane, Node
from tsim.ui import textures
from tsim.ui.constants import LEVEL_HEIGHT
from tsim.ui.textures import create_texture

COLORS = ('crimson', 'orange', 'gold', 'limegreen',
          'turquoise', 'deepskyblue', 'blueviolet', 'hotpink')
PPM = 32

VERTEX_FORMAT = GeomVertexFormat.get_v3t2()

CARD_MAKER = CardMaker('lane_connections_card_maker')
CARD_MAKER.set_frame((-16, 16, -16, 16))


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


def create_lane_connections_image(node: Node) -> Image:
    """Generate image showing lane connections on a node."""
    image = Image.new('RGBA', (1024, 1024))
    draw = aggdraw.Draw(image)

    middle = Vector(512, -512)
    colors = {w: c for w, c in zip(node.oriented_ways, cycle(COLORS))}
    vectors = {w: w.way.direction_from_node(node, w.endpoint).normalized()
               for w in node.oriented_ways}

    points = {}
    for way in node.oriented_ways:
        vector = vectors[way] * PPM
        right = vector.rotated_right()
        first = right * Lane(*way, 0).distance_from_center()
        for lane in way.iterate_lanes(include_opposite=True):
            points[lane] = (
                vector * (node.geometry.distance(way) + LANE_WIDTH * 0.5)
                + right * lane.index * LANE_WIDTH
                + first + middle)

    crossings = {}
    for source, dests in node.lane_connections.items():
        for lane1, lane2 in product((source,), dests):
            crossings[(lane1, lane2)] = line_intersection_safe(
                points[lane1], vectors[lane1.oriented_way],
                points[lane2], vectors[lane2.oriented_way])

    for lanes, crossing in crossings.items():
        start, end = map(points.get, lanes)
        start, crossing, end = map(Vector.y_flipped, (start, crossing, end))
        path = aggdraw.Path()
        path.moveto(*start)
        path.curveto(*start, *crossing, *end)
        draw.path(path, aggdraw.Pen(colors[lanes[0].oriented_way], 12))

    draw.flush()
    # image.show()
    return image


def create_lane_connections_card(node: Node, parent: NodePath) -> NodePath:
    """Create textured mesh showing lane connectins and attach it."""
    image = create_lane_connections_image(node)
    texture = create_texture(image)

    card = parent.attach_new_node(CARD_MAKER.generate())
    card.set_pos((*node.position, 0.25))
    card.look_at(card, (0.0, 0.0, -1.0))
    card.set_texture(texture)
    card.set_transparency(TransparencyAttrib.M_alpha)
    return card
