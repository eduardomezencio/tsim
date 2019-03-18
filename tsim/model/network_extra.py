"""Extra network functions and classes."""

from itertools import chain
from typing import List

from tsim.model.entity import EntityIndex
from tsim.model.geometry import line_intersection, midpoint
from tsim.model.network import Node, Way

LANE_WIDTH = 2.5


def dissolve_node(index: EntityIndex, node: Node):
    """Remove a node joining the two ways it connects."""
    two_ways = len(node.starts) + len(node.ends) == 2
    loops = (node.starts and node.ends and
             node.starts[0].value is node.ends[0].value)
    if not two_ways or loops:
        raise ValueError(
            'Can only dissolve nodes connected to exactly two ways.')

    ways = [r.value for r in chain(node.ends, node.starts)]
    assert len(ways) == 2
    start, end = (w.other(node) for w in ways)

    if not node.level == start.level == end.level:
        raise ValueError('Can not dissolve nodes in different levels.')

    if not ways[0].lanes == ways[1].lanes:
        raise ValueError('Can not dissolve nodes with lane changes.')

    waypoints = []
    waypoints.extend(ways[0].waypoints if ways[0].end is node
                     else reversed(ways[0].waypoints))
    waypoints.append(node.position)
    waypoints.extend(ways[1].waypoints if ways[1].start is node
                     else reversed(ways[1].waypoints))

    index.delete(node)

    way = Way(start, end, lanes=ways[0].lanes, waypoints=tuple(waypoints))
    index.add(way)


class NodeGeometry:
    """Information on the geometric shape of a node.

    The points that form the shape of the node, calculated from the ways that
    are adjacent to this node.
    """

    __slots__ = ('node', 'way_indexes', 'way_distances', 'points')

    def __init__(self, node: Node):
        self.node = node
        ways = sorted((r.value for r in chain(node.starts, node.ends)),
                      key=lambda w: w.direction_from_node(node))
        self.way_indexes = {w: i for i, w in enumerate(ways)}
        self.way_distances = [None for _ in ways]
        self.points = self.way_distances * 2
        self._calc_points(ways)

    def _calc_points(self, ways: List[Way]):
        directions = tuple(w.direction_from_node(self.node).normalized()
                           for w in ways)
        for i, way in enumerate(ways):
            left = ways[i - 1]
            half_widths = tuple(w.total_lanes * LANE_WIDTH / 2
                                for w in (left, way))
            # Points relative to the node position.
            points = (directions[i - 1].rotated_left() * half_widths[0],
                      directions[i].rotated_right() * half_widths[1])
            try:
                point = line_intersection(points[0], directions[0],
                                          points[1], directions[1])
            except ZeroDivisionError:
                point = midpoint(*points)
            self.points[2 * i - 1] = point
        for i, (way, direction) in enumerate(zip(ways, directions)):
            farthest = max(self.points[2 * i - 1], self.points[2 * i + 1],
                           key=lambda v: v.norm())
            projection, reflection = farthest.projection_reflection(direction)
            self.way_distances[i] = projection.norm()
            self.points[2 * i] = reflection
