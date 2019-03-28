"""Extra network functions and classes."""

from itertools import chain
from typing import List, Tuple

from tsim.model.entity import EntityIndex
from tsim.model.geometry import line_intersection, midpoint
import tsim.model.network as network

LANE_WIDTH = 3.6


def dissolve_node(index: EntityIndex, node: 'network.Node'):
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

    way = network.Way(start, end, lanes=ways[0].lanes,
                      waypoints=tuple(waypoints))
    index.add(way)


class NodeGeometry:
    """Information on the geometric shape of a node.

    The points that form the shape of the node, calculated from the ways that
    are adjacent to this node.
    """

    __slots__ = ('node', 'way_indexes', 'way_distances', 'points')

    def __init__(self, node: 'network.Node'):
        self.node = node
        ways = node.sorted_ways()
        self.way_indexes = {w: i for i, (w, _) in enumerate(ways)}
        self.way_distances = [None for _ in ways]
        self.points = self.way_distances * 2
        self._calc_points(ways)

    def distance(self, way: 'network.Way'):
        """Get distance from node center to where way should start or end."""
        return self.way_distances[self.way_indexes[way]]

    def _calc_points(self,
                     ways: List[Tuple['network.Way', 'network.Way.Endpoint']]):
        """Calculate points for the geometric bounds of the node."""
        directions = tuple(w.direction_from_node(self.node, e).normalized()
                           for w, e in ways)
        for i, (way, _) in enumerate(ways):
            half_widths = tuple(w.total_lanes * LANE_WIDTH / 2
                                for w in (ways[i - 1][0], way))
            # Points relative to the node position.
            points = (directions[i - 1].rotated_left() * half_widths[0],
                      directions[i].rotated_right() * half_widths[1])
            try:
                point = line_intersection(points[0], directions[i - 1],
                                          points[1], directions[i])
                proportion = (points[0].distance(point)
                              / points[0].distance(points[1]))
                if ((len(directions) == 2 and proportion > 2.0)
                        or proportion > 5.0):
                    point = midpoint(*points)
            except ZeroDivisionError:
                point = midpoint(*points)

            self.points[2 * i - 1] = point
        for i, (way, direction) in enumerate(zip(ways, directions)):
            farthest = max(self.points[2 * i - 1], self.points[2 * i + 1],
                           key=lambda v, d=direction: v.dot_product(d))
            projection, reflection = farthest.projection_reflection(direction)
            self.way_distances[i] = projection.norm()
            self.points[2 * i] = reflection
