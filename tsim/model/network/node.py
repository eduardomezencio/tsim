"""Node and related classes."""

from __future__ import annotations

from collections import defaultdict, deque
from dataclasses import dataclass, field
from itertools import chain, combinations, islice, product, repeat
from math import pi
from typing import DefaultDict, Deque, Dict, Iterator, List, Set, Tuple

from cached_property import cached_property
from dataslots import with_slots

from tsim.model.entity import Entity, EntityRef
from tsim.model.geometry import (BoundingRect, Point, Vector, angle,
                                 line_intersection, midpoint)
from tsim.model.index import INSTANCE as INDEX
from tsim.model.network.way import LANE_WIDTH, Lane, OrientedWay, Way


@with_slots
@dataclass(eq=False)
class Node(Entity):
    """A node of the network.

    A Node can be the endpoint of a Way or a junction of 3 or more Ways.
    """

    position: Point
    level: int = field(default_factory=int)
    starts: List[EntityRef[Way]] = field(default_factory=list)
    ends: List[EntityRef[Way]] = field(default_factory=list)

    @cached_property
    def geometry(self) -> NodeGeometry:
        """Get the geometry info of the node."""
        return NodeGeometry(self)

    @cached_property
    def intersection(self) -> Intersection:
        """Lane connections on this node."""
        return Intersection(self)

    @cached_property
    def out_neighbors(self) -> Dict[Node, OrientedWay]:
        """Get the outgoing connections to other nodes.

        Each node that is an out-neighbor of this node is returned as a key in
        the dictionary, with the oriented way that connects to this node with
        minimum weight as the value.
        """
        neighbors: Dict[Node, OrientedWay] = {}
        for way in filter(lambda w: w.lanes, self.oriented_ways):
            other = way.way.other(self)
            existing = neighbors.get(other, None)
            if not existing or existing.weight < way.weight:
                neighbors[other] = way
        return neighbors

    @property
    def lane_connections_iter(self) -> Iterator[Tuple[Lane, Lane]]:
        """Lane connections as an iterator of lane tuples."""
        for src, values in self.intersection.connections.items():
            for dest in values:
                yield src, dest

    @property
    def max_lanes(self) -> int:
        """Maximum number of lanes in incident ways."""
        return max(e.value.total_lanes for e in chain(self.starts, self.ends))

    @property
    def total_way_connections(self) -> int:
        """Get number of way connections to this node."""
        return len(self.starts) + len(self.ends)

    @property
    def oriented_ways(self) -> Iterator[OrientedWay]:
        """Get incident ways with orentation (endpoint)."""
        return (OrientedWay(r.value, e) for r, e in chain(
            zip(self.starts, repeat(Way.Endpoint.START)),
            zip(self.ends, repeat(Way.Endpoint.END))))

    @property
    def ways(self) -> Set[Way]:
        """Get all incident ways."""
        return {r.value for r in chain(self.starts, self.ends)}

    def calc_bounding_rect(self,
                           accumulated: BoundingRect = None) -> BoundingRect:
        """Calculate the bounding rect of the node."""
        return self.position.calc_bounding_rect(accumulated)

    def reset_connections(self):
        """Invalidate geometry and lane connections."""
        for key in ('geometry', 'intersection'):
            try:
                del self.__dict__[key]
            except KeyError:
                pass

    def distance(self, point: Point, squared: bool = False) -> float:
        """Calculate distance from the node to a point."""
        if squared:
            return self.position.distance_squared(point)
        return self.position.distance(point)

    def sorted_ways(self) -> List[OrientedWay]:
        """Get incident ways sorted in counterclockwise order."""
        return sorted(
            self.oriented_ways,
            key=lambda t: (t.way.direction_from_node(self, t.endpoint)
                           .sorting_key()))

    def ways_closest_to(self, oriented_way: OrientedWay
                        ) -> Tuple[OrientedWay, OrientedWay]:
        """Get incident way closest to given way in each direction.

        Returns a tuple where the first value is the closest way in the
        clockwise direction and the second in the counterclockwise direction.
        If the way passed is the only way incident to the node, it will be
        returned as both values in the tuple. If there is only one other way,
        it will likewise be on both values.
        """
        sorted_ways = self.sorted_ways()
        index = sorted_ways.index(oriented_way)
        return (sorted_ways[index - 1],
                sorted_ways[(index + 1) % len(sorted_ways)])

    def dissolve(self, delete_if_dissolved=False):
        """Remove a node joining the two ways it connects."""
        two_ways = len(self.starts) + len(self.ends) == 2
        # pylint: disable=unsubscriptable-object
        loops = (self.starts and self.ends and
                 self.starts[0].value is self.ends[0].value)
        # pylint: enable=unsubscriptable-object
        if not two_ways or loops:
            raise ValueError(
                'Can only dissolve nodes connected to exactly two ways.')

        ways = [r.value for r in chain(self.ends, self.starts)]
        assert len(ways) == 2
        if any(not w.is_valid for w in ways):
            raise ValueError(
                'Can only dissolve nodes connected to valid ways.')

        start, end = (w.other(self) for w in ways)

        if not self.level == start.level == end.level:
            raise ValueError('Can not dissolve nodes in different levels.')

        same_dir = (((ways[0].end is self) and (ways[1].start is self)) or
                    ((ways[0].start is self) and (ways[1].end is self)))

        if ((same_dir and ways[0].lanes != ways[1].lanes) or
                (not same_dir and ways[0].lanes != ways[1].swapped_lanes)):
            raise ValueError('Can not dissolve nodes with lane changes.')

        waypoints = []
        waypoints.extend(ways[0].waypoints if ways[0].end is self
                         else reversed(ways[0].waypoints))
        waypoints.append(self.position)
        waypoints.extend(ways[1].waypoints if ways[1].start is self
                         else reversed(ways[1].waypoints))

        ways[0].disconnect(start)
        ways[1].disconnect(end)

        lanes = ways[0].lanes if ways[0].end is self else ways[0].swapped_lanes
        way = Way(start, end, lanes=lanes, waypoints=tuple(waypoints))
        way.xid = ways[0].xid if ways[0].xid is not None else ways[1].xid
        INDEX.add(way)

        if delete_if_dissolved:
            INDEX.delete(self)

    def on_delete(self):
        """Disconnect this node from the network.

        Returns the ways that must be disconnected to free this node.
        """
        for way in map(lambda r: r.value, self.starts):
            way.start = None
        for way in map(lambda r: r.value, self.ends):
            way.end = None
        return {r.value for r in set(chain(self.starts, self.ends))}


class NodeGeometry:
    """Information on the geometric shape of a node.

    The points that form the shape of the node, calculated from the ways that
    are adjacent to this node.
    """

    __slots__ = ('node', 'way_indexes', 'way_distances', 'points')

    node: Node
    way_indexes: Dict[OrientedWay, int]
    way_distances: List[float]
    points: List[Point]

    def __init__(self, node: Node):
        self.node = node
        ways = node.sorted_ways()
        self.way_indexes = {w: i for i, w in enumerate(ways)}
        self.way_distances = [None for _ in ways]
        self.points = self.way_distances * 2
        self._calc_points(ways)

    def distance(self, oriented_way: OrientedWay) -> float:
        """Get distance from node center to where way should start or end."""
        return self.way_distances[self.way_indexes[oriented_way]]

    def _calc_points(self, ways: List[OrientedWay]):
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
        for i, (_, direction) in enumerate(zip(ways, directions)):
            farthest: Vector = max(
                self.points[2 * i - 1], self.points[2 * i + 1],
                key=lambda v, d=direction: v.dot_product(d))
            projection, reflection = farthest.projection_reflection(direction)
            self.way_distances[i] = projection.norm()
            self.points[2 * i] = reflection


LaneConnections = Dict['Lane', Set['Lane']]


class Intersection:
    """Intersection information for a node.

    Contains lane connections and conflict points.
    """

    connections: LaneConnections

    __slots__ = ('connections',)

    def __init__(self, node: Node):
        self.connections = build_lane_connections(node)


def build_lane_connections(node: Node) -> LaneConnections:
    """Calculate default lane connections for the given node."""
    angles: List[float]
    connections: DefaultDict[Lane, Set[Lane]]
    new_conn: DefaultDict[Lane, Set[Lane]]
    way: OrientedWay
    ways: Deque[OrientedWay]

    def iterate_new_conn() -> Iterator[Tuple[Lane, Lane]]:
        for key, values in new_conn.items():
            yield from product((key,), values)

    def curvier(angle_a: float, angle_b: float) -> bool:
        return int(abs(pi - angle_a) < abs(pi - angle_b))

    def rotate_angles():
        angles[0] = 2 * pi
        size = len(angles)
        angles[0:] = [angles[i + 1 - size] - angles[1]
                      for i in range(size)]

    def self_connect():
        """Connect ways[0] to itself."""
        connections.update({l: {o} for l, o in zip(
            way.iterate_lanes(opposite_only=True),
            way.iterate_lanes(l_to_r=False))})

    def connect_to(index: int, other: OrientedWay) -> LaneConnections:
        """Connect ways[0] to the given way."""
        outwards = angles[index] > pi * 2.0 / 3.0
        for source, dest in zip(way.iterate_lanes(l_to_r=not(outwards),
                                                  opposite_only=True),
                                other.iterate_lanes(outwards)):
            new_conn[source].add(dest)
            conn_angles[(source, dest)] = angles[index]

    def resolve_conflicts():
        """Leave only one connection when there are conflicts."""
        to_remove = set()
        for conn_1, conn_2 in combinations(iterate_new_conn(), 2):
            if conn_1[0] == conn_2[0]:
                continue
            if conn_1 in to_remove or conn_2 in to_remove:
                continue
            conn = ((conn_2, conn_1) if conn_1[0].index > conn_2[0].index
                    else (conn_1, conn_2))
            angle_1, angle_2 = conn_angles[conn[0]], conn_angles[conn[1]]
            if angle_1 > angle_2:
                to_remove.add(conn[curvier(angle_1, angle_2)])
        for conn in to_remove:
            new_conn[conn[0]].remove(conn[1])

    connections = defaultdict(set)
    new_conn = defaultdict(set)
    conn_angles = {}
    ways = deque(node.sorted_ways())
    if ways:
        way = ways[0]
        if len(ways) == 1:
            self_connect()
        else:
            way_vector = way.way.direction_from_node(node, way.endpoint)

            angles = [angle(way_vector, w.direction_from_node(node, d))
                      if i > 0 else 0.0 for i, (w, d) in enumerate(ways)]

            for _ in range(len(ways)):
                new_conn.clear()
                conn_angles.clear()
                for i, way_ in islice(enumerate(ways), 1, None):
                    connect_to(i, way_)
                resolve_conflicts()
                connections.update(new_conn)
                ways.rotate(-1)
                way = ways[0]
                rotate_angles()

    return connections
