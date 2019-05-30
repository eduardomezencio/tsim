"""Network related classes for tsim."""

from __future__ import annotations

from collections import deque, defaultdict
from dataclasses import dataclass, field
from enum import Enum
from itertools import accumulate, chain, combinations, islice, repeat, product
from math import pi, sqrt
from typing import (DefaultDict, Deque, Dict, Iterator, List, NamedTuple, Set,
                    Tuple)

from cached_property import cached_property
from dataslots import with_slots

from tsim.model.entity import Entity, EntityRef
from tsim.model.geometry import (BoundingRect, Point, Vector, distance,
                                 line_intersection, midpoint, angle)
from tsim.model.index import INSTANCE as INDEX

DEFAULT_MAX_SPEED_KPH = 60.0
LANE_WIDTH = 3.0


@with_slots
@dataclass(eq=False)
class Node(Entity):
    """A node of the network.

    A Node can be the endpoint of a Way or a junction of 3 or more Ways.
    """

    LaneConnections = Dict['Lane', Set['Lane']]

    position: Point
    level: int = field(default_factory=int)
    starts: List[EntityRef['Way']] = field(default_factory=list)
    ends: List[EntityRef['Way']] = field(default_factory=list)

    @cached_property
    def geometry(self) -> NodeGeometry:
        """Get the geometry info of the node."""
        return NodeGeometry(self)

    @cached_property
    def lane_connections(self) -> Node.LaneConnections:
        """Lane connections on this node."""
        return self.calc_default_lane_connections()

    @property
    def lane_connections_iter(self) -> Iterator[Tuple[Lane, Lane]]:
        """Lane connections as an iterator of lane tuples."""
        for src, values in self.lane_connections.items():
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

    def calc_default_lane_connections(self) -> Node.LaneConnections:
        """Calculate default lane connections for this node."""
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

        def connect_to(index: int, other: OrientedWay) -> Node.LaneConnections:
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
        ways = deque(self.sorted_ways())
        if ways:
            way = ways[0]
            if len(ways) == 1:
                self_connect()
            else:
                way_vector = way.way.direction_from_node(self, way.endpoint)

                angles = [angle(way_vector, w.direction_from_node(self, d))
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

    def reset_connections(self):
        """Invalidate geometry and lane connections."""
        for key in ('geometry', 'lane_connections'):
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


@with_slots
@dataclass(eq=False)
class Way(Entity):
    """A connection between two Nodes.

    A Way connects two Nodes and can have a list of intermediary points, called
    waypoints.
    """

    class Endpoint(Enum):
        """Endpoints of a Way."""

        START = 0
        END = 1

        @property
        def other(self) -> Way.Endpoint:
            """Get the opposite endpoint."""
            return (Way.Endpoint.END if self is Way.Endpoint.START
                    else Way.Endpoint.START)

    start: Node
    end: Node
    lanes: Tuple[int, int]
    waypoints: Tuple[Point] = field(default_factory=tuple)
    max_speed: float = field(default_factory=lambda: DEFAULT_MAX_SPEED_KPH)

    def __post_init__(self):
        self.start.starts.append(EntityRef(self))
        self.start.reset_connections()
        self.end.ends.append(EntityRef(self))
        self.end.reset_connections()

    @cached_property
    def length(self) -> float:
        """Total length of the Way."""
        return sum(self.distances())

    @cached_property
    def weight(self) -> float:
        """Weight of the Way for path finding."""
        return self.length / self.max_speed

    @property
    def one_way(self) -> bool:
        """Whether the way accepts traffic in only one direction.

        The direction of traffic is always from start to end, if one_way is
        true.
        """
        return not self.lanes[1]

    @property
    def total_lanes(self) -> int:
        """Total number of lanes."""
        return sum(self.lanes)

    @property
    def swapped_lanes(self) -> Tuple[int, int]:
        """Get number of lanes, with swapped directions."""
        return self.lanes[1::-1]

    @property
    def is_valid(self) -> bool:
        """Check if way has start and end nodes."""
        return self.start is not None and self.end is not None

    def calc_bounding_rect(self,
                           accumulated: BoundingRect = None) -> BoundingRect:
        """Calculate the bounding rect of the way."""
        for point in self.points():
            accumulated = point.calc_bounding_rect(accumulated)
        return accumulated

    def distance(self, point: Point, squared: bool = False) -> float:
        """Calculate smallest distance from the way to a point."""
        result = min(point.distance_to_segment(s, v, squared=True)
                     for s, v in zip(self.points(), self.vectors()))
        return result if squared else sqrt(result)

    def other(self, node: Node):
        """Get the other endpoint of the way, opposite to node."""
        return self.start if self.end is node else self.end

    def points(self, skip=0, reverse=False) -> Iterator[Point]:
        """Get generator for points in order, including nodes and waypoints."""
        if reverse:
            # pylint: disable=bad-reversed-sequence
            iterator = chain((self.end.position,), reversed(self.waypoints),
                             (self.start.position,))
            # pylint: enable=bad-reversed-sequence
        else:
            iterator = chain((self.start.position,), self.waypoints,
                             (self.end.position,))
        yield from islice(iterator, skip, None)

    def vectors(self) -> Iterator[Vector]:
        """Get vectors for each edge on the way."""
        yield from ((q - p) for p, q in
                    zip(self.points(), self.points(skip=1)))

    def distances(self) -> Iterator[float]:
        """Get generator for the distance between all consecutive points."""
        yield from (distance(p, q) for p, q in
                    zip(self.points(), self.points(skip=1)))

    def accumulated_length(self) -> Iterator[float]:
        """Get generator for the accumulated length up to each point."""
        yield 0.0
        yield from accumulate(self.distances())

    def waypoint_normals(self) -> Iterator[Vector]:
        """Get the 'normals' of this way's waypoints.

        Normal here is used for the lack of a better term, meaning a unit
        vector pointing to the right side of the road. One is returned for each
        node or waypoint on the way.
        """
        # pylint: disable=unsubscriptable-object
        other = self.waypoints[0] if self.waypoints else self.end.position
        yield (other - self.start.position).rotated_left().normalized()

        window = zip(self.points(), self.points(skip=1), self.points(skip=2))
        yield from (((q - p).normalized() + (r - q).normalized())
                    .rotated_left().normalized()
                    for p, q, r in window)

        other = self.waypoints[-1] if self.waypoints else self.start.position
        yield (self.end.position - other).rotated_left().normalized()
        # pylint: enable=unsubscriptable-object

    def edge_normals(self) -> Iterator[Vector]:
        """Get the 'normals' of this way's edges.

        Get a unit vector pointing right from each edge on the way.
        """
        yield from ((p - q).rotated_left().normalized()
                    for p, q in zip(self.points(), self.points(skip=1)))

    def direction_from_node(self, node: Node,
                            priority: Endpoint = Endpoint.START) -> Vector:
        """Get vector in the direction from the node to this way.

        Get a vector pointing to the direction that this way touches the given
        node. The node must be one of the endpoints of this way. The priority
        argument is used in case of a loop, where the start and end nodes are
        the same. The direction given in this case will be the direction to the
        first way point if direction is START or to the last waypoint if
        direction is END.
        """
        if priority is Way.Endpoint.START and node is self.start:
            reverse = False
        else:
            reverse = node is self.end
            if not reverse and node is not self.start:
                raise ValueError('Node is not endpoint of this way.')
        point1, point2 = islice(self.points(reverse=reverse), 2)
        return point2 - point1

    def address_position(self, address: int) -> Point:
        """Get the position at given address relative to the way.

        The address is an integer and corresponds to a distance in meters from
        the start node. Even numbers are to the left and odd numbers to the
        right.
        """
        address = int(address)
        if address > 0:
            counter = float(address)
            for point, vector, length in zip(self.points(), self.vectors(),
                                             self.distances()):
                if counter > length:
                    counter -= length
                else:
                    vector = vector.normalized()
                    side = (vector.rotated_right() if address % 2
                            else vector.rotated_right()) * (LANE_WIDTH / 2.0)
                    return point + vector.normalized() * counter + side
        raise ValueError('Address outside of Way.')

    def iterate_lanes(self, direction: Way.Endpoint, l_to_r: bool = True,
                      include_opposite: bool = False,
                      opposite_only: bool = False) -> Iterator[Lane]:
        """Get lanes in the given direction.

        Get lanes in the given direction. Check Lane documentation for more
        information. If l_to_r is True (default), return lanes starting from
        the left, otherwise start from the right.
        """
        opposite = include_opposite or opposite_only
        lanes = range(
            -self.lanes[direction.other.value] if opposite else 0,
            0 if opposite_only else self.lanes[direction.value])

        for i in lanes if l_to_r else reversed(lanes):
            yield Lane(self, direction, i)

    def disconnect(self, node: Node):
        """Disconnect this way from a node.

        Removes the connections from this way to  the given node and also the
        reference from the node to this way. Will make the way invalid.
        """
        if node is self.start:
            index = next(i for i, v in enumerate(self.start.starts)
                         if v.id == self.id)
            del self.start.starts[index]
            node.reset_connections()
            self.start = None

        if node is self.end:
            index = next(i for i, v in enumerate(self.end.ends)
                         if v.id == self.id)
            del self.end.ends[index]
            node.reset_connections()
            self.end = None

    def on_delete(self):
        """Disconnect this way from the network.

        Removes the connections from this way to nodes and also the references
        from the nodes to this way.
        """
        nodes = {self.start, self.end} - {None}
        for node in nodes:
            self.disconnect(node)
            INDEX.updated(node)
            for way in node.ways:
                INDEX.updated(way)

        result = set()
        for node in nodes:
            if not node.starts and not node.ends:
                result.add(node)
            else:
                try:
                    node.dissolve()
                    result.add(node)
                except ValueError:
                    pass
        return result


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


class OrientedWay(NamedTuple):
    """A tuple containing a Way and an Endpoint.

    Used to represent a specific connection of a Way and a Node. Useful when
    referencing an incident Way from a Node, to avoid ambiguity, since both
    ends of a Way can be connected to the same Node. Can be interpreted as the
    given Way in the direction starting from the given Endpoint.
    """

    way: Way
    endpoint: Way.Endpoint

    def iterate_lanes(self, l_to_r: bool = True,
                      include_opposite: bool = False,
                      opposite_only: bool = False) -> Iterator[Lane]:
        """Get lanes in the direction of the oriented way.

        A shortcut for Way.iterate_lanes.
        """
        return self.way.iterate_lanes(self.endpoint, l_to_r, include_opposite,
                                      opposite_only)


class Lane(NamedTuple):
    """A tuple containing the same as OrientedWay, with a lane index.

    The two first elements are equivalent to the values of OrientedWay and the
    third value is the number of the lane. Lanes start from zero, meaning the
    leftmost lane if looking at the way in the direction of the two first
    values, interpreted as an OrientedWay, to n - 1, with n being the number of
    lanes in that direction.
    """

    way: Way
    endpoint: Way.Endpoint
    index: int

    @property
    def positive(self):
        """Get equivalent lane with positive index."""
        if self.index >= 0:
            return self
        return Lane(self.way, self.endpoint.other, -self.index - 1)

    @property
    def oriented_way(self):
        """Get oriented way from this lane."""
        return OrientedWay(self.way, self.endpoint)

    def distance_from_center(self) -> float:
        """Get distance to the right from way center to the lane."""
        return (1.0 - self.way.lanes[self.endpoint.value]
                + self.way.lanes[self.endpoint.other.value]) * LANE_WIDTH / 2
