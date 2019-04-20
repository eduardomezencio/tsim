"""Network related classes for tsim."""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
from itertools import accumulate, chain, islice, repeat
from math import sqrt
from typing import Generator, List, Tuple

from cached_property import cached_property
from dataslots import with_slots

from tsim.model.entity import Entity, EntityRef
from tsim.model.geometry import (BoundingRect, Point, Vector, distance,
                                 line_intersection, midpoint)
from tsim.model.index import INSTANCE as INDEX

LANE_WIDTH = 3.0


@with_slots
@dataclass(eq=False)
class Node(Entity):
    """A node of the network.

    A Node can be the endpoint of a Way or a junction of 3 or more Ways.
    """

    position: Point
    level: int = field(default_factory=int)
    starts: List[EntityRef['Way']] = field(default_factory=list)
    ends: List[EntityRef['Way']] = field(default_factory=list)

    @cached_property
    def geometry(self) -> NodeGeometry:
        """Get the geometry info of the node."""
        return NodeGeometry(self)

    @property
    def max_lanes(self) -> int:
        """Maximum number of lanes in incident ways."""
        return max(e.value.total_lanes for e in chain(self.starts, self.ends))

    @property
    def oriented_ways(self) -> Generator[Way.Oriented]:
        """Get incident ways with orentation (endpoint)."""
        return ((r.value, e) for r, e in chain(
            zip(self.starts, repeat(Way.Endpoint.START)),
            zip(self.ends, repeat(Way.Endpoint.END))))

    def calc_bounding_rect(self,
                           accumulated: BoundingRect = None) -> BoundingRect:
        """Calculate the bounding rect of the node."""
        return self.position.calc_bounding_rect(accumulated)

    def distance(self, point: Point, squared: bool = False) -> float:
        """Calculate distance from the node to a point."""
        if squared:
            return self.position.distance_squared(point)
        return self.position.distance(point)

    def sorted_ways(self) -> List[Way.Oriented]:
        """Get incident ways sorted in counterclockwise order."""
        return sorted(
            self.oriented_ways,
            key=lambda t: t[0].direction_from_node(self, t[1]).sorting_key())

    def ways_closest_to(self, oriented_way: Way.Oriented
                        ) -> Tuple[Way.Oriented, Way.Oriented]:
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

        if not ways[0].lanes == ways[1].lanes:
            raise ValueError('Can not dissolve nodes with lane changes.')

        waypoints = []
        waypoints.extend(ways[0].waypoints if ways[0].end is self
                         else reversed(ways[0].waypoints))
        waypoints.append(self.position)
        waypoints.extend(ways[1].waypoints if ways[1].start is self
                         else reversed(ways[1].waypoints))

        ways[0].disconnect(start)
        ways[1].disconnect(end)

        way = Way(start, end, lanes=ways[0].lanes, waypoints=tuple(waypoints))
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

    Endpoint = Enum('Endpoint', 'START END')
    Oriented = Tuple['Way', 'Endpoint']

    start: Node
    end: Node
    lanes: Tuple[int, int]
    waypoints: Tuple[Point] = field(default_factory=tuple)

    def __post_init__(self):
        self.start.starts.append(EntityRef(self))
        self.end.ends.append(EntityRef(self))

    @cached_property
    def length(self) -> float:
        """Total length of the Way."""
        return sum(self.distances())

    @property
    def one_way(self) -> bool:
        """Whether the way accepts traffic in only one direction.

        The direction of traffic is always from start to end, if one_way is
        true.
        """
        return not self.lanes[1]

    @property
    def total_lanes(self):
        """Total number of lanes."""
        return sum(self.lanes)

    @property
    def is_valid(self):
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

    def points(self, skip=0, reverse=False) -> Generator[Point]:
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

    def vectors(self) -> Generator[Vector]:
        """Get vectors for each edge on the way."""
        yield from ((p - q) for p, q in
                    zip(self.points(), self.points(skip=1)))

    def distances(self) -> Generator[float]:
        """Get generator for the distance between all consecutive points."""
        yield from (distance(p, q) for p, q in
                    zip(self.points(), self.points(skip=1)))

    def accumulated_length(self) -> Generator[float]:
        """Get generator for the accumulated length up to each point."""
        yield 0.0
        yield from accumulate(self.distances())

    def waypoint_normals(self) -> Generator[Vector]:
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

    def edge_normals(self) -> Generator[Vector]:
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
        right. Returns None if the given distance goes outside of the way.
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
                    return point + counter * vector.normalized() + side
        return None

    def disconnect(self, node: Node):
        """Disconnect this way from a node.

        Removes the connections from this way to  the given node and also the
        reference from the node to this way. Will make the way invalid.
        """
        if node is self.start:
            index = next(i for i, v in enumerate(self.start.starts)
                         if v.id == self.id)
            del self.start.starts[index]
            self.start = None

        if node is self.end:
            index = next(i for i, v in enumerate(self.end.ends)
                         if v.id == self.id)
            del self.end.ends[index]
            self.end = None

    def on_delete(self):
        """Disconnect this way from the network.

        Removes the connections from this way to nodes and also the references
        from the nodes to this way.
        """
        nodes = {self.start, self.end} - {None}
        for node in nodes:
            self.disconnect(node)

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

    def __init__(self, node: Node):
        self.node = node
        ways = node.sorted_ways()
        self.way_indexes = {w: i for i, (w, _) in enumerate(ways)}
        self.way_distances = [None for _ in ways]
        self.points = self.way_distances * 2
        self._calc_points(ways)

    def distance(self, way: Way):
        """Get distance from node center to where way should start or end."""
        return self.way_distances[self.way_indexes[way]]

    def _calc_points(self,
                     ways: List[Tuple[Way, Way.Endpoint]]):
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
