"""Node and related classes."""

from __future__ import annotations

from dataclasses import dataclass, field
from itertools import chain, repeat
from typing import (TYPE_CHECKING, Dict, Iterable, Iterator, List, NamedTuple,
                    Set, Tuple)

from cached_property import cached_property
from dataslots import with_slots

import tsim.model.index as Index
from tsim.model.entity import Entity, EntityRef
from tsim.model.geometry import (BoundingRect, Point, Vector,
                                 line_intersection, midpoint)
from tsim.model.network.intersection import Intersection
from tsim.model.network.way import LANE_WIDTH, Lane, OrientedWay, Way

if TYPE_CHECKING:
    from tsim.model.network.intersection import Curve, LaneConnection


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
        return (OrientedWay.build(r.value, e) for r, e in chain(
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
        for key in ('geometry', 'intersection', 'out_neighbors'):
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
        loops = (self.starts and self.ends and
                 self.starts[0].value is self.ends[0].value)

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
        Index.INSTANCE.add(way)

        if delete_if_dissolved:
            Index.INSTANCE.delete(self)

    def on_delete(self) -> Iterable[Entity]:
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
        directions = tuple(w().direction_from_node(self.node, e).normalized()
                           for w, e in ways)
        for i, (way, _) in enumerate(ways):
            half_widths = tuple(w().total_lanes * LANE_WIDTH / 2
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


class NodeLaneConnection(NamedTuple):
    """Information about a lane connection on a node.

    Contains the node itself and a tuple with the source and destination lanes.
    Can be used to identify the position of something on this path.
    """

    node: Node
    lanes: LaneConnection

    @property
    def curve(self) -> Curve:
        """Get the curve of this lane connection."""
        return self.node.intersection.curves[self.lanes]
