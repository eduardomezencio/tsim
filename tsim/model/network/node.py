"""`Node` and related classes."""

from __future__ import annotations

from dataclasses import dataclass, field
from itertools import chain, count, repeat
from typing import (ClassVar, Dict, Iterable, Iterator, List, Optional, Set,
                    Tuple)

from dataslots import with_slots

import tsim.model.index as Index
from tsim.model.entity import DeleteResult, Entity, EntityRef
from tsim.model.geometry import (BoundingRect, Point, Polygon, Vector,
                                 calc_bounding_rect, line_intersection,
                                 midpoint, point_in_polygon)
from tsim.model.network.intersection import Intersection, LaneConnection
from tsim.model.network.way import (LANE_WIDTH, Endpoint, LaneRef, OrientedWay,
                                    Way)
from tsim.utils.cached_property import cached_property


@with_slots
@dataclass(eq=False)
class Node(Entity):
    """A node of the network.

    A `Node` can be the endpoint of a `Way` or a junction of 3 or more ways. A
    node that connects two ways can be dissolved into a waypoint with the
    `dissolve` method, unless the two ways have different lane configurations
    and can't be merged.
    """

    cached: ClassVar[Iterable[str]] = (Entity.cached +
                                       ('geometry', 'intersection'))

    position: Point
    level: int = field(default_factory=int)
    starts: List[EntityRef[Way]] = field(default_factory=list)
    ends: List[EntityRef[Way]] = field(default_factory=list)

    @cached_property
    def geometry(self) -> NodeGeometry:
        """Get the geometry info for the node."""
        return NodeGeometry(self)

    @cached_property
    def intersection(self) -> Intersection:
        """Get intersection info for the node."""
        return Intersection(self)

    @property
    def neighbors(self) -> Iterable[Entity]:
        """Get iterable with entities directly connected to this node."""
        return self.ways

    @property
    def polygon(self) -> Polygon:
        """Get polygon from the node geometry."""
        return self.geometry.polygon

    @property
    def max_lanes(self) -> int:
        """Maximum number of lanes in incident ways."""
        return max(e.value.total_lane_count
                   for e in chain(self.starts, self.ends))

    @property
    def total_way_connections(self) -> int:
        """Get number of way connections to this node."""
        return len(self.starts) + len(self.ends)

    @property
    def oriented_ways(self) -> Iterator[OrientedWay]:
        """Get incident ways with orentation (endpoint)."""
        return (OrientedWay.build(r.value, e) for r, e in chain(
            zip(self.starts, repeat(Endpoint.START)),
            zip(self.ends, repeat(Endpoint.END))))

    @property
    def ways(self) -> Set[Way]:
        """Get all incident ways."""
        return {r.value for r in chain(self.starts, self.ends)}

    def calc_bounding_rect(self,
                           accumulated: BoundingRect = None) -> BoundingRect:
        """Calculate the bounding rect of the node."""
        return self.geometry.calc_bounding_rect(accumulated)

    def distance(self, point: Point, squared: bool = False) -> float:
        """Calculate distance from the node to a point."""
        if squared:
            return self.position.distance_squared(point)
        return self.position.distance(point)

    def way_connections(self, source: OrientedWay) -> Set[OrientedWay]:
        """Get the outgoing way connections coming from `source`."""
        return self.intersection.way_connections[source]

    def sorted_ways(self) -> List[OrientedWay]:
        """Get incident ways sorted in counterclockwise order."""
        return sorted(
            self.oriented_ways,
            key=lambda t: (t.way.direction_from_node(self, t.endpoint)
                           .sorting_key()))

    def ways_closest_to(self, oriented_way: OrientedWay) \
            -> Tuple[OrientedWay, OrientedWay]:
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

    def get_lane_connection(self, source: LaneRef, dest: OrientedWay) \
            -> Optional[LaneConnection]:
        """Get the lane connection leading to the given destination.

        The source is a LaneRef, representing the current position for an
        agent. The destination is an OrientedWay, considering there's no need
        to reach the destination way on a specific lane. This method tries to
        get a connection from the source lane, but may return a connection from
        a different lane if there is no connection from the given lane.

        Useful navigating a path that does not contain lane information.
        """
        return self.intersection.connection_map.get((source, dest), None)

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

        if ((same_dir and ways[0].lane_count != ways[1].lane_count)
                or (not same_dir
                    and ways[0].lane_count != ways[1].swapped_lane_count)):
            raise ValueError('Can not dissolve nodes with lane changes.')

        waypoints1 = list(ways[0].waypoints if ways[0].end is self
                          else reversed(ways[0].waypoints))
        waypoints2 = list(ways[1].waypoints if ways[1].start is self
                          else reversed(ways[1].waypoints))

        # Only add node as waypoint if it's far enough from neighbors
        if waypoints1 and waypoints2 and any(
                p1.close_to(p2, threshold=0.5)
                for p1, p2 in ((waypoints1[-1], self.position),
                               (waypoints2[0], self.position))):
            waypoint = []
        else:
            waypoint = [self.position]

        waypoints = chain(waypoints1, waypoint, waypoints2)

        ways[0].disconnect(start)
        ways[1].disconnect(end)

        lane_count = (ways[0].lane_count if ways[0].end is self
                      else ways[0].swapped_lane_count)
        way = Way(start, end, lane_count=lane_count,
                  waypoints=tuple(waypoints))
        way.xid = ways[0].xid if ways[0].xid is not None else ways[1].xid

        if delete_if_dissolved:
            Index.INSTANCE.delete(self)

    def clear_intersecting_waypoints(self) -> int:
        """Clear waypoints from incident ways intersecting the node polygon.

        With each waypoint removed, the geometry is updated for the node and
        for the incident ways. Returns the number of waypoints cleared.
        """
        def clear_waypoint():
            for way in self.ways:
                if not way.waypoints:
                    continue
                for index in (0, -1):
                    if point_in_polygon(way.waypoints[index], self.polygon):
                        waypoints = list(way.waypoints)
                        del waypoints[index]
                        way.waypoints = tuple(waypoints)
                        self.clear_cache(True)
                        return True
            return False

        for i in count():
            if not clear_waypoint():
                return i

    def on_delete(self) -> DeleteResult:
        """Disconnect this node from the network.

        Returns the ways that must be disconnected to free this node.
        """
        for way in map(lambda r: r.value, self.starts):
            way.start = None
        for way in map(lambda r: r.value, self.ends):
            way.end = None
        to_delete = {r.value for r in set(chain(self.starts, self.ends))}
        return DeleteResult(to_delete, ())

    def __repr__(self):
        return f'{Node.__name__}(id={self.id}, xid={self.xid})'


@with_slots
@dataclass(eq=False)
class NodeGeometry:
    """Information on the geometric shape of a node.

    The points that form the shape of the node, calculated from the ways that
    are adjacent to this node.
    """

    node: Node
    way_indexes: Dict[OrientedWay, int] = field(init=False)
    way_distances: List[float] = field(init=False)
    polygon: Polygon = field(init=False, default_factory=list)

    def __post_init__(self):
        ways = self.node.sorted_ways()
        self.way_indexes = {w: i for i, w in enumerate(ways)}
        self.way_distances = [None for _ in ways]
        self._build_polygon(ways)

    def distance(self, oriented_way: OrientedWay) -> float:
        """Get distance from node center to where way should start or end."""
        return self.way_distances[self.way_indexes[oriented_way]]

    def calc_bounding_rect(self,
                           accumulated: BoundingRect = None) -> BoundingRect:
        """Calculate the bounding rect of the node."""
        if not self.polygon:
            return self.node.position.calc_bounding_rect(accumulated)
        return calc_bounding_rect(self.polygon, accumulated)

    def _build_polygon(self, ways: List[OrientedWay]):
        """Calculate points for the geometric bounds of the node."""
        polygon = [None] * (2 * len(ways))
        directions = tuple(w().direction_from_node(self.node, e).normalized()
                           for w, e in ways)
        for i, (way, _) in enumerate(ways):
            half_widths = tuple(w().total_lane_count * LANE_WIDTH / 2
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

            polygon[2 * i - 1] = point
        for i, (_, direction) in enumerate(zip(ways, directions)):
            farthest: Vector = max(
                polygon[2 * i - 1], polygon[2 * i + 1],
                key=lambda v, d=direction: v.dot_product(d))
            projection, reflection = farthest.projection_reflection(direction)
            self.way_distances[i] = projection.norm()
            polygon[2 * i] = reflection
        self.polygon = [p + self.node.position for p in polygon]
