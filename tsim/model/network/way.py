"""Way and related classes."""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
from itertools import accumulate, chain, islice
from math import sqrt
from typing import (TYPE_CHECKING, Iterable, Iterator, List, NamedTuple,
                    Optional, Tuple)

from cached_property import cached_property
from dataslots import with_slots

import tsim.model.index as Index
from tsim.model.entity import Entity, EntityRef
from tsim.model.geometry import BoundingRect, Point, Vector, sec
from tsim.utils.iterators import window_iter

if TYPE_CHECKING:
    from tsim.model.network.node import Node

DEFAULT_MAX_SPEED_KPH = 60.0
LANE_WIDTH = 3.0
HALF_LANE_WIDTH = 0.5 * LANE_WIDTH


class Endpoint(Enum):
    """Endpoints of a Way."""

    START = 0
    END = 1

    @property
    def other(self) -> Endpoint:
        """Get the opposite endpoint."""
        return Endpoint.END if self is Endpoint.START else Endpoint.START


@with_slots
@dataclass(eq=False)
class Way(Entity):
    """A connection between two Nodes.

    A Way connects two Nodes and can have a list of intermediary points, called
    waypoints.
    """

    start: Node
    end: Node
    lane_count: Tuple[int, int]
    waypoints: Tuple[Point] = field(default_factory=tuple)
    max_speed: float = field(default_factory=lambda: DEFAULT_MAX_SPEED_KPH)

    def __post_init__(self):
        self.start.starts.append(EntityRef(self))
        self.start.reset_connections()
        self.end.ends.append(EntityRef(self))
        self.end.reset_connections()

    @cached_property
    def geometry(self) -> WayGeometry:
        """Get the geometry info for the way."""
        return WayGeometry(self)

    @cached_property
    def length(self) -> float:
        """Total length of the Way."""
        return sum(self.distances())

    @cached_property
    def weight(self) -> Tuple[float, float]:
        """Weight of the Way in each direction, for path finding."""
        weight = self.length / self.max_speed
        return (weight, weight)

    @property
    def one_way(self) -> bool:
        """Whether the way accepts traffic in only one direction.

        The direction of traffic is always from start to end, if one_way is
        true.
        """
        return not self.lane_count[1]

    @property
    def total_lane_count(self) -> int:
        """Total number of lanes."""
        return sum(self.lane_count)

    @property
    def swapped_lane_count(self) -> Tuple[int, int]:
        """Get number of lanes, with swapped directions."""
        return self.lane_count[1::-1]

    @property
    def segment_count(self) -> int:
        """Get number of segments that compose this way."""
        return len(self.waypoints) + 1

    @property
    def is_valid(self) -> bool:
        """Check if way has both start and end nodes."""
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

    def other(self, node: Node) -> Node:
        """Get the other endpoint of the way, opposite to node."""
        return self.start if self.end is node else self.end

    def oriented(self, endpoint: Endpoint = Endpoint.START) -> OrientedWay:
        """Get OrientedWay from this way."""
        return OrientedWay.build(self, endpoint)

    def points(self, skip=0, reverse=False) -> Iterator[Point]:
        """Get generator for points in order, including nodes and waypoints."""
        if reverse:
            iterator = chain((self.end.position,), reversed(self.waypoints),
                             (self.start.position,))
        else:
            iterator = chain((self.start.position,), self.waypoints,
                             (self.end.position,))
        yield from islice(iterator, skip, None)

    def point_pairs(self, reverse=False) -> Iterator[Tuple[Point, Point]]:
        """Get pairs of consecutive points, including nodes and waypoints."""
        return window_iter(self.points(reverse=reverse))

    def vectors(self, reverse=False) -> Iterator[Vector]:
        """Get vectors for each edge on the way."""
        yield from (q - p for p, q in
                    self.point_pairs(reverse=reverse))

    def distances(self, reverse=False) -> Iterator[float]:
        """Get generator for the distance between all consecutive points."""
        yield from (p.distance(q) for p, q in
                    self.point_pairs(reverse=reverse))

    def accumulated_length(self) -> Iterator[float]:
        """Get generator for the accumulated length up to each point."""
        yield 0.0
        yield from accumulate(self.distances())

    def waypoint_normals(self, reverse: bool = False) -> Iterator[Vector]:
        """Get the 'normals' of this way's waypoints.

        Normal here is used for the lack of a better term, meaning a unit
        vector pointing to the right side of the road. One is returned for each
        waypoint.
        """
        yield from (((q - p).normalized() + (r - q).normalized())
                    .rotated_right().normalized()
                    for p, q, r in window_iter(self.points(reverse=reverse),
                                               size=3))

    def edge_normals(self, reverse: bool = False) -> Iterator[Vector]:
        """Get the 'normals' of this way's edges.

        Get a unit vector pointing right from each edge on the way.
        """
        yield from ((q - p).rotated_right().normalized()
                    for p, q in self.point_pairs(reverse=reverse))

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
        if priority is Endpoint.START and node is self.start:
            reverse = False
        else:
            reverse = node is self.end
            if not reverse and node is not self.start:
                raise ValueError('Node is not endpoint of this way.')
        point1, point2 = islice(self.points(reverse=reverse), 2)
        return point2 - point1

    def lane_distance_from_center(
            self, lane: int, endpoint: Endpoint = Endpoint.START) -> float:
        """Get distance to the right from way center to the lane."""
        return (1 + lane
                - self.lane_count[endpoint.value]
                + self.lane_count[endpoint.other.value]) * HALF_LANE_WIDTH

    def get_position(self, distance: float,
                     endpoint: Endpoint = Endpoint.START,
                     lane: int = 0) -> Point:
        """Get the position at given location in this way.

        The distance is in meters from the given endpoint, being START the
        default value. The position will be on the given lane.
        """
        if distance >= 0.0:
            counter = distance
            reverse = endpoint is Endpoint.END
            for point1, point2 in self.point_pairs(reverse=reverse):
                length = point1.distance(point2)
                if counter > length:
                    counter -= length
                else:
                    vector = (point2 - point1).normalized()
                    side = (vector.rotated_right() *
                            self.lane_distance_from_center(lane, endpoint))
                    return point1 + vector * counter + side
        raise ValueError('Point outside of Way.')

    def lane_refs(self, direction: Endpoint, l_to_r: bool = True,
                  include_opposite: bool = False,
                  opposite_only: bool = False) -> Iterator[LaneRef]:
        """Get lanes in the given direction.

        Get lanes in the given direction. Check LaneRef documentation for more
        information. If l_to_r is True (default), return lanes starting from
        the left, otherwise start from the right.
        """
        opposite = include_opposite or opposite_only
        lanes = range(
            -self.lane_count[direction.other.value] if opposite else 0,
            0 if opposite_only else self.lane_count[direction.value])

        for i in lanes if l_to_r else reversed(lanes):
            yield LaneRef.build(self, direction, i)

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

    def on_delete(self) -> Iterable[Entity]:
        """Disconnect this way from the network.

        Removes the connections from this way to nodes and also the references
        from the nodes to this way.
        """
        nodes = {self.start, self.end} - {None}
        for node in nodes:
            self.disconnect(node)
            Index.INSTANCE.updated(node)
            for way in node.ways:
                Index.INSTANCE.updated(way)

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

    def __repr__(self):
        return f'{Way.__name__}(id={self.id}, xid={self.xid})'


class WaySegment(NamedTuple):
    """A segment in the way geometry.

    The segment starts at point start and ends at point end. The vector is
    normalized, pointing from start to end and the normal points 90 degrees
    clockwise from vector. The offsets determine the positions of the points
    that make the trapezoid of the way segment. For example:

    start_left = (start - normal * way_geometry.half_width
                  + vector * start_left_offset)
    """

    start: Point
    end: Point
    vector: Vector
    normal: Vector
    start_left_offset: float
    start_right_offset: float
    end_left_offset: float
    end_right_offset: float

    def start_left(self, half_width: float) -> Point:
        """Get the left start point of the segment trapezoid."""
        return (self.start
                - self.normal * half_width
                + self.vector * self.start_left_offset)

    def start_right(self, half_width: float) -> Point:
        """Get the right start point of the segment trapezoid."""
        return (self.start
                + self.normal * half_width
                + self.vector * self.start_right_offset)

    def end_left(self, half_width: float) -> Point:
        """Get the left end point of the segment trapezoid."""
        return (self.end
                - self.normal * half_width
                + self.vector * self.end_left_offset)

    def end_right(self, half_width: float) -> Point:
        """Get the right end point of the segment trapezoid."""
        return (self.end
                + self.normal * half_width
                + self.vector * self.end_right_offset)

    def length(self) -> float:
        """Get the length of this segment.

        The length is calculated by translating start and end points by the
        avarage of their left and right points and measuring the distance
        between them. The sum of all segment lenths will not be equal to the
        way length.
        """
        mean_start = (self.start_left_offset + self.start_right_offset) / 2
        mean_end = (self.end_left_offset + self.end_right_offset) / 2
        return abs(self.end + self.vector * mean_end
                   - (self.start + self.vector * mean_start))


@with_slots
@dataclass(eq=False)
class WayGeometry:
    """Information on the geometric shape of a way."""

    way: Way
    half_width: float = field(init=False)
    segments: List[WaySegment] = field(init=False, default_factory=list)

    def __post_init__(self):
        self.half_width = self.way.total_lane_count * HALF_LANE_WIDTH
        self._build_segments()

    @property
    def start_offset(self):
        """Distance from the start node to the start of the way geometry."""
        return self.way.start.geometry.distance(self.way.oriented())

    @property
    def end_offset(self):
        """Distance from the end node to the end of the way geometry."""
        return self.way.end.geometry.distance(
            self.way.oriented(endpoint=Endpoint.END))

    def _build_segments(self):
        """Build way segments."""
        for i, points in enumerate(window_iter(self.way.points(), size=3)):
            vectors = ((points[1] - points[0]), (points[2] - points[1]))
            norm_vectors = tuple(v.normalized() for v in vectors)
            normals = tuple(v.rotated_right() for v in norm_vectors)

            vector = (norm_vectors[0] + norm_vectors[1]).normalized()
            width_vector = (sec(vector, vectors[1]) * self.half_width
                            * vector.rotated_right())

            width_projection = width_vector.scalar_projection_on(vectors[0])
            if width_projection > 0:
                end_point = points[1] - width_vector.projection_on(vectors[0])
            else:
                end_point = points[1] + width_vector.projection_on(vectors[0])

            start_offset = 0.0 if i > 0 else self.start_offset
            self.segments.append(
                WaySegment(points[0], end_point, norm_vectors[0], normals[0],
                           start_offset, start_offset, 0.0, 0.0))

            self.segments.append(
                WaySegment(end_point, points[1], norm_vectors[0], normals[0],
                           0.0, 0.0, -width_projection, width_projection))

            end_point = points[1] + abs(width_projection) * norm_vectors[1]
            self.segments.append(
                WaySegment(points[1], end_point, norm_vectors[1], normals[1],
                           width_projection, -width_projection, 0.0, 0.0))

        points = list(islice(self.way.points(reverse=True), 2))
        start_offset = (self.start_offset if self.way.segment_count == 1
                        else 0.0)
        end_offset = -self.end_offset
        vector = (points[0] - points[1]).normalized()
        self.segments.append(
            WaySegment(points[1], points[0], vector, vector.rotated_right(),
                       start_offset, start_offset, end_offset, end_offset))


class OrientedWay(NamedTuple):
    """A tuple containing a Way and an Endpoint.

    Used to represent a specific connection of a Way and a Node. Useful when
    referencing an incident Way from a Node, to avoid ambiguity, since both
    ends of a Way can be connected to the same Node. Can be interpreted as the
    given Way in the direction starting from the given Endpoint.
    """

    way_ref: EntityRef[Way]
    endpoint: Endpoint

    @staticmethod
    def build(way: Way, endpoint: Endpoint):
        """Create OrientedWay from a Way instead of a weak reference."""
        return OrientedWay(EntityRef(way), endpoint)

    @property
    def way(self) -> Optional[Way]:
        """Get the referenced way."""
        return self.way_ref()

    @property
    def start(self) -> Node:
        """Get the source node of the way in this direction."""
        return (self.way.start if self.endpoint is Endpoint.START
                else self.way.end)

    @property
    def end(self) -> Node:
        """Get the target node of the way in this direction."""
        return (self.way.end if self.endpoint is Endpoint.START
                else self.way.start)

    @property
    def lane_count(self) -> int:
        """Get the number of lanes of the way in this direction."""
        return self.way.lane_count[self.endpoint.value]

    @property
    def length(self) -> float:
        """Get the length of the way."""
        return self.way.length

    @property
    def weight(self) -> float:
        """Get the weight of the way in this direction."""
        return self.way.weight[self.endpoint.value]

    def flipped(self) -> OrientedWay:
        """Get OrientedWay with same way in the other direction."""
        return OrientedWay(self.way_ref, self.endpoint.other)

    def lane_refs(self, l_to_r: bool = True, include_opposite: bool = False,
                  opposite_only: bool = False) -> Iterator[LaneRef]:
        """Get lanes in the direction of the oriented way.

        A shortcut for Way.lane_refs.
        """
        return self.way.lane_refs(self.endpoint, l_to_r, include_opposite,
                                  opposite_only)

    def points(self, skip=0) -> Iterator[Point]:
        """Get generator for points in order, including nodes and waypoints."""
        return self.way.points(skip, self.endpoint is Endpoint.END)

    def __repr__(self):
        return (f'{OrientedWay.__name__}(way_id={self.way.id}, '
                f'endpoint={self.endpoint.name[0]})')


class LaneRef(NamedTuple):
    """A tuple containing the same as OrientedWay, with a lane index.

    The two first elements are equivalent to the values of OrientedWay and the
    third value is the number of the lane. Lanes start from zero, meaning the
    leftmost lane if looking at the way in the direction of the two first
    values, interpreted as an OrientedWay, to n - 1, with n being the number of
    lanes in that direction.
    """

    way_ref: EntityRef[Way]
    endpoint: Endpoint
    index: int

    @staticmethod
    def build(way: Way, endpoint: Endpoint, index: int):
        """Create LaneRef from a Way instead of a weak reference."""
        return LaneRef(EntityRef(way), endpoint, index)

    @property
    def way(self) -> Optional[Way]:
        """Get the referenced way."""
        return self.way_ref()

    @property
    def oriented_way(self) -> OrientedWay:
        """Get oriented way from this lane."""
        return OrientedWay(self.way_ref, self.endpoint)

    @property
    def start(self) -> Node:
        """Get the start node of the lane."""
        return (self.way.start if self.endpoint is Endpoint.START
                else self.way.end)

    @property
    def end(self) -> Node:
        """Get the end node of the lane."""
        return (self.way.end if self.endpoint is Endpoint.START
                else self.way.start)

    def positive(self) -> LaneRef:
        """Get equivalent lane with positive index."""
        if self.index >= 0:
            return self
        return LaneRef(self.way_ref, self.endpoint.other, -self.index - 1)

    def distance_from_center(self) -> float:
        """Get distance to the right from way center to the lane."""
        return self.way.lane_distance_from_center(self.index, self.endpoint)

    def __repr__(self):
        return (f'{LaneRef.__name__}(way_id={self.way.id}, '
                f'endpoint={self.endpoint.name[0]}, index={self.index})')


class LaneSegment(NamedTuple):

    distance: float
    way_distance: float
    start_point: Point
    vector: Vector


class Lane:

    __slots__ = 'segments', 'lane_ref'

    lane_ref: LaneRef
    distance_from_center: float
    length: float
    segments: Tuple[LaneSegment]

    def __init__(self, lane_ref: LaneRef,
                 vectors: Iterator[Vector] = None,
                 edge_normals: Iterator[Vector] = None,
                 waypoint_normals: Iterator[Vector] = None):
        self.lane_ref = lane_ref.positive()
        self.distance_from_center = lane_ref.distance_from_center()

        reverse = self.endpoint is Endpoint.END
        if vectors is None:
            vectors = self.way.vectors(reverse=reverse)
        if edge_normals is None:
            edge_normals = self.way.edge_normals(reverse=reverse)
        if waypoint_normals is None:
            waypoint_normals = self.way.waypoint_normals(reverse=reverse)

        self._build_segments(vectors, edge_normals, waypoint_normals)

    @property
    def way(self) -> Optional[Way]:
        """Get the referenced way."""
        return self.lane_ref.way_ref()

    @property
    def endpoint(self) -> Endpoint:
        """Get the lane orientation."""
        return self.lane_ref.endpoint

    @property
    def index(self) -> int:
        """Get lane index."""
        return self.lane_ref.index

    @property
    def oriented_way(self) -> OrientedWay:
        """Get oriented way from this lane."""
        return self.lane_ref.oriented_way

    @property
    def start(self) -> Node:
        """Get the start node of the lane."""
        return self.lane_ref.start

    @property
    def end(self) -> Node:
        """Get the end node of the lane."""
        return self.lane_ref.end

    def _build_segments(self, vectors: Iterator[Vector] = None,
                        edge_normals: Iterator[Vector] = None,
                        waypoint_normals: Iterator[Vector] = None):
        length = 0.0
        distance = 0.0
        way_distance = self.start.geometry.distance(self.oriented_way)

        for i, (vector, edge_normal, waypoint_normal) in enumerate(
                zip(vectors, edge_normals, waypoint_normals)):
            vector = vector.normalized()

        self.length = length
