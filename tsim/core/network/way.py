"""Way and related classes."""

from __future__ import annotations

from dataclasses import dataclass, field
from itertools import accumulate, chain, islice
from math import floor, sqrt
from operator import itemgetter
from typing import (TYPE_CHECKING, Iterable, Iterator, List, Optional, Tuple,
                    Union)

from dataslots import with_slots

import tsim.core.index as Index
from tsim.core.entity import DeleteResult, EntityRef
from tsim.core.geometry import (BoundingRect, Point, Polygon, Vector,
                                 calc_bounding_rect, line_intersection,
                                 point_in_polygon, sec)
from tsim.core.network.endpoint import Endpoint
from tsim.core.network.entity import NetworkEntity
from tsim.core.network.lane import (HALF_LANE_WIDTH, LANE_WIDTH, Lane,
                                     LanePosition, LaneRef)
from tsim.core.network.orientedway import OrientedWay, OrientedWayPosition
from tsim.core.units import kph_to_mps
from tsim.utils.cachedproperty import add_cached, cached_property
from tsim.utils.iterators import drop_duplicates, window_iter

if TYPE_CHECKING:
    from tsim.core.network.node import Node

DEFAULT_MAX_SPEED_KPH = 60.0
DEFAULT_MAX_SPEED_MPS = kph_to_mps(DEFAULT_MAX_SPEED_KPH)


@add_cached
class Way(NetworkEntity):
    """A connection between two Nodes.

    A Way connects two Nodes and can have a list of intermediary points, called
    waypoints.
    """

    __slots__ = 'start_ref', 'end_ref', 'lane_count', 'waypoints', 'max_speed'

    start_ref: EntityRef[Node]
    end_ref: EntityRef[Node]
    lane_count: Tuple[int, int]
    waypoints: Tuple[Point]
    max_speed: float

    def __init__(self, start: Union[Node, EntityRef[Node]],
                 end: Union[Node, EntityRef[Node]],
                 lane_count: Tuple[int, int], waypoints: Tuple[Point] = (),
                 max_speed: float = DEFAULT_MAX_SPEED_MPS):
        self.id = None
        self.start_ref = EntityRef(start)
        self.end_ref = EntityRef(end)
        self.start.starts.append(EntityRef(self))
        self.start.clear_cache(True)
        self.end.ends.append(EntityRef(self))
        self.end.clear_cache(True)
        self.lane_count = lane_count
        self.waypoints = waypoints
        self.max_speed = max_speed
        super().__init__()

    @cached_property
    def geometry(self) -> WayGeometry:
        """Get the geometry info for the way."""
        return WayGeometry(self)

    geometry.on_update(NetworkEntity.update_index_bounding_rect)

    @cached_property
    def lanes(self) -> Tuple[Lane]:
        """Get the lanes of this way in a tuple.

        The indexes of the lanes are as in a LaneRef oriented from START. Lane
        -1 is the first lane going to the opposite direction and so on. This
        kind of indexing will work because Python.
        """
        return tuple(chain(
            (Lane(l) for l in self.lane_refs(Endpoint.START)),
            (Lane(l) for l in self.lane_refs(Endpoint.START,
                                             opposite_only=True))))

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
    def start(self) -> Node:
        """Get or set the referenced start node."""
        if self.start_ref is not None:
            return self.start_ref()
        return None

    @start.setter
    def start(self, value):
        if value is None:
            self.start_ref = None
        else:
            self.start_ref = EntityRef(value)

    @property
    def end(self) -> Node:
        """Get or set the referenced end node."""
        if self.end_ref is not None:
            return self.end_ref()
        return None

    @end.setter
    def end(self, value):
        if value is None:
            self.end_ref = None
        else:
            self.end_ref = EntityRef(value)

    @property
    def neighbors(self) -> Iterable[NetworkEntity]:
        """Get iterable with entities directly connected to this way."""
        return (self.start, self.end)

    @property
    def polygon(self) -> Polygon:
        """Get polygon from the way geometry."""
        return self.geometry.polygon

    @property
    def start_offset(self):
        """Distance from the start node to the start of the way geometry."""
        return self.start.geometry.distance(self.oriented())

    @property
    def end_offset(self):
        """Distance from the end node to the end of the way geometry."""
        return self.end.geometry.distance(self.oriented(endpoint=Endpoint.END))

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
        return self.geometry.calc_bounding_rect(accumulated)

    def distance(self, point: Point, squared: bool = False) -> float:
        """Calculate smallest distance from the way to a point."""
        result, _ = min((point.distance_to_segment(p, v, squared=True)
                         for p, v in zip(self.points(), self.vectors())),
                        key=itemgetter(0))
        return result if squared else sqrt(result)

    def distance_and_point(self, point: Point, squared: bool = False) \
            -> Tuple[float, Point]:
        """Calculate smallest distance from the way to a point.

        Returns the smallest distance and the closest point in a tuple.
        """
        result, closest = min((point.distance_to_segment(p, v, squared=True)
                               for p, v in zip(self.points(), self.vectors())),
                              key=itemgetter(0))
        return (result if squared else sqrt(result), closest)

    def way_position_raw(self, point: Point, include_lane: bool = True) \
            -> Tuple[float, Optional[int]]:
        """Get the way position at the given point.

        Returns a tuple with the distance in meters from the start of the way
        and the lane index at the given point. If `include_lane` is False, the
        lane index is `None`.
        """
        way_position, distance = self.geometry.way_position_raw(point)
        if include_lane:
            index = self.lane_index_from_distance(distance)
        else:
            index = None

        return way_position, index

    def way_position_at(self, point: Point) -> WayPosition:
        """Get the way position at the given point.

        The `position` value is in meters from the start of the way.
        """
        way_position, _ = self.way_position_raw(point, False)
        return WayPosition(EntityRef(self), way_position)

    def oriented_way_position_at(self, point: Point) -> OrientedWayPosition:
        """Get the oriented way position at the given point.

        The `position` value is in meters from the endpoint of the oriented
        way.
        """
        way_position, index = self.way_position_raw(point)
        if index >= 0:
            endpoint = Endpoint.START
        else:
            endpoint = Endpoint.END
            way_position = self.length - way_position
        return OrientedWayPosition(self.oriented(endpoint), way_position)

    def lane_position_at(self, point: Point) -> LanePosition:
        """Get the way position at the given point.

        The `position` value is in meters from the start of the lane.
        """
        way_position, index = self.way_position_raw(point)
        lane = self.lanes[index]
        return LanePosition(lane, lane.way_to_lane_position(way_position))

    def other(self, node: Node) -> Node:
        """Get the other endpoint of the way, opposite to node."""
        return self.start if self.end is node else self.end

    def oriented(self, endpoint: Endpoint = Endpoint.START) -> OrientedWay:
        """Get OrientedWay from this way."""
        return OrientedWay.build(self, endpoint)

    def points(self, offsets: Optional[Tuple[float, Optional[float]]] = None,
               skip: int = 0, reverse: bool = False) -> Iterator[Point]:
        """Get generator for points in order, including nodes and waypoints."""
        first, last = self.start.position, self.end.position
        waypoints = self.waypoints
        if reverse:
            first, last = last, first
            waypoints = reversed(waypoints)

        if offsets is not None:
            start, end = offsets
            if end is not None and start > end:
                raise ValueError('Start offset cannot be after end offset.')

            covered = 0.0
            points = enumerate(self.point_pairs(reverse=reverse))

            drop, take = 0, None
            i, point1, point2 = None, None, None
            if start > 0.0:
                for i, (point1, point2) in points:
                    vector = point2 - point1
                    distance = abs(vector)
                    if covered + distance >= start:
                        vector = vector.normalized()
                        first = point1 + vector * (start - covered)
                        drop = i
                        break
                    covered += distance
                if end is not None and covered + distance >= end:
                    last = point1 + vector * (end - covered)
                    take = i
                    end = None
                else:
                    covered += distance

            if end is not None:
                for i, (point1, point2) in points:
                    vector = point2 - point1
                    distance = abs(vector)
                    if covered + distance >= end:
                        vector = vector.normalized()
                        last = point1 + vector * (end - covered)
                        take = i
                        break
                    covered += distance

            waypoints = islice(waypoints, drop, take)

        iterator = chain((first,), waypoints, (last,))
        yield from drop_duplicates(islice(iterator, skip, None))

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

    def lane_divider_distance(self, endpoint: Endpoint = Endpoint.START) \
            -> float:
        """Get distance to the right of the central lane divider."""
        return (self.lane_count[endpoint.other.value]
                - self.total_lane_count / 2) * LANE_WIDTH

    def lane_distance_from_center(self, lane: int,
                                  endpoint: Endpoint = Endpoint.START) \
            -> float:
        """Get distance to the right from way center to the lane."""
        return (1 + 2 * lane
                - self.lane_count[endpoint.value]
                + self.lane_count[endpoint.other.value]) * HALF_LANE_WIDTH

    def lane_index_valid(self, index: int,
                         endpoint: Endpoint = Endpoint.START) -> bool:
        """Get whether the given lane index is valid in given orientation."""
        return (-self.lane_count[endpoint.other.value] <= index
                < self.lane_count[endpoint.value])

    def lane_index_from_distance(self, distance: float,
                                 endpoint: Endpoint = Endpoint.START) \
            -> Optional[int]:
        """Get lane index from distance to the right from way center."""
        index = floor((distance - self.lane_divider_distance(endpoint))
                      / LANE_WIDTH)
        if self.lane_index_valid(index, endpoint):
            return index
        return None

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
                  include_opposite: bool = False, opposite_only: bool = False,
                  positive: bool = False) -> Iterator[LaneRef]:
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
            lane_ref = LaneRef.build(self, direction, i)
            yield lane_ref.positive() if positive else lane_ref

    def disconnect(self, node: Node):
        """Disconnect this way from a node.

        Removes the connections from this way to  the given node and also the
        reference from the node to this way. Will make the way invalid.
        """
        if node is self.start:
            index = next(i for i, v in enumerate(self.start.starts)
                         if v.id == self.id)
            del self.start.starts[index]
            node.clear_cache(True)
            self.start = None

        if node is self.end:
            index = next(i for i, v in enumerate(self.end.ends)
                         if v.id == self.id)
            del self.end.ends[index]
            node.clear_cache(True)
            self.end = None

    def on_delete(self) -> DeleteResult:
        """Disconnect this way from the network.

        Removes the connections from this way to nodes and also the references
        from the nodes to this way.
        """
        nodes = {self.start, self.end} - {None}
        updated = set()
        for node in nodes:
            self.disconnect(node)

        for node in nodes:
            updated.add(node)
            for way in node.ways:
                updated.add(way)

        to_delete = set()
        for node in nodes:
            if not node.starts and not node.ends:
                to_delete.add(node)
            else:
                try:
                    node.dissolve()
                    to_delete.add(node)
                except ValueError:
                    pass
        Index.INSTANCE.rebuild_path_map()
        return DeleteResult(to_delete, updated)


@with_slots
@dataclass(frozen=True)
class WayPosition:
    """A position in a `Way`.

    The position is in meters from the start of the way.
    """

    way: EntityRef[Way]
    position: float

    def world_position(self) -> Tuple[Point, Vector]:
        """Get world position and direction at this way position."""
        raise NotImplementedError()


@with_slots
@dataclass
class WaySegment:
    """A segment in the way geometry.

    The segment starts at point `start` and ends at point `end`. The `vector`
    is normalized, pointing from `start` to `end` and the `width_vector` points
    90 degrees clockwise from `vector`, with norm equal to half the way width.
    The offsets determine the positions of the points that make the trapezoid
    of the way segment. For example::

        start_left = start - width_vector + vector * start_left_offset

    """

    start: Point
    end: Point
    vector: Vector
    width_vector: Vector
    start_left_offset: float
    start_right_offset: float
    end_left_offset: float
    end_right_offset: float
    start_left: Point = field(init=False)
    start_right: Point = field(init=False)
    end_left: Point = field(init=False)
    end_right: Point = field(init=False)

    def __post_init__(self):
        self.start_left = (self.start - self.width_vector
                           + self.vector * self.start_left_offset)
        self.start_right = (self.start + self.width_vector
                            + self.vector * self.start_right_offset)
        self.end_left = (self.end - self.width_vector
                         + self.vector * self.end_left_offset)
        self.end_right = (self.end + self.width_vector
                          + self.vector * self.end_right_offset)

    @property
    def is_rectangular(self):
        """Get whether the segment is rectangular.

        A segment that is a rectangle has both start offsets equal and both end
        offsets equal. Otherwise the segment is just a trapezoid.
        """
        return (self.start_left_offset == self.start_right_offset and
                self.end_left_offset == self.end_right_offset)

    @property
    def start_vector(self):
        """Vector from `start_left` to `start_right`."""
        return self.start_right - self.start_left

    @property
    def end_vector(self):
        """Vector from `end_left` to `end_right`."""
        return self.end_right - self.end_left

    def length(self) -> float:
        """Get the length of this segment.

        The length is calculated by translating start and end points by the
        avarage of their left and right points and measuring the distance
        between them. The sum of all segment lenths will not be equal to the
        way length because of the offsets from the endpoints.
        """
        mean_start = (self.start_left_offset + self.start_right_offset) / 2
        mean_end = (self.end_left_offset + self.end_right_offset) / 2
        return abs(self.end + self.vector * mean_end
                   - (self.start + self.vector * mean_start))

    def inverted(self) -> WaySegment:
        """Get the same segment, in the opposite direction."""
        return WaySegment(self.end, self.start,
                          -self.vector, -self.width_vector,
                          -self.end_right_offset, -self.end_left_offset,
                          -self.start_right_offset, -self.start_left_offset)

    def contains_point(self, point: Point) -> bool:
        """Get whether the segment contains the given point."""
        return point_in_polygon(point, [self.start_left, self.end_left,
                                        self.end_right, self.start_right])

    def way_distance_at(self, point: Point) -> float:
        """Way distance from the start of this segment, with offsets."""
        if self.is_rectangular:
            offset = (self.start_left_offset + self.start_right_offset) / 2
            return (point - (self.start + self.vector * offset)) \
                .scalar_projection_on(self.vector)

        center = line_intersection(self.end_left, self.end_vector,
                                   self.start_left, self.start_vector)
        way_point = line_intersection(center, point - center,
                                      self.start, self.vector)
        return abs(way_point - self.start)


@with_slots
@dataclass(eq=False)
class WayGeometry:
    """Information on the geometric shape of a way."""

    way_ref: EntityRef[Way]
    half_width: float = field(init=False)
    segments: List[WaySegment] = field(init=False, default_factory=list)
    polygon: Polygon = field(init=False, default_factory=list)

    def __post_init__(self):
        if not isinstance(self.way_ref, EntityRef):
            self.way_ref = EntityRef(self.way_ref)

        self.half_width = self.way_ref().total_lane_count * HALF_LANE_WIDTH
        self._build_segments()
        self._build_polygon()

    @property
    def way(self):
        """Get the referenced way."""
        return self.way_ref()

    @property
    def start_offset(self):
        """Distance from the start node to the start of the way geometry."""
        return self.way.start_offset

    @property
    def end_offset(self):
        """Distance from the end node to the end of the way geometry."""
        return self.way.end_offset

    def calc_bounding_rect(self,
                           accumulated: BoundingRect = None) -> BoundingRect:
        """Calculate the bounding rect of the way."""
        return calc_bounding_rect(self.polygon, accumulated)

    def way_position_raw(self, point: Point) -> Optional[Tuple[float, float]]:
        """Get way_position and distance to the right at given point."""
        segment = None
        acc_length = self.start_offset
        for segment_ in self.segments:
            if segment_.contains_point(point):
                segment = segment_
                break
            acc_length += segment_.length()
        if segment is None:
            return None

        way_position = acc_length + segment.way_distance_at(point)
        right_distance = \
            (point - segment.start).scalar_projection_on(segment.width_vector)
        return way_position, right_distance

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

            if i == 0:
                start_point = points[0]
                start_offset = self.start_offset
            else:
                start_offset = 0.0

            self.segments.append(
                WaySegment(start_point, end_point, norm_vectors[0],
                           normals[0] * self.half_width,
                           start_offset, start_offset, 0.0, 0.0))

            self.segments.append(
                WaySegment(end_point, points[1], norm_vectors[0],
                           normals[0] * self.half_width,
                           0.0, 0.0, -width_projection, width_projection))

            end_point = points[1] + abs(width_projection) * norm_vectors[1]
            self.segments.append(
                WaySegment(points[1], end_point, norm_vectors[1],
                           normals[1] * self.half_width,
                           width_projection, -width_projection, 0.0, 0.0))
            start_point = end_point

        points = list(islice(self.way.points(reverse=True), 2))

        if self.way.segment_count == 1:
            start_offset = self.start_offset
            start_point = points[1]
        else:
            start_offset = 0.0

        end_offset = -self.end_offset
        vector = (points[0] - start_point).normalized()
        self.segments.append(
            WaySegment(start_point, points[0], vector,
                       vector.rotated_right() * self.half_width,
                       start_offset, start_offset, end_offset, end_offset))

    def _build_polygon(self):
        self.polygon.append(self.segments[0].start_left)
        for segment in self.segments:
            self.polygon.append(segment.end_left)
        self.polygon.append(self.segments[-1].end_right)
        for segment in reversed(self.segments):
            self.polygon.append(segment.start_right)
