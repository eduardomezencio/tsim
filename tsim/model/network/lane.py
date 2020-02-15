"""Lane and related classes."""

from __future__ import annotations

from dataclasses import dataclass
from typing import TYPE_CHECKING, NamedTuple, Optional, Tuple

from dataslots import with_slots

from tsim.model.entity import EntityRef
from tsim.model.geometry import Point, Vector, line_intersection
from tsim.model.network.endpoint import Endpoint
from tsim.model.network.orientedway import OrientedWay, OrientedWayPosition
from tsim.model.network.location import (NetworkLocation, NetworkPosition,
                                         WorldAndSegmentPosition)
from tsim.model.network.traffic import Traffic
from tsim.utils.linkedlist import LinkedList

if TYPE_CHECKING:
    from tsim.model.network.intersection import Curve
    from tsim.model.network.node import Node
    from tsim.model.network.way import Way

LANE_WIDTH = 3.0
HALF_LANE_WIDTH = 0.5 * LANE_WIDTH


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
    def lane(self) -> Lane:
        """Get the referenced lane."""
        return self.oriented_way.lane(self.index)

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

    @property
    def left_neighbor(self) -> LaneRef:
        """Get lane reference one lane to the left."""
        return LaneRef(self.way_ref, self.endpoint, self.index - 1)

    @property
    def right_neighbor(self) -> LaneRef:
        """Get lane reference one lane to the right."""
        return LaneRef(self.way_ref, self.endpoint, self.index + 1)

    def positive(self) -> LaneRef:
        """Get equivalent lane with positive index."""
        if self.index >= 0:
            return self
        return LaneRef(self.way_ref, self.endpoint.other, -self.index - 1)

    def distance_from_center(self) -> float:
        """Get distance to the right from way center to the lane."""
        return self.way.lane_distance_from_center(self.index, self.endpoint)

    def __call__(self) -> Lane:
        """Get the referenced lane."""
        return self.lane

    def __repr__(self):
        return (f'{LaneRef.__name__}(way_id={self.way.id}, '
                f'endpoint={self.endpoint.name[0]}, index={self.index})')


@with_slots
@dataclass(frozen=True)
class LaneSegment:
    """Segment of a `Lane`."""

    start_distance: float
    end_distance: float
    start_way_distance: float
    factor: float
    start: Point
    vector: Vector
    is_turning: bool


class Lane(NetworkLocation):
    """A longitudinal section of a `Way` for one-way flow."""

    __slots__ = ('lane_ref', 'distance_from_center', 'length', 'segments',
                 'traffic')

    lane_ref: LaneRef
    distance_from_center: float
    length: float
    segments: Tuple[LaneSegment]
    traffic: Traffic

    def __init__(self, lane_ref: LaneRef):
        self.lane_ref = lane_ref.positive()
        self.distance_from_center = self.lane_ref.distance_from_center()
        self._build_segments()
        self.traffic = LinkedList()

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

    lane_index = index

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

    @property
    def segment_count(self) -> int:
        """Get the number of lane segments."""
        return len(self.segments)

    @property
    def left_neighbor(self) -> Lane:
        """Get the lane to the left of this one."""
        return self.lane_ref.left_neighbor()

    @property
    def right_neighbor(self) -> Lane:
        """Get the lane to the right of this one."""
        return self.lane_ref.right_neighbor()

    def world_and_segment_position(self, position: float) \
            -> WorldAndSegmentPosition:
        """Get world and segment position on given lane position."""
        segment: LaneSegment = None
        if position >= 0.0:
            i, segment = next(((i, s) for i, s in enumerate(self.segments)
                               if s.end_distance >= position), None)

        if segment is not None:
            segment_position = position - segment.start_distance
            point = segment.start + segment.vector * segment_position
            return WorldAndSegmentPosition(point, segment.vector,
                                           i, segment.end_distance)

        raise ValueError('Position outside of lane.')

    def lane_to_way_position(self, position: float) -> float:
        """Get way position from lane position."""
        way_position = None
        if position >= 0.0:
            segment = next((s for s in self.segments
                            if s.end_distance >= position), None)
            if segment is not None:
                way_position = (segment.start_way_distance +
                                (position - segment.start_distance)
                                / segment.factor)

        if way_position is not None:
            if self.endpoint is Endpoint.END:
                way_position = self.way.length - way_position
            return way_position

        raise ValueError('Position outside of lane.')

    def lane_to_oriented_position(self, position: float) -> float:
        """Get oriented way position from lane position."""
        way_position = self.lane_to_way_position(position)
        if self.endpoint is Endpoint.START:
            return way_position
        return self.way.length - way_position

    def oriented_way_position(self, position: float) -> OrientedWayPosition:
        """Get `OrientedWayPosition` from lane position.

        Almost the same as `lane_to_oriented_position`, but returns position as
        a `OrientedWayPosition` instead of the position value only.
        """
        return OrientedWayPosition(self.oriented_way,
                                   self.lane_to_oriented_position(position))

    def way_to_lane_position(self, position: float,
                             endpoint: Endpoint = Endpoint.START,
                             default: float = None) -> float:
        """Get lane position from way position.

        The `position` argument is a distance in meters from the given
        `endpoint`. If position is taken from a way position without
        orientation there's no need to set the `endpoint`, since an oriented
        way position from the `START` endpoint is equivalent to a way position.
        """
        offsets = (self.oriented_way.start_offset,
                   self.oriented_way.end_offset)
        if self.endpoint is not endpoint:
            position = self.way.length - position

        last_segment = None
        if offsets[0] < position <= self.way.length - offsets[1]:
            for segment in self.segments:
                if segment.start_way_distance > position:
                    break
                last_segment = segment

        segment = last_segment
        if segment is not None:
            return (segment.start_distance +
                    segment.factor * (position - segment.start_way_distance))

        if default is not None:
            return default
        raise ValueError('Position outside of lane.')

    def get_curve(self, dest: OrientedWay,
                  accept_lane_change: bool = True) -> Optional[Curve]:
        """Get the curve connecting this lane to `dest` oriented way."""
        connection = self.end.get_lane_connection(self.lane_ref, dest)
        if accept_lane_change or connection[0].index == self.index:
            return self.end.intersection.curves[connection]
        return None

    def _build_segments(self):
        distance = 0.0
        segments = []

        way_segments = self.lane_ref.way.geometry.segments
        if self.endpoint is Endpoint.END:
            way_distance = self.lane_ref.way.geometry.end_offset
            way_segments = (s.inverted() for s in reversed(way_segments))
        else:
            way_distance = self.lane_ref.way.geometry.start_offset

        for way_segment in way_segments:
            point = (way_segment.start
                     + (self.distance_from_center
                        * way_segment.width_vector.normalized()))
            left = way_segment.start_left
            vector = (way_segment.start_right - left).normalized()
            start = line_intersection(point, way_segment.vector,
                                      left, vector)
            left = way_segment.end_left
            vector = (way_segment.end_right - left).normalized()
            end = line_intersection(point, way_segment.vector,
                                    left, vector)
            length = abs(end - start)
            way_length = way_segment.length()
            end_distance = distance + length
            segments.append(LaneSegment(distance, end_distance, way_distance,
                                        length / way_length, start,
                                        way_segment.vector,
                                        not way_segment.is_rectangular))
            distance = end_distance
            way_distance += way_length

        self.length = distance
        self.segments = tuple(segments)

    def __repr__(self):
        return (f'{Lane.__name__}(way_id={self.way.id}, '
                f'endpoint={self.endpoint.name[0]}, index={self.index})')


@with_slots
@dataclass(frozen=True)
class LanePosition(NetworkPosition):
    """A position in a `Lane`.

    The position is in meters from the start of the lane.
    """

    lane: Lane
    position: float

    @property
    def location(self) -> NetworkLocation:
        """Get the `NetworkLocation` of this lane position."""
        return self.lane

    @property
    def remaining(self) -> float:
        """Get distance in meters to the end of the lane."""
        return self.lane.length - self.position

    @property
    def oriented_way(self) -> OrientedWay:
        """Get the oriented way of this lane."""
        return self.lane.oriented_way

    @property
    def oriented_way_position(self) -> OrientedWayPosition:
        """Get the oriented way position at this lane position."""
        return OrientedWayPosition(
            self.lane.oriented_way,
            self.lane.lane_to_oriented_position(self.position))

    @property
    def left_neighbor(self) -> LanePosition:
        """Get same position on the lane to the left."""
        way_position = self.lane.lane_to_way_position(self.position)
        lane = self.lane.left_neighbor
        try:
            return LanePosition(lane, lane.way_to_lane_position(way_position))
        except ValueError:
            return LanePosition(lane, self.position)

    @property
    def right_neighbor(self) -> LanePosition:
        """Get same position on the lane to the right."""
        way_position = self.lane.lane_to_way_position(self.position)
        lane = self.lane.right_neighbor
        try:
            return LanePosition(lane, lane.way_to_lane_position(way_position))
        except ValueError:
            return LanePosition(lane, self.position)

    def world_and_segment_position(self) -> WorldAndSegmentPosition:
        """Get world and segment position at this lane position."""
        return self.lane.world_and_segment_position(self.position)
