"""Different representations of positions on the network."""

from __future__ import annotations

from dataclasses import dataclass
from typing import TYPE_CHECKING, Tuple, Union

from dataslots import with_slots

if TYPE_CHECKING:
    from tsim.model.entity import EntityRef
    from tsim.model.geometry import Point, Vector
    from tsim.model.network.intersection import Curve
    from tsim.model.network.way import Lane, OrientedWay, Way


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
@dataclass(frozen=True)
class OrientedWayPosition:
    """A position in an `OrientedWay`.

    The position is in meters from the endpoint of the oriented way.
    """

    oriented_way: OrientedWay
    position: float

    def world_position(self) -> Tuple[Point, Vector]:
        """Get world position and direction at this oriented way position."""
        raise NotImplementedError()


@with_slots
@dataclass(frozen=True)
class LanePosition:
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
    def oriented_way(self) -> OrientedWay:
        """Get the oriented way of this lane."""
        return self.lane.oriented_way

    @property
    def oriented_way_position(self) -> OrientedWayPosition:
        """Get the oriented way position at this lane position."""
        return OrientedWayPosition(
            self.lane.oriented_way,
            self.lane.lane_to_oriented_position(self.position))

    def world_and_segment_position(self) -> WorldAndSegmentPosition:
        """Get world and segment position at this lane position."""
        return self.lane.world_and_segment_position(self.position)


@with_slots
@dataclass(frozen=True)
class CurvePosition:
    """A position in a `Curve`.

    The position is in meters from the start of the curve.
    """

    curve: Curve
    position: float

    @property
    def location(self) -> NetworkLocation:
        """Get the `NetworkLocation` of this curve position."""
        return self.curve

    def world_and_segment_position(self) -> WorldAndSegmentPosition:
        """Get world and segment position at this curve position.

        Curves don't have segments, so the segment is always `0` and the
        segment's end is the length of the curve.
        """
        world = self.curve.world_position(self.position)
        return WorldAndSegmentPosition(*world, 0, self.curve.length)


@with_slots
@dataclass(frozen=True)
class WorldAndSegmentPosition:
    """World position and segment information bundled together."""

    position: Point
    direction: Vector
    segment: int
    segment_end: float


if TYPE_CHECKING:
    NetworkLocation = Union[Lane, Curve]
    NetworkPosition = Union[LanePosition, CurvePosition]
