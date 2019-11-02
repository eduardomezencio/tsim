"""Different representations of positions on the network."""

from __future__ import annotations

from dataclasses import dataclass
from typing import TYPE_CHECKING

from dataslots import with_slots

if TYPE_CHECKING:
    from tsim.model.entity import EntityRef
    from tsim.model.network.way import Lane, OrientedWay, Way


@with_slots
@dataclass
class WayPosition:
    """A position in a `Way`.

    The position is in meters from the start of the way.
    """

    way: EntityRef[Way]
    position: float


@with_slots
@dataclass
class OrientedWayPosition:
    """A position in an `OrientedWay`.

    The position is in meters from the endpoint of the oriented way.
    """

    oriented_way: OrientedWay
    position: float


@with_slots
@dataclass
class LanePosition:
    """A position in a `Lane`.

    The position is in meters from the start of the lane.
    """

    lane: Lane
    position: float
