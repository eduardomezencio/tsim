"""Network location and position base classes."""

from __future__ import annotations

from abc import ABC, abstractmethod, abstractproperty
from dataclasses import dataclass
from typing import TYPE_CHECKING

from dataslots import with_slots

from tsim.utils.linkedlist import LinkedListNode

if TYPE_CHECKING:
    from tsim.core.geometry import Point, Vector
    from tsim.core.network.orientedway import OrientedWayPosition
    from tsim.core.network.traffic import Traffic, TrafficAgent


class NetworkLocation(ABC):
    """Abstract base for a location in the network.

    A network location is an object with a single line of traffic flow on it.
    """

    length: float
    traffic: Traffic

    @abstractproperty
    def lane_index(self):
        """Get lane index of the network location or its destination."""

    @abstractproperty
    def segment_count(self):
        """Get the number of segments of this network location."""

    @abstractmethod
    def oriented_way_position(self, position: float) -> OrientedWayPosition:
        """Get `OrientedWayPosition` from this network location."""

    def insert_agent(self, agent: TrafficAgent, buffer: int) \
            -> LinkedListNode[TrafficAgent]:
        """Insert agent in traffic in sorted position."""
        return self.traffic.insort(
            agent, key=lambda a: a.get_network_position(self, buffer))


@with_slots
@dataclass(frozen=True)
class WorldAndSegmentPosition:
    """World position and segment information bundled together."""

    position: Point
    direction: Vector
    segment: int
    segment_end: float


class NetworkPosition(ABC):
    """Abstract base for a position in a `NetworkLocation`."""

    position: float

    @abstractproperty
    def location(self) -> NetworkLocation:
        """Get the `NetworkLocation` of this position."""

    @abstractmethod
    def world_and_segment_position(self) -> WorldAndSegmentPosition:
        """Get world and segment position at this position."""
