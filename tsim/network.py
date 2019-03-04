"""Network related classes for tsim."""

from __future__ import annotations

from dataclasses import dataclass, field
from itertools import chain
from typing import Tuple

from cached_property import cached_property
from dataslots import with_slots

from tsim.entity import Entity
from tsim.geometry import BoundingRect, distance, Point


@with_slots
@dataclass
class Node(Entity):
    """A node of the network.

    A Node can be the endpoint of a Way or a junction of 3 or more Ways.
    """

    position: Point

    def calc_bounding_rect(self,
                           accumulated: BoundingRect = None) -> BoundingRect:
        """Calculate the bounding rect of the node."""
        return self.position.calc_bounding_rect(accumulated)


@with_slots
@dataclass
class Way(Entity):
    """A connection between two Nodes.

    A Way connects two Nodes and can have a list of intermediary points, called
    waypoints.
    """

    start: Node
    end: Node
    waypoints: Tuple[Point] = field(default_factory=tuple)

    @cached_property
    def length(self) -> float:
        """Total length of the Way."""
        return sum(distance(p, q) for p, q in
                   zip(chain((self.start.position,), self.waypoints),
                       chain(self.waypoints, (self.end.position,))))

    def calc_bounding_rect(self,
                           accumulated: BoundingRect = None) -> BoundingRect:
        """Calculate the bounding rect of the node."""
        for point in chain((self.start,), self.waypoints, (self.end,)):
            accumulated = point.calc_bounding_rect(accumulated)
        return accumulated
