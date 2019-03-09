"""Network related classes for tsim."""

from __future__ import annotations

from dataclasses import dataclass, field
from itertools import chain, islice
from typing import Generator, List, Tuple

from cached_property import cached_property
from dataslots import with_slots

from tsim.model.entity import Entity, EntityRef
from tsim.model.geometry import BoundingRect, distance, Point, Vector


@with_slots
@dataclass
class Node(Entity):
    """A node of the network.

    A Node can be the endpoint of a Way or a junction of 3 or more Ways.
    """

    position: Point
    starts: List[EntityRef[Way]] = field(default_factory=list)
    ends: List[EntityRef[Way]] = field(default_factory=list)

    def calc_bounding_rect(self,
                           accumulated: BoundingRect = None) -> BoundingRect:
        """Calculate the bounding rect of the node."""
        return self.position.calc_bounding_rect(accumulated)

    def disconnect(self):
        """Disconnect this node from the network.

        Returns the ways that must be disconnected to free this node.
        """
        return [r.value for r in set(chain(self.starts, self.ends))]

    on_delete = disconnect


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

    def __post_init__(self):
        self.start.starts.append(EntityRef(self))
        self.end.ends.append(EntityRef(self))

    @cached_property
    def length(self) -> float:
        """Total length of the Way."""
        return sum(distance(p, q) for p, q in
                   zip(self.points(), self.points(skip=1)))

    def calc_bounding_rect(self,
                           accumulated: BoundingRect = None) -> BoundingRect:
        """Calculate the bounding rect of the node."""
        for point in self.points():
            accumulated = point.calc_bounding_rect(accumulated)
        return accumulated

    def other(self, node: Node):
        """Get the other endpoint of the way, opposite to node."""
        return self.start if self.end is node else self.end

    def points(self, skip=0) -> Generator[Point]:
        """Get generator for points in order, including nodes and waypoints."""
        yield from islice(chain((self.start.position,), self.waypoints,
                                (self.end.position,)),
                          skip, None)

    def normals(self) -> Generator[Vector]:
        """Get the 'normals' of this way.

        Normal here is used for the lack of a better term, meaning a unit
        vector pointing to the right side of the road. One is returned for each
        node or waypoint on the way.
        """
        other = self.waypoints[0] if self.waypoints else self.end.position
        yield (other - self.start.position).rotated_right().normalized()

        window = zip(self.points(), self.points(skip=1), self.points(skip=2))
        yield from (((q - p) + (r - q)).rotated_right().normalized()
                    for p, q, r in window)

        other = self.waypoints[-1] if self.waypoints else self.start.position
        yield (self.end.position - other).rotated_right().normalized()

    def disconnect(self):
        """Disconnect this way from the network.

        Removes the connections from this way to nodes and also the references
        from the nodes to this way.
        """
        start_index = next(i for i, v in enumerate(self.start.starts)
                           if v.id == self.id)
        end_index = next(i for i, v in enumerate(self.end.ends)
                         if v.id == self.id)
        del self.start.starts[start_index]
        del self.end.ends[end_index]
        self.start = None
        self.end = None

    on_delete = disconnect
