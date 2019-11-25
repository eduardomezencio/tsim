"""OrientedWay class."""

from __future__ import annotations

from typing import (TYPE_CHECKING, Iterable, Iterator, NamedTuple, Optional,
                    Tuple)

from tsim.model.entity import EntityRef
from tsim.model.network.endpoint import Endpoint

if TYPE_CHECKING:
    from tsim.model.geometry import Point
    from tsim.model.network.lane import Lane, LaneRef
    from tsim.model.network.node import Node
    from tsim.model.network.way import Way


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
    def way_id(self) -> int:
        """Get the id of the referenced way."""
        return self.way_ref.id

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
    def start_offset(self):
        """Distance from the start node to the start of the way geometry."""
        return (self.way.start_offset if self.endpoint is Endpoint.START
                else self.way.end_offset)

    @property
    def end_offset(self):
        """Distance from the end node to the end of the way geometry."""
        return (self.way.end_offset if self.endpoint is Endpoint.START
                else self.way.start_offset)

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

    @property
    def way_connections(self) -> Iterable[OrientedWay]:
        """Get the way connections from the end of this oriented way."""
        return self.end.way_connections(self)

    def flipped(self) -> OrientedWay:
        """Get OrientedWay with same way in the other direction."""
        return OrientedWay(self.way_ref, self.endpoint.other)

    def lane(self, index: int) -> Lane:
        """Get lane with given index in the direction of the oriented way."""
        return self.way.lanes[index if self.endpoint is Endpoint.START
                              else -(index + 1)]

    def lane_refs(self, l_to_r: bool = True, include_opposite: bool = False,
                  opposite_only: bool = False,
                  positive: bool = False) -> Iterator[LaneRef]:
        """Get lane references in the direction of the oriented way.

        A shortcut for Way.lane_refs.
        """
        return self.way.lane_refs(self.endpoint, l_to_r, include_opposite,
                                  opposite_only, positive)

    def points(self, offsets: Optional[Tuple[float, Optional[float]]] = None,
               skip: int = 0) -> Iterator[Point]:
        """Get generator for points in order, including nodes and waypoints."""
        return self.way.points(offsets=offsets, skip=skip,
                               reverse=self.endpoint is Endpoint.END)

    def __repr__(self):
        return (f'{OrientedWay.__name__}(way_id={self.way.id}, '
                f'endpoint={self.endpoint.name[0]})')
