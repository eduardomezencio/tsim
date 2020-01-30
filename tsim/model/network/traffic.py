"""Base classes for traffic agents."""

from __future__ import annotations

from abc import ABC, abstractmethod
from typing import Optional, Sequence

from tsim.model.network.location import NetworkLocation
from tsim.utils.linkedlist import LinkedList, LinkedListNode


class TrafficAgent(ABC):
    """Base for classes that can be part of the traffic."""

    owner: Optional[TrafficAgent]
    acquiring: Optional[TrafficAgent]
    speed: Sequence[float]
    traffic_node: LinkedListNode[TrafficAgent]

    @abstractmethod
    def get_network_position(self, location: NetworkLocation,
                             buffer: Optional[int] = None) -> float:
        """Get network position in the given `location`.

        Used as sorting key for agents in network locations.
        """

    @abstractmethod
    def is_at(self, location: NetworkLocation,
              buffer: Optional[int] = None) -> bool:
        """Get whether agent is at given `location`."""

    @abstractmethod
    def notify(self):
        """Notify this agent of lead events."""

    @abstractmethod
    def acquire(self, lock: TrafficLock, buffer: Optional[int] = None):
        """Register acquisition of `lock` by agent."""

    @abstractmethod
    def add_follower(self, agent: TrafficDynamicAgent,
                     buffer: Optional[int] = None):
        """Register agent as follower."""

    @abstractmethod
    def remove_follower(self, agent: TrafficDynamicAgent):
        """Unregister agent as follower."""


class TrafficDynamicAgent(TrafficAgent):
    """Base class for agents that move."""

    lead: TrafficAgent
    distance_to_lock: float

    @abstractmethod
    def distance_to_lead(self, buffer: Optional[int] = None) -> float:
        """Get the distance to the lead or infinite if there's no lead."""


class TrafficLock(TrafficAgent):
    """Base class for lockable traffic resources."""

    owner_secondary: Optional[TrafficDynamicAgent]

    @abstractmethod
    def lock(self, agent: TrafficAgent, terminal: bool = False):
        """Lock this traffic lock to `agent`."""

    @abstractmethod
    def release(self, agent: TrafficAgent, terminal: bool = False):
        """Release this lock by `agent`."""


Traffic = LinkedList[TrafficAgent]  # pylint: disable=invalid-name
