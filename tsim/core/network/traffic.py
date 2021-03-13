"""Base classes for traffic agents."""

from __future__ import annotations

from abc import ABC, abstractmethod
from typing import Deque, Optional, Mapping, Sequence

from tsim.core.network.location import NetworkLocation
from tsim.utils.linkedlist import LinkedList, LinkedListNode


class TrafficAgent(ABC):
    """Base for classes that can be part of the traffic."""

    active: bool
    owner: Optional[TrafficAgent]
    speed: Sequence[float]
    traffic_node: LinkedListNode[TrafficAgent]

    @abstractmethod
    def get_network_position(self, location: NetworkLocation,
                             buffer: int) -> float:
        """Get network position in the given `location`.

        Used as sorting key for agents in network locations.
        """

    @abstractmethod
    def is_at(self, location: NetworkLocation) -> bool:
        """Get whether agent is at given `location`."""

    @abstractmethod
    def add_follower(self, agent: TrafficDynamicAgent, buffer: int):
        """Register agent as follower."""

    @abstractmethod
    def update_followers(self, buffer: int):
        """Update lead of registered followers."""

    @abstractmethod
    def remove_follower(self, agent: TrafficDynamicAgent, buffer: int):
        """Unregister agent as follower."""

    @abstractmethod
    def notify(self, buffer: int):
        """Notify this agent of lead events."""

    @abstractmethod
    def notify_followers(self, buffer: int):
        """Notify followers of events."""


class TrafficDynamicAgent(TrafficAgent):
    """Base class for agents that move."""

    lead: TrafficAgent
    lock_queue: Deque[TrafficLock]
    lock_count: Mapping[TrafficLock, int]
    distance_to_lock: float

    @abstractmethod
    def acquire(self, lock: TrafficLock, buffer: int, terminal: bool):
        """Register acquisition of `lock` by agent."""

    @abstractmethod
    def distance_to_lead(self, buffer: int) -> float:
        """Get the distance to the lead or infinite if there's no lead."""


class TrafficLock(TrafficAgent):
    """Base class for lockable traffic resources."""

    @abstractmethod
    def lock(self, agent: TrafficDynamicAgent, buffer: int,
             terminal: bool = False, location: NetworkLocation = None):
        """Lock this traffic lock to `agent`."""

    @abstractmethod
    def release(self, agent: TrafficDynamicAgent, buffer: int,
                terminal: bool = False):
        """Release this lock by `agent`."""


Traffic = LinkedList[TrafficAgent]  # pylint: disable=invalid-name
