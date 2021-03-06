"""Simulation class."""

from __future__ import annotations

from collections import deque
from math import floor
from typing import Any, Callable, Deque, List, Set, Tuple

from orderedset import OrderedSet

from tsim.core.network.traffic import TrafficDynamicAgent
from tsim.core.units import Duration, Timestamp, INT_MINUTE, INT_HOUR, INT_DAY

Listener = Callable[[str, Tuple[Any, ...]], None]

# Event 'passed_second' is checked separately.
TIME_EVENTS = ((INT_MINUTE, 'passed_minute'),
               (INT_HOUR, 'passed_hour'),
               (INT_DAY, 'passed_day'))


class Simulation:
    """Agent-based simulation."""

    time: Timestamp
    agents: Set[TrafficDynamicAgent]
    active: Set[TrafficDynamicAgent]
    ready_buffer: int
    _queue: Deque[Tuple[Callable, Tuple]]
    _listeners = List[Listener]

    def __init__(self):
        self.time = 0
        self.agents = OrderedSet()
        self.active = OrderedSet()
        self.ready_buffer = 0
        self._queue = deque()
        self._listeners = []

    @property
    def target_buffer(self) -> int:
        """Get index of current target buffer."""
        return (self.ready_buffer + 1) % 2

    def add(self, agent: TrafficDynamicAgent):
        """Add agent to the simulation."""
        self.agents.add(agent)
        self.update_active_set(agent)
        self.raise_event('new_agent')

    def update(self, dt: Duration):
        """Update the simulation with time step of `dt`."""
        ready = self.ready_buffer
        target = (ready + 1) % 2

        for agent in self.active:
            agent.update(dt, ready, target)

        while self._queue:
            callable_, args = self._queue.popleft()
            callable_(*args)

        old_time = self.time
        self.time += dt
        self.flip_buffer()
        self.raise_event('simulation_step', dt)
        self._check_time_events(old_time)

    def _check_time_events(self, old_time: Timestamp):
        """Raise time events when appropriate."""
        old_time, time = floor(old_time), floor(self.time)
        if old_time == time:
            return
        self.raise_event('passed_second')

        for timespan, event in TIME_EVENTS:
            if old_time // timespan == time // timespan:
                return
            self.raise_event(event)

    def flip_buffer(self):
        """Flip ready buffer index."""
        self.ready_buffer = self.target_buffer

    def update_active_set(self, agent: TrafficDynamicAgent):
        """Activate or deactivate agent according to its `active` attribute.

        When an agent is activated or deactivated during update, this should be
        called instead of adding or removing directly from the active set,
        because the set is being iterated and cannot change during iteration.
        """
        def _update_active_set():
            if agent.active:
                self.active.add(agent)
            else:
                self.active.discard(agent)
        self.enqueue(_update_active_set)
        self.raise_event('active_set_updated')

    def remove(self, agent: TrafficDynamicAgent):
        """Remove agent from simulation."""
        if agent in self.agents:
            self.agents.remove(agent)
            self.raise_event('removed_agent', agent)

    def enqueue(self, callable_: Callable, args: Tuple = ()):
        """Enqueue function to be called after update loop ends."""
        self._queue.append((callable_, args))

    def register_listener(self, listener: Listener):
        """Register listener to receive raised events."""
        if listener not in self._listeners:
            self._listeners.append(listener)

    def raise_event(self, name: str, *args):
        """Raise event to all registered listeners."""
        for listener in self._listeners:
            listener(name, *args)
