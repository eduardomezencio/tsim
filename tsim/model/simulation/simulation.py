"""Simulation class."""

from __future__ import annotations

from collections import deque
from typing import Callable, Deque, Set, Tuple

from tsim.model.network.traffic import TrafficDynamicAgent
from tsim.model.units import Duration, Timestamp


class Simulation:
    """Agent-based simulation."""

    time: Timestamp
    agents: Set[TrafficDynamicAgent]
    active: Set[TrafficDynamicAgent]
    ready_buffer: int
    _queue: Deque[Tuple[Callable, Tuple]]

    def __init__(self):
        self.time = 0
        self.agents = set()
        self.active = set()
        self.ready_buffer = 0
        self._queue = deque()

    @property
    def target_buffer(self) -> int:
        """Get index of current target buffer."""
        return (self.ready_buffer + 1) % 2

    def add(self, agent: TrafficDynamicAgent):
        """Add agent to the simulation."""
        self.agents.add(agent)
        self.update_active_set(agent)

    def update(self, dt: Duration):
        """Update the simulation with time step of `dt`."""
        ready = self.ready_buffer
        target = (ready + 1) % 2

        for agent in self.active:
            agent.update(dt, ready, target)

        while self._queue:
            callable_, args = self._queue.popleft()
            callable_(*args)

        self.time += dt
        self.flip_buffer()

    def flip_buffer(self):
        """Flip ready buffer index."""
        self.ready_buffer = self.target_buffer

    def update_active_set(self, agent: TrafficDynamicAgent):
        """Activate or deactivate agent according to its `active` attribute.

        When an agent is activated or deactivated during update, this should be
        called instead of adding or removing directly from the active set,
        because the set is being iterated and cannot change during iteration.
        """
        self.enqueue(self.active.add if agent.active else self.active.discard,
                     (agent,))

    def enqueue(self, callable_: Callable, args: Tuple):
        """Enqueue function to be called after update loop ends."""
        self._queue.append((callable_, args))

    # TODO: Remove or rewrite in a way to not mix panda3d in simulation code.
    def debug_focus(self, agent):
        """Send panda3d message to focus on agent.

        FOR DEBUG ONLY.
        """
        import tsim.ui.panda3d as p3d
        p3d.MESSENGER.send('focus', [agent])
