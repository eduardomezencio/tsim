"""Simulation class."""

from __future__ import annotations

from typing import Set

from tsim.model.simulation.agent import Agent
from tsim.model.units import Duration


class Simulation:
    """Agent-based simulation."""

    time: int
    agents: Set[Agent]
    active: Set[Agent]
    ready_buffer: int
    _active_updates: Set[Agent]
    _in_update: bool

    def __init__(self):
        self.time = 0
        self.agents = set()
        self.active = set()
        self.ready_buffer = 0
        self._active_updates = set()
        self._in_update = False

    @property
    def target_buffer(self) -> int:
        """Get index of current target buffer."""
        return (self.ready_buffer + 1) % 2

    def add(self, agent: Agent):
        """Add agent to the simulation."""
        self.agents.add(agent)
        self.update_active_set(agent)

    def update(self, dt: Duration):
        """Update the simulation with time step of `dt`."""
        ready = self.ready_buffer
        target = (ready + 1) % 2

        self._in_update = True
        for agent in self.active:
            agent.update(dt, ready, target)
        self._in_update = False

        if self._active_updates:
            for agent in self._active_updates:
                if agent.active:
                    self.active.add(agent)
                else:
                    self.active.remove(agent)
            self._active_updates.clear()

        self.time += dt
        self.flip_buffer()

    def flip_buffer(self):
        """Flip ready buffer index."""
        self.ready_buffer = (self.ready_buffer + 1) % 2

    def update_active_set(self, agent: Agent):
        """Activate or deactivate agent according to its `active` attribute.

        When an agent is activated or deactivated during update, this should be
        called instead of adding or removing directly from the active set,
        because the set is being iterated and cannot change during iteration.
        """
        if self._in_update:
            self._active_updates.add(agent)
        elif agent.active:
            self.active.add(agent)
        else:
            self.active.remove(agent)
