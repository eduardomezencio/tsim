"""Classes related to the agent's schedule."""

from __future__ import annotations

from bisect import bisect_right
from typing import List, NamedTuple, Tuple

from tsim.core.network.orientedway import OrientedWayPosition
from tsim.core.units import Timestamp


class ScheduleEntry(NamedTuple):
    """Entry of the schedule."""

    timestamp: Timestamp
    address: OrientedWayPosition

    def __lt__(self, value: ScheduleEntry):
        return self.timestamp < value.timestamp

    def __le__(self, value: ScheduleEntry):
        return self.timestamp <= value.timestamp

    def __eq__(self, value: ScheduleEntry):
        return self.timestamp == value.timestamp

    def __gt__(self, value: ScheduleEntry):
        return self.timestamp > value.timestamp

    def __ge__(self, value: ScheduleEntry):
        return self.timestamp >= value.timestamp


class Schedule:
    """Schedule for an agent."""

    _entries: List[ScheduleEntry]
    _sorted: bool

    def __init__(self):
        self._entries = []
        self._sorted = True

    @property
    def entries(self):
        """Get schedule entries sorted."""
        if not self._sorted:
            self._entries.sort()
        return self._entries

    def query(self, timestamp: Timestamp) \
            -> Tuple[ScheduleEntry, ScheduleEntry]:
        """Find entries during (before) and after given timestamp."""
        if len(self.entries) < 2:
            return (self._entries[0],) * 2 if self._entries else None
        index = bisect_right(self._entries, (timestamp,))
        if index in (0, len(self.entries) + 1):
            return self._entries[-1], self.entries[0]
        return self._entries[index-1:index+1]
