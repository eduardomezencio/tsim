"""Way statistics collection."""

from collections import defaultdict
from dataclasses import dataclass
from functools import partialmethod
from math import inf
from typing import Callable, Dict

from dataslots import with_slots
from intervaltree import Interval, IntervalTree

from tsim.model.network.orientedway import OrientedWay, OrientedWayPosition
from tsim.model.network.traffic import TrafficDynamicAgent
from tsim.model.units import Timestamp, mps_to_kph, MINUTE


@with_slots
@dataclass(frozen=True)
class WayStats:
    """Way statistics."""

    average_speed_kph: float = 0.0
    throughput_cars_per_minute: float = 0.0


class WayStatsCollector:
    """Collector and processor of `Way` statistics.

    Collects information using the `push` method and calculates statistics
    using the `collect` method. This class keeps all information collected and
    can return statistics from any period in the past.
    """

    _data: IntervalTree
    _intervals: Dict[TrafficDynamicAgent, Interval]
    _listeners: Dict[str, Callable]

    def __init__(self):
        self._data = IntervalTree()
        self._intervals = {}

    def push(self, entered: bool, agent: TrafficDynamicAgent,
             timestamp: Timestamp, oriented_way_position: OrientedWayPosition):
        """Push new data traffic flow data."""
        if entered:
            interval = Interval(timestamp, inf, oriented_way_position)
            self._intervals[agent] = interval
            self._data.add(interval)
        else:
            interval = self._intervals.get(agent, None)
            if interval is not None:
                begin, data = interval.begin, interval.data
                data = (data.oriented_way, data.position,
                        oriented_way_position.position)
                self._data.remove(interval)
                interval = Interval(begin, timestamp, data)
                self._data.add(interval)
                del self._intervals[agent]

    def collect(self, start_timestamp, end_timestamp, copy_tree=False) \
            -> Dict[OrientedWay, WayStats]:
        """Calculate and return way statistics in given period."""
        tree = IntervalTree(self._data) if copy_tree else self._data
        data = (d for d in tree[start_timestamp:end_timestamp]
                if not isinstance(d.data, OrientedWayPosition))
        speeds = defaultdict(list)
        scale = MINUTE / (end_timestamp - start_timestamp)

        for begin, end, stats in data:
            oriented_way, start_position, end_position = stats
            mps = (end_position - start_position) / (end - begin)
            speeds[oriented_way].append(mps_to_kph(mps))

        return {w: WayStats(sum(s) / len(s), len(s) * scale)
                for w, s in speeds.items()}

    on_entered_way = partialmethod(push, True)
    on_left_way = partialmethod(push, False)
