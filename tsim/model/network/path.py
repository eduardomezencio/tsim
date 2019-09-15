"""Path and path finding related functions."""

from __future__ import annotations

import sys
from collections import defaultdict, deque
from dataclasses import dataclass, field
from typing import TYPE_CHECKING, DefaultDict, Dict, Iterable, List

from dataslots import with_slots
from fibonacci_heap_mod import Fibonacci_heap as FibonacciHeap

if TYPE_CHECKING:
    from fibonacci_heap_mod import Entry
    from tsim.model.network.node import Node
    from tsim.model.network.way import OrientedWay

SingleSourceMap = Dict[Node, OrientedWay]
AllPairsMap = Dict[Node, SingleSourceMap]

INF = sys.float_info.max


@with_slots
@dataclass
class _NodeInfo:
    dist: float = field(default=INF)
    prev: OrientedWay = field(default=None)
    entry: Entry = field(default=None)


def dijkstra(source: Node) -> SingleSourceMap:
    """Dijkstra algorithm."""
    queue = FibonacciHeap()
    visited = set()
    source_info = _NodeInfo(0.0, None, None)
    source_info.entry = queue.enqueue(source, 0)
    info: DefaultDict[Node, _NodeInfo] = defaultdict(_NodeInfo,
                                                     {source: source_info})

    while queue:
        node = queue.dequeue_min().get_value()
        node_info = info[node]
        visited.add(node)
        for neighbor, way in node.out_neighbors.items():
            if neighbor in visited:
                continue
            neighbor_info = info[neighbor]
            new_dist = node_info.dist + way.weight
            if new_dist < neighbor_info.dist:
                if neighbor_info.dist == INF:
                    neighbor_info.entry = queue.enqueue(neighbor,
                                                        neighbor_info.dist)
                else:
                    queue.decrease_key_unchecked(neighbor_info.entry,
                                                 neighbor_info.dist)
                neighbor_info.dist = new_dist
                neighbor_info.prev = way

    return {n: i.prev for n, i in info.items()}


@with_slots
@dataclass
class Path:
    """A path from a source node to a dest node."""

    ways: List[OrientedWay]

    @property
    def source(self) -> Node:
        """Get the path's source node."""
        return self.ways[0].start

    @property
    def dest(self) -> Node:
        """Get the path's destination node."""
        return self.ways[-1].end

    @property
    def length(self) -> float:
        """Get the total length of the path."""
        return sum(w.lenth for w in self.ways)

    @property
    def weight(self) -> float:
        """Get the total weight of the path."""
        return sum(w.weight for w in self.ways)


class PathMap:
    """Contains the shortest path between all pairs of nodes."""

    all_pairs: AllPairsMap

    def __init__(self, nodes: Iterable[Node]):
        self.all_pairs = {n: dijkstra(n) for n in nodes}

    def path(self, source: Node, dest: Node) -> Path:
        """Get path from source node to dest node.

        Can return None if there is no path between the given nodes.
        """
        result = deque()
        single_source = self.all_pairs[source]
        # Get last way of the path and build it backwards
        way = single_source.get(dest, None)
        while way:
            result.appendleft(way)
            way = single_source.get(way.start, None)
        return Path(list(result)) if result else None
