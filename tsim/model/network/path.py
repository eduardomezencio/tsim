"""Path and path finding related functions."""

from __future__ import annotations

import sys
from collections import defaultdict
from dataclasses import dataclass, field
from itertools import islice
from operator import attrgetter
from typing import (TYPE_CHECKING, DefaultDict, Dict, Iterator, List, Set,
                    Tuple)

from dataslots import with_slots
from fibonacci_heap_mod import (Fibonacci_heap as FibonacciHeap,
                                Entry as FibonacciHeapEntry)

from tsim.model.geometry import Vector, sec
from tsim.model.network.lane import LANE_WIDTH
from tsim.model.network.orientedway import OrientedWay, OrientedWayPosition
from tsim.utils.iterators import window_iter

SingleSourceMap = Dict[OrientedWay, Tuple[OrientedWay, int]]
AllPairsMap = Dict[OrientedWay, SingleSourceMap]

if TYPE_CHECKING:
    from tsim.model.geometry import Point

INF = sys.float_info.max


@with_slots
@dataclass
class _NodeInfo:
    dist: float = field(default=INF)
    prev: OrientedWay = field(default=None)
    entry: FibonacciHeapEntry = field(default=None)
    depth: int = field(default=0)


def dijkstra(source: OrientedWay) -> SingleSourceMap:
    """Dijkstra algorithm."""
    info: DefaultDict[OrientedWay, _NodeInfo]
    node: OrientedWay
    visited: Set[OrientedWay]

    queue = FibonacciHeap()
    visited = set()
    source_info = _NodeInfo(0.0)
    source_info.entry = queue.enqueue(source, 0)
    info = defaultdict(_NodeInfo, {source: source_info})

    while queue:
        node = queue.dequeue_min().get_value()
        node_info = info[node]
        visited.add(node)
        weight = node.weight
        for neighbor in node.way_connections:
            if neighbor in visited:
                continue
            neighbor_info = info[neighbor]
            new_dist = node_info.dist + weight
            if new_dist < neighbor_info.dist:
                if neighbor_info.dist == INF:
                    neighbor_info.entry = queue.enqueue(neighbor, new_dist)
                else:
                    queue.decrease_key_unchecked(neighbor_info.entry, new_dist)
                neighbor_info.dist = new_dist
                neighbor_info.prev = node
                neighbor_info.depth = node_info.depth + 1

    return {n: (i.prev, i.depth) for n, i in info.items()}


@with_slots
@dataclass
class Path:
    """A path between a source and a destination `OrientedWay`."""

    ways: List[OrientedWay]
    offsets: Tuple[float, float]

    @staticmethod
    def build(single_source: SingleSourceMap,
              source: OrientedWay, dest: OrientedWay,
              start_offset: float = 0.0, end_offset: float = 0.0,
              prepend_source: bool = False) -> Path:
        """Create a new Path from a single source map and the destination."""
        empty = (None, None)
        way, index = single_source.get(dest, (None, 0))
        result = [None] * (index + 1)

        if prepend_source:
            result.append(None)
            result[0] = source
        else:
            index -= 1

        result[-1] = dest
        while way:
            result[index] = way
            way, _ = single_source.get(way, empty)
            index -= 1

        if not prepend_source and result[0] != source:
            result = []

        return Path(result, (start_offset, end_offset)) if result else None

    @property
    def length(self) -> float:
        """Get the total length of the path."""
        return (sum(w.length for w in islice(self.ways, len(self.ways) - 1))
                - self.offsets[0] + self.offsets[1])

    @property
    def weight(self) -> float:
        """Get the total weight of the path.

        This property is not taking into account the start and end offsets,
        counting the full weight of both oriented ways.
        """
        return sum(w.weight for w in self.ways)

    def points(self) -> Iterator[Point]:
        """Get generator for points in order, including nodes and waypoints.

        This is meant to be used as a helper for drawing the path.
        """
        if not self.ways:
            return
        if len(self.ways) == 1:
            yield from self.ways[0].points(offsets=self.offsets)
            return

        yield from self.ways[0].points(offsets=(self.offsets[0], None))
        for last_way, way in window_iter(islice(self.ways,
                                                len(self.ways) - 1)):
            skip = 0 if way == last_way.flipped() else 1
            yield from way.points(skip=skip)
        last_way, way = self.ways[-2:]
        skip = 0 if way == last_way.flipped() else 1
        yield from way.points(offsets=(0.0, self.offsets[1]), skip=skip)

    def oriented_points(self, offset=LANE_WIDTH) -> List[Point]:
        """Get points shifted by orientation to help drawing path."""
        def width2(point1: Point, point2: Point) -> Vector:
            return (point2 - point1).normalized().rotated_right() * offset

        def width3(point1: Point, point2: Point, point3: Point) -> Vector:
            last = (point2 - point1).normalized()
            vector = (point3 - point2).normalized()
            bisector = (last + vector).normalized()
            return sec(bisector, vector) * offset * bisector.rotated_right()

        points = list(self.points())
        if len(points) < 2:
            return points

        points_ = [points[0] + width2(points[0], points[1])]
        for point1, point2, point3 in window_iter(points, size=3):
            if point1.close_to(point2):
                points_.append(point2 + width2(point2, point3))
            elif point2.close_to(point3):
                points_.append(point2 + width2(point1, point2))
            else:
                points_.append(point2 + width3(point1, point2, point3))
        points_.append(points[-1] + width2(points[-2], points[-1]))
        return points_


@with_slots
@dataclass
class PathMap:
    """Contains the shortest path between all pairs of oriented ways."""

    all_pairs: AllPairsMap = field(default_factory=dict)

    def path(self,
             source: OrientedWayPosition,
             dest: OrientedWayPosition) -> Path:
        """Get path from `source` to `dest`.

        Can return None if there is no path between the given oriented ways.
        """
        if source.oriented_way == dest.oriented_way \
                and source.position > dest.position:
            return self._path_to_same(source.oriented_way,
                                      source.position, dest.position)

        single_source = self.single_source(source.oriented_way)
        return Path.build(single_source,
                          source.oriented_way, dest.oriented_way,
                          source.position, dest.position)

    def single_source(self, source: OrientedWay) -> SingleSourceMap:
        """Get the single source map from the `source` oriented way."""
        single_source = self.all_pairs.get(source, None)
        if single_source is None:
            single_source = dijkstra(source)
            self.all_pairs[source] = single_source
        return single_source

    def _path_to_same(self, oriented_way: OrientedWay,
                      source: float, dest: float) -> Path:
        """Get a path between two points on the same way.

        This method is specifically for the case where the destination is
        behind the source, so a path must be found to reach the same oriented
        way from the other side.
        """
        paths = []
        for way in oriented_way.way_connections:
            single_source = self.single_source(way)
            paths.append(Path.build(single_source, oriented_way, oriented_way,
                                    source, dest, prepend_source=True))
        return min(paths, key=attrgetter('length'), default=None)
