"""Intersection class and related functions."""

from __future__ import annotations

from collections import defaultdict, deque
from dataclasses import dataclass, field
from itertools import chain, combinations, islice, product
from math import pi
from typing import (TYPE_CHECKING, DefaultDict, Deque, Dict, Iterator, List,
                    Set, Tuple)

import bezier
import numpy
from dataslots import with_slots
from rtree.index import Rtree

from tsim.model.geometry import Point, angle, line_intersection_safe
from tsim.model.network.way import (HALF_LANE_WIDTH, LANE_WIDTH, Lane,
                                    OrientedWay)

if TYPE_CHECKING:
    from tsim.model.network.node import Node

    LaneConnection = Tuple[Lane, Lane]
    LaneConnections = Dict[Lane, Set[Lane]]

MERGE_RADIUS = 0.1
NEIGHBOR_RADIUS = 1.0


class Intersection:
    """Intersection information for a node.

    Contains lane connections and conflict points.
    """

    __slots__ = ('connections', 'curves')

    connections: LaneConnections
    curves: Dict[LaneConnection, Curve]

    def __init__(self, node: Node):
        self.connections = build_lane_connections(node)
        self.curves = build_bezier_curves(node, self.connections)
        build_conflict_points(self.curves)

    def iterate_connections(self) -> Iterator[LaneConnection]:
        """Get iterator for connections as Tuple[Lane, Lane]s."""
        for source, dests in self.connections.items():
            yield from product((source,), dests)

    def curve_points(self, lane_connection: LaneConnection = None) \
            -> Tuple[Point]:
        """Get control points of curve for the given lane connection."""
        nodes = self.curves[lane_connection].curve.nodes
        return tuple(Point(*nodes[:, i]) for i in range(3))

    def iterate_connections_curve_points(self) \
            -> Iterator[Tuple[LaneConnection, Tuple[Point]]]:
        """Get iterator for curve points for all lane connections."""
        yield from ((c, self.curve_points(c))
                    for c in self.iterate_connections())


def build_lane_connections(node: Node) -> LaneConnections:
    """Calculate default lane connections for the given node."""
    angles: List[float]
    connections: DefaultDict[Lane, Set[Lane]]
    new_conn: DefaultDict[Lane, Set[Lane]]
    way: OrientedWay
    ways: Deque[OrientedWay]

    def iterate_new_conn() -> Iterator[Tuple[Lane, Lane]]:
        for key, values in new_conn.items():
            yield from product((key,), values)

    def curvier(angle_a: float, angle_b: float) -> bool:
        return int(abs(pi - angle_a) < abs(pi - angle_b))

    def rotate_angles():
        angles[0] = 2 * pi
        size = len(angles)
        angles[0:] = [angles[i + 1 - size] - angles[1]
                      for i in range(size)]

    def self_connect():
        """Connect ways[0] to itself."""
        connections.update({l: {o} for l, o in zip(
            way.iterate_lanes(opposite_only=True),
            way.iterate_lanes(l_to_r=False))})

    def connect_to(index: int, other: OrientedWay) -> LaneConnections:
        """Connect ways[0] to the given way."""
        outwards = angles[index] > pi * 2.0 / 3.0
        for source, dest in zip(way.iterate_lanes(l_to_r=not(outwards),
                                                  opposite_only=True),
                                other.iterate_lanes(outwards)):
            new_conn[source].add(dest)
            conn_angles[(source, dest)] = angles[index]

    def resolve_conflicts():
        """Leave only one connection when there are conflicts."""
        to_remove = set()
        for conn_1, conn_2 in combinations(iterate_new_conn(), 2):
            if conn_1[0] == conn_2[0]:
                continue
            if conn_1 in to_remove or conn_2 in to_remove:
                continue
            conn = ((conn_2, conn_1) if conn_1[0].index > conn_2[0].index
                    else (conn_1, conn_2))
            angle_1, angle_2 = conn_angles[conn[0]], conn_angles[conn[1]]
            if angle_1 > angle_2:
                to_remove.add(conn[curvier(angle_1, angle_2)])
        for conn in to_remove:
            new_conn[conn[0]].remove(conn[1])

    connections = defaultdict(set)
    new_conn = defaultdict(set)
    conn_angles = {}
    ways = deque(node.sorted_ways())
    if ways:
        way = ways[0]
        if len(ways) == 1:
            self_connect()
        else:
            way_vector = way.way.direction_from_node(node, way.endpoint)

            angles = [angle(way_vector, w.direction_from_node(node, d))
                      if i > 0 else 0.0 for i, (w, d) in enumerate(ways)]

            for _ in range(len(ways)):
                new_conn.clear()
                conn_angles.clear()
                for i, way_ in islice(enumerate(ways), 1, None):
                    connect_to(i, way_)
                resolve_conflicts()
                connections.update(new_conn)
                ways.rotate(-1)
                way = ways[0]
                rotate_angles()

    return connections


def build_bezier_curves(node: Node, connections: LaneConnections) \
        -> Dict[LaneConnection, Curve]:
    """Create bezier curves for each lane connection on the node."""
    vectors = {w: w.way.direction_from_node(node, w.endpoint).normalized()
               for w in node.oriented_ways}

    points = {}
    for way in node.oriented_ways:
        vector = vectors[way]
        right = vector.rotated_right()
        first = right * Lane(*way, 0).distance_from_center()
        for lane in way.iterate_lanes(include_opposite=True):
            points[lane] = (
                vector * (node.geometry.distance(way) + HALF_LANE_WIDTH)
                + right * lane.index * LANE_WIDTH
                + first)

    crossings = {}
    for source, dests in connections.items():
        for lane1, lane2 in product((source,), dests):
            crossings[(lane1, lane2)] = line_intersection_safe(
                points[lane1], vectors[lane1.oriented_way],
                points[lane2], vectors[lane2.oriented_way])

    curves = {}
    for lanes, crossing in crossings.items():
        start, end = map(points.get, lanes)
        curve = bezier.Curve(
            numpy.asfortranarray(list(zip(start, crossing, end))), degree=2)
        curves[lanes] = Curve(curve)

    return curves


def build_conflict_points(curves: Dict[LaneConnection, Curve]):
    """Create conflict points from lane connection curves.

    Fill conflict points in given Curve objects. Merges points that are less
    than MERGE_RADIUS apart and add points that are within NEIGHBOR_RADIUS as
    neighbors.
    """
    points = {}
    for curve1, curve2 in combinations(curves.values(), 2):
        try:
            intersection = curve1.intersect(curve2)[:, 0]
            point = ConflictPoint(curve1.evaluate(intersection[0]))
            curve1.add_conflict_point(intersection[0], point)
            curve2.add_conflict_point(intersection[1], point)
            points[id(point)] = point
        except IndexError:
            pass

    if len(points) < 2:
        return

    rtree = Rtree((id_, p.point.bounding_rect, None)
                  for id_, p in points.items())
    merge_points(points, rtree)
    fill_neighbors(points, rtree)


def merge_points(points: Dict[int, ConflictPoint], rtree: Rtree):
    """Merge conflict points closer than MERGE_RADIUS."""
    merged = set()
    for id_, point in points.items():
        if id_ in merged:
            continue
        for other_id in rtree.intersection(
                point.point.enclosing_rect(MERGE_RADIUS)):
            if other_id == id_:
                continue
            other = points[other_id]
            for curve in other.curves:
                curve.replace_conflict_point(other, point)
            merged.add(other_id)
            rtree.delete(other_id, other.point.bounding_rect)
    for id_ in merged:
        del points[id_]


def fill_neighbors(points: Dict[int, ConflictPoint], rtree: Rtree):
    """Add conflict points closer than NEIGHBOR_RADIUS as neighbors."""
    for id_, point in points.items():
        for other_id in rtree.intersection(
                point.point.enclosing_rect(NEIGHBOR_RADIUS)):
            if other_id == id_:
                continue
            point.neighbors.add(points[other_id])


@with_slots
@dataclass(eq=False)
class ConflictPoint:
    """Conflict point in an intersection.

    The point where two lane connections cross.
    """

    point: Point
    neighbors: Set[ConflictPoint] = field(default_factory=set, repr=False)
    curves: Set[Curve] = field(default_factory=set, repr=False)


class Curve:
    """The curve of a lane connection.

    Contains the bezier curve connecting one lane to the other and the conflict
    points that intersect with this curve.
    """

    __slots__ = ('_curve', '_conflict_points', '_sorted')

    curve: bezier.Curve
    length: float
    _conflict_points: List[Tuple[float, ConflictPoint]]
    _sorted: bool

    def __init__(self, curve: bezier.Curve):
        self.curve = curve
        self.length = curve.length
        self._conflict_points = []
        self._sorted = True

    @property
    def conflict_points(self) -> Iterator(Tuple[float, ConflictPoint]):
        """Get iterator for sorted conflict points."""
        if not self._sorted:
            self._conflict_points.sort()
            self._sorted = True
        yield from self._conflict_points

    def add_conflict_point(self, param: float, point: ConflictPoint):
        """Add conflict point to curve, with given param for sorting."""
        self._sorted = False
        self._conflict_points.append((param, point))
        point.curves.add(self)

    def replace_conflict_point(self, old: ConflictPoint, new: ConflictPoint):
        """Replace conflict point, for use when merging conflict points."""
        for i, (param, point) in enumerate(self._conflict_points):
            if point is old:
                self._conflict_points[i] = (param, new)
                new.curves.add(self)
                break

    def intersect(self, other: Curve) -> numpy.ndarray:
        """Calculate intersection of bezier curves."""
        return self.curve.intersect(other.curve)

    def evaluate(self, param: float) -> Point:
        """Evaluate bezier curve at t=param."""
        return Point(*chain.from_iterable(self.curve.evaluate(param)))
