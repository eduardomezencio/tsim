"""Intersection class and related functions."""

from __future__ import annotations

from collections import defaultdict, deque
from dataclasses import dataclass, field
from enum import Enum
from itertools import chain, combinations, islice, product
from math import pi
from statistics import median_low
from typing import (TYPE_CHECKING, Callable, DefaultDict, Deque, Dict,
                    Iterator, List, Optional, Set, Tuple)

import bezier
import numpy
from dataslots import with_slots
from rtree.index import Rtree

from tsim.model.entity import EntityRef
from tsim.model.geometry import Point, Vector, angle, line_intersection_safe
from tsim.model.network.lane import LANE_WIDTH, Lane, LaneRef
from tsim.model.network.position import OrientedWayPosition
from tsim.model.network.way import OrientedWay

if TYPE_CHECKING:
    from tsim.model.network.node import Node

LaneConnection = Tuple[LaneRef, LaneRef]
LaneConnections = Dict[LaneRef, Set[LaneRef]]
WayConnections = Dict[OrientedWay, Set[OrientedWay]]

MERGE_RADIUS = 0.1
NEIGHBOR_RADIUS = 1.8
NEIGHBOR_RADIUS_SQUARED = NEIGHBOR_RADIUS ** 2


class Intersection:
    """Intersection information for a node.

    Contains way and lane connections with their curves and conflict points.

    Attributes:
        node_ref: Reference to the node
        lane_connections: Maps each lane to the set of lanes it connects to.
        way_connections: Maps each incoming oriented way to a set of all
            oriented ways it has lane connections to.
        connection_map: Maps one source lane and one destination oriented way
            to all the lane connections between them.
        curves: Maps each lane connection to its `Curve` object.

    """

    __slots__ = ('node_ref', 'lane_connections', 'way_connections',
                 'connection_map', 'curves')

    node_ref: EntityRef[Node]
    lane_connections: LaneConnections
    way_connections: WayConnections
    connection_map: Dict[Tuple[LaneRef, OrientedWay], LaneConnection]
    curves: Dict[LaneConnection, Curve]

    def __init__(self, node: Node):
        self.node_ref = EntityRef(node)
        self.lane_connections = _build_lane_connections(node)
        self.connection_map = _build_connection_map(node,
                                                    self.lane_connections)
        self.curves = _build_bezier_curves(node, self.lane_connections)
        _build_conflict_points(node, self.curves)
        self._build_way_connections()

    def iterate_connections(self) -> Iterator[LaneConnection]:
        """Get iterator for lane connections in this intersection."""
        for source, dests in self.lane_connections.items():
            yield from product((source,), dests)

    def curve_points(self, lane_connection: LaneConnection = None,
                     relative: bool = True) -> Tuple[Point]:
        """Get control points of curve for the given lane connection."""
        nodes = self.curves[lane_connection].curve.nodes
        points = (Point(*nodes[:, i]) for i in range(3))
        if relative:
            points = map((-self.node_ref().position).add, points)
        return tuple(points)

    def iterate_connections_curve_points(self, relative: bool = True) \
            -> Iterator[Tuple[LaneConnection, Tuple[Point]]]:
        """Get iterator for curve points for all lane connections."""
        yield from ((c, self.curve_points(c, relative))
                    for c in self.iterate_connections())

    def _build_way_connections(self):
        connections = defaultdict(set)
        for source, dests in self.lane_connections.items():
            way = source.positive().oriented_way
            connections[way].update(d.oriented_way for d in dests)
        self.way_connections = dict(connections)


class ConflictPointType(Enum):
    """Types of conflict point."""

    DIVERGE = 0
    MERGE = 1
    CROSSING = 2


@with_slots
@dataclass(eq=False)
class ConflictPoint:
    """Conflict point in an intersection.

    The point where two lane connections cross.
    """

    point: Point
    type: ConflictPointType
    neighbors: Set[ConflictPoint] = field(default_factory=set, repr=False)
    curves: Set[Curve] = field(default_factory=set, repr=False)


class Curve:
    """The curve of a lane connection.

    Contains the bezier curve connecting one lane to the other and the conflict
    points that intersect with this curve. The bezier curves are in world
    coordinates, while the conflict points are in coordinates relative to the
    node position.
    """

    __slots__ = ('node_ref', 'source', 'dest', 'curve', 'length',
                 '_conflict_points', '_sorted')

    node_ref: EntityRef[Node]
    source: Lane
    dest: Lane
    curve: bezier.Curve
    length: float
    _conflict_points: List[Tuple[float, ConflictPoint]]
    _sorted: bool

    def __init__(self, node_ref: EntityRef[Node], lanes: LaneConnection,
                 curve: bezier.Curve):
        self.node_ref = node_ref
        self.source, self.dest = (lane_ref() for lane_ref in lanes)
        self.curve = curve
        self.length = curve.length
        self._conflict_points = []
        self._sorted = True

    @property
    def node(self) -> Node:
        """Get the referenced node, where the curve is located."""
        return self.node_ref()

    @property
    def conflict_points(self) -> Iterator(Tuple[float, ConflictPoint]):
        """Get iterator for sorted conflict points."""
        if not self._sorted:
            self._conflict_points.sort()
            self._sorted = True
        yield from self._conflict_points

    @property
    def lane_index(self) -> int:
        """Get lane index from destination."""
        return self.dest.index

    @property
    def segment_count(self) -> int:
        """Get the number of segments.

        A `Curve` has always one single segment. This property exists so that
        the curve can be used as a `NetworkLocation`.
        """
        return 1

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
                return
        raise ValueError('Old conflict point not found in curve.')

    def remove_conflict_point_duplicates(self):
        """Let each conflict point appear only once on this curve."""
        point_map = defaultdict(list)
        for param, cpoint in self.conflict_points:
            point_map[cpoint].append(param)
        for cpoint, params in list(point_map.items()):
            if len(params) <= 1:
                continue
            self._conflict_points.remove((median_low(params), cpoint))

    def intersect(self, other: Curve) -> numpy.ndarray:
        """Calculate intersection of bezier curves."""
        try:
            return self.curve.intersect(other.curve)
        except NotImplementedError:
            return numpy.array([])

    def oriented_way_position(self, _position: float) -> OrientedWayPosition:
        """Get oriented way position from this curve.

        Implements the `NetworkLocation` interface. This method ignores the
        position argument and returns an `OrientedWayPosition` in the start of
        the destination way of this curve.
        """
        oriented_way = self.dest.oriented_way
        return OrientedWayPosition(oriented_way, oriented_way.start_offset)

    def world_position(self, position: float) -> Tuple[Point, Vector]:
        """Get world position and direction in the given curve position."""
        param = position / self.length

        params = ((param, (position + 0.1) / self.length)
                  if position < 0.1 else
                  ((position - 0.1) / self.length, param))

        evaluated = self.curve.evaluate_multi(numpy.array(params))
        points = (Point(*chain.from_iterable(evaluated[:, 0])),
                  Point(*chain.from_iterable(evaluated[:, 1])))

        point = (points[0 if position < 0.1 else 1] + self.node.position)

        vector = (points[1] - points[0]).normalized()
        return point, vector

    def evaluate(self, param: float) -> Point:
        """Evaluate bezier curve at t=param."""
        return Point(*chain.from_iterable(self.curve.evaluate(param)))

    def evaluate_position(self, position: float,
                          curve_override: Optional[bezier.Curve] = None) \
            -> Point:
        """Evaluate bezier curve at t=position/length."""
        curve = self.curve if curve_override is None else curve_override
        return Point(*chain.from_iterable(curve.evaluate(position /
                                                         self.length)))


def _build_lane_connections(node: Node) -> LaneConnections:
    """Calculate default lane connections for the given node."""
    angles: List[float]
    connections: DefaultDict[LaneRef, Set[LaneRef]]
    new_conn: DefaultDict[LaneRef, Set[LaneRef]]
    way: OrientedWay
    ways: Deque[OrientedWay]

    def iterate_new_conn() -> Iterator[LaneConnection]:
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
            way.lane_refs(opposite_only=True, positive=True),
            way.lane_refs(l_to_r=False))})

    def connect_to(index: int, other: OrientedWay) -> LaneConnections:
        """Connect ways[0] to the given way."""
        outwards = angles[index] > pi * 2.0 / 3.0
        for source, dest in zip(way.lane_refs(l_to_r=not(outwards),
                                              opposite_only=True,
                                              positive=True),
                                other.lane_refs(outwards)):
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
            conn = ((conn_1, conn_2) if conn_1[0].index > conn_2[0].index
                    else (conn_2, conn_1))
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

            angles = [angle(way_vector, w().direction_from_node(node, d))
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


def _build_connection_map(node: Node, connections: LaneConnections) \
        -> Dict[Tuple[LaneRef, OrientedWay], LaneConnection]:
    """Create connection map from (LaneRef, OrientedWay) keys.

    The map contains all lanes that reach the given node. The lane connection
    it maps to can be from the lane itself or from another lane if there is no
    connection from the given lane to the oriented way.
    """
    def index_diff(lane: LaneRef) -> Callable[[LaneConnection], int]:
        def index_diff_(connection: LaneConnection) -> int:
            return abs(connection[0].index - lane.index)
        return index_diff_

    result = {}
    for way in node.oriented_ways:
        missing = []
        found = defaultdict(list)
        for lane, dest_way in product(way.lane_refs(opposite_only=True,
                                                    positive=True),
                                      node.oriented_ways):
            dest_lane = next((l for l in connections[lane]
                              if l.oriented_way == dest_way),
                             None)
            if dest_lane is not None:
                found[dest_way].append((lane, dest_lane))
            else:
                missing.append((lane, dest_way))

        for dest_way, connections_ in found.items():
            for lane, dest_lane in connections_:
                result[lane, dest_way] = (lane, dest_lane)

        for lane, dest_way in missing:
            found_ = found[dest_way]
            if found_:
                result[lane, dest_way] = min(found_, key=index_diff(lane))

    return result


def _build_bezier_curves(node: Node, connections: LaneConnections) \
        -> Dict[LaneConnection, Curve]:
    """Create bezier curves for each lane connection on the node."""
    vectors = {w: w.way.direction_from_node(node, w.endpoint).normalized()
               for w in node.oriented_ways}

    points = {}
    for way in node.oriented_ways:
        vector = vectors[way]
        right = vector.rotated_right()
        first = right * LaneRef(*way, 0).distance_from_center()
        for lane in way.lane_refs(include_opposite=True):
            points[lane.positive()] = (vector * node.geometry.distance(way)
                                       + right * lane.index * LANE_WIDTH
                                       + first)

    crossings = {}
    for source, dests in connections.items():
        for lane1, lane2 in product((source,), dests):
            crossings[(lane1, lane2)] = line_intersection_safe(
                points[lane1], vectors[lane1.oriented_way.flipped()],
                points[lane2], vectors[lane2.oriented_way])

    curves = {}
    node_ref = EntityRef(node)
    for lanes, crossing in crossings.items():
        start, end = map(points.get, lanes)
        points_ = list(zip(*map(node.position.add, (start, crossing, end))))
        curve = bezier.Curve(numpy.asfortranarray(points_), degree=2)
        curves[lanes] = Curve(node_ref, lanes, curve)

    return curves


def _build_conflict_points(node: Node, curves: Dict[LaneConnection, Curve]):
    """Create conflict points from lane connection curves.

    Fill conflict points in given Curve objects. Merges points that are less
    than MERGE_RADIUS apart and add points that are within NEIGHBOR_RADIUS as
    neighbors.
    """
    points: Dict[int, ConflictPoint] = {}
    diverge = defaultdict(set)
    merge = defaultdict(set)
    for (lanes1, curve1), (lanes2, curve2) in combinations(curves.items(), 2):
        if lanes1[0] == lanes2[0]:
            diverge[lanes1[0]].update((curve1, curve2))
        if lanes1[1] == lanes2[1]:
            merge[lanes1[1]].update((curve1, curve2))
        _add_crossing_conflict_points(points, curve1, curve2)

    _add_diverge_merge_conflict_points(points, diverge, merge)

    if len(points) > 1:
        rtree = Rtree((id_, p.point.bounding_rect, None)
                      for id_, p in points.items())
        _merge_points(points, rtree)
        _fill_neighbors(points, rtree)

    # This makes the conflict point positions relative to the node
    for point in points.values():
        point.point = point.point - node.position


def _add_crossing_conflict_points(points: Dict[int, ConflictPoint],
                                  curve1: Curve, curve2: Curve):
    """Create crossing conflict points points between given curves."""
    try:
        intersection = curve1.intersect(curve2)[:, 0]
        if any(numpy.isnan(intersection)):
            return

        # Intersections close to the endpoints are diverging or merging.
        distance = (0.5 - abs(intersection[0] - 0.5)) * curve1.length
        if distance < LANE_WIDTH / 3:
            return

        point = ConflictPoint(curve1.evaluate(intersection[0]),
                              ConflictPointType.CROSSING)
        curve1.add_conflict_point(intersection[0], point)
        curve2.add_conflict_point(intersection[1], point)
        points[id(point)] = point
    except IndexError:
        pass


def _add_diverge_merge_conflict_points(points: Dict[int, ConflictPoint],
                                       diverge: Dict[LaneRef, Set[Curve]],
                                       merge: Dict[LaneRef, Set[Curve]]):
    """Create diverge and merge points from given dicts."""
    for collection, type_, param in zip(
            (diverge, merge),
            (ConflictPointType.DIVERGE, ConflictPointType.MERGE),
            (0.0, 1.0)):
        for curves_ in collection.values():
            point = ConflictPoint(next(iter(curves_)).evaluate(param), type_)
            points[id(point)] = point
            for curve in curves_:
                curve.add_conflict_point(param, point)


def _merge_points(points: Dict[int, ConflictPoint], rtree: Rtree):
    """Merge conflict points closer than MERGE_RADIUS."""
    curves = set()
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
                curves.add(curve)
                curve.replace_conflict_point(other, point)
            merged.add(other_id)
            rtree.delete(other_id, other.point.bounding_rect)
    for id_ in merged:
        del points[id_]

    for curve in curves:
        curve.remove_conflict_point_duplicates()


def _fill_neighbors(points: Dict[int, ConflictPoint], rtree: Rtree):
    """Add conflict points closer than NEIGHBOR_RADIUS as neighbors."""
    for id_, point in points.items():
        for other_id in rtree.intersection(
                point.point.enclosing_rect(NEIGHBOR_RADIUS)):
            if other_id == id_:
                continue
            other = points[other_id]
            distance_squared = point.point.distance_squared(other.point)
            if distance_squared <= NEIGHBOR_RADIUS_SQUARED:
                point.neighbors.add(other)
