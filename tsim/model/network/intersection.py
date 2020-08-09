"""Intersection class and related functions."""

from __future__ import annotations

import logging as log
from collections import defaultdict, deque
from dataclasses import dataclass
from enum import Enum
from itertools import chain, combinations, count, dropwhile, islice, product
from math import pi as PI
from statistics import median_low
from typing import (TYPE_CHECKING, Callable, DefaultDict, Deque, Dict,
                    Iterable, Iterator, List, Optional, Set, Tuple)

import bezier
import numpy
from dataslots import with_slots
from rtree.index import Rtree

import tsim.model.index as Index
from tsim.model.entity import EntityRef
from tsim.model.geometry import Point, Vector, angle, line_intersection_safe
from tsim.model.network.lane import LANE_WIDTH, Lane, LaneRef
from tsim.model.network.orientedway import OrientedWayPosition
from tsim.model.network.location import (NetworkLocation, NetworkPosition,
                                         WorldAndSegmentPosition)
from tsim.model.network.traffic import (Traffic, TrafficAgent,
                                        TrafficDynamicAgent, TrafficLock)
from tsim.model.network.way import OrientedWay
from tsim.utils.linkedlist import LinkedList
from tsim.utils.maptovalue import MapToValue

if TYPE_CHECKING:
    from tsim.model.network.node import Node

LaneConnection = Tuple[LaneRef, LaneRef]
LaneConnections = Dict[LaneRef, Set[LaneRef]]
WayConnections = Dict[OrientedWay, Set[OrientedWay]]

MERGE_RADIUS = 0.1
NEIGHBOR_RADIUS = 1.8
NEIGHBOR_RADIUS_SQUARED = NEIGHBOR_RADIUS ** 2
MAP_TO_ZERO = MapToValue(0.0)


class Intersection:
    """Intersection information for a node.

    Contains way and lane connections with their curves and conflict points.

    Attributes:
        node_ref: Reference to the node.
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

        for curve in self.curves.values():
            curve.init_traffic()

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


class ConflictPoint(TrafficLock):
    """Conflict point in an intersection.

    The point where two lane connections cross. The `id` is unique only among
    conflict points in the same intersection.
    """

    id: int
    point: Point
    type: ConflictPointType
    neighbors: Set[ConflictPoint]
    curves: Set[Curve]
    lock_order: Dict[Curve, Tuple[ConflictPoint, ...]]
    owner: Optional[TrafficAgent]
    owner_location: Optional[NetworkLocation]
    terminal: bool
    queue: LinkedList[Tuple[TrafficAgent, bool]]

    def __init__(self, id_: int, point: Point, type_: ConflictPointType):
        self.id = id_
        self.point = point
        self.type = type_
        self.neighbors = set()
        self.curves = set()
        self.lock_order = ()
        self.owner = None
        self.terminal = False
        self.queue = LinkedList()

    @property
    def active(self) -> bool:
        """Get whether agent is active in the simulation.

        Always false for conflict points, so that inactivating agents stopped
        in traffic works correctly.
        """
        return False

    @property
    def lock_queue(self) -> Iterable[TrafficLock]:
        """Get owned lock queue, always empty for conflict points."""
        return ()

    @property
    def speed(self) -> float:
        """Get agent speed, always zero for conflict points."""
        return MAP_TO_ZERO

    def get_network_position(self, location: Curve, buffer: int) -> float:
        """Get network position of the conflict point in given `location`."""
        if not hasattr(location, 'positions'):
            return None
        return location.positions.get(self, None)

    def is_at(self, location: NetworkLocation) -> bool:
        """Get whether conflict point is at given `location`."""
        return hasattr(location, 'positions') and self in location.positions

    def create_lock_order(self):
        """Create the `lock_order` list after `neighbors` is filled."""
        self.lock_order = {
            c: tuple(sorted(
                set(chain.from_iterable(
                    chain([p[1]], p[1].neighbors) for p in
                    dropwhile(lambda q: q[1] is not self, c.conflict_points))),
                key=lambda p: p.id))
            for c in self.curves
        }
        # Add lock order that is not related to a curve, but only to this
        # conflict point. By locking each conflict point separately, the cars
        # can move to each individual conflict point without locking all the
        # others on the curve. This raises the chance of deadlocks
        # drastically.
        self.lock_order[None] = tuple(sorted(chain(self.neighbors, [self]),
                                             key=lambda cp: cp.id))

    def add_follower(self, agent: TrafficDynamicAgent, buffer: int):
        """Register agent as follower."""
        distance = agent.distance_to_lead(buffer)
        lock_distance = 10  # TODO: Remove hard-coded distance here.
        if distance <= lock_distance:
            agent.start_locking_lead(buffer)
        else:
            agent.distance_to_lock = distance - lock_distance

    def update_followers(self, buffer: int):
        """Update lead of registered followers."""

    def remove_follower(self, agent: TrafficDynamicAgent, buffer: int):
        """Unregister agent as follower."""
        agent.distance_to_lock = None
        # Here release was probably already called, but is called just in case
        # some agent got in the way before `agent` could reach the lock.
        self.release(agent, buffer)

    def notify(self, buffer: int):
        """Notify this agent of lead events."""

    def notify_followers(self, buffer: int):
        """Notify followers of events."""

    def lock(self, agent: TrafficDynamicAgent, buffer: int,
             terminal: bool = False, location: NetworkLocation = None):
        """Lock this traffic lock to `agent`.

        If the lock is available, start the lock process immediately. If
        terminal this lock is acquired, otherwise start acquiring the
        neighbors. If the lock is not available, the `(agent, terminal)` tuple
        is added to the queue.
        """
        # If not terminal, assume agent already owns the lock, because he had
        # to acquire all terminals before locking with terminal=False.
        already_owned = (not terminal
                         or (self in agent.lock_count
                             and agent.lock_count[self] > 0))
        if already_owned:
            self.queue.appendleft((agent, terminal, location))
            self._dequeue(buffer)
            return

        if self.owner is not None and self.owner.waiting[1] is not None:
            waiting = self.owner.waiting
            if waiting[1][waiting[0]].owner == agent:
                log.debug('[%s] Deadlock contingcy for %s',
                          __name__, repr(agent))
                agent.release_all_locks(buffer)
                agent.update_lead(buffer)
                return

        self.queue.append((agent, terminal, location))
        if self.owner is None:
            self._dequeue(buffer)

    def release(self, agent: TrafficDynamicAgent, buffer: int,
                terminal: bool = False):
        """Release this lock.

        If `terminal` is false, will release all neighbors in order, calling
        recursively with `terminal=True`.
        """
        if self.owner is not agent:
            return

        if terminal:
            lock_count = agent.lock_count[self]
            if not lock_count > 0:
                Index.INSTANCE.simulation.raise_event('focus', agent)
                log.error('Releasing %d, lock_count <= 0', self.id)
                return
            if lock_count == 1:
                del agent.lock_count[self]
                self.owner = None
                self._dequeue(buffer)
            else:
                agent.lock_count[self] -= 1
        else:
            try:
                for lock in self.lock_order[self.owner_location]:
                    lock.release(agent, buffer, True)
                # Passing the agent here directly instead of enqueueing to
                # avoid problems with lead updates before _pass is called.
                self._pass(agent, buffer)
            except KeyError as error:
                Index.INSTANCE.simulation.raise_event('focus', agent)
                log.exception(error)

    def drop(self, agent: TrafficAgent):
        """Remove agent from queue without locking."""
        agent_node = next((n for n in self.queue.iter_nodes()
                           if n.data[0] is agent),
                          None)
        if agent_node is not None:
            agent_node.remove()

    def _pass(self, agent: TrafficAgent, buffer: int):
        node = agent.traffic_node.next
        if node.data is self:
            agent.traffic_node.remove()
            agent.traffic_node = node.insert_after(agent)
        agent.notify_followers(buffer)

    def _dequeue(self, buffer: int):
        """Lock to the next agent in the queue."""
        agent, terminal, location = (self.queue.popleft() if self.queue
                                     else (None, None, None))
        if agent is not None:
            self.owner, self.terminal, self.owner_location = (
                agent, terminal, location)
            agent.acquire(self, buffer, terminal, location)

    def __repr__(self):
        return f'{ConflictPoint.__name__}(id={self.id})'


class Curve(NetworkLocation):
    """The curve of a lane connection.

    Contains the bezier curve connecting one lane to the other and the conflict
    points that intersect with this curve. The bezier curves are in world
    coordinates, while the conflict points are in coordinates relative to the
    node position.
    """

    __slots__ = ('node_ref', 'source', 'dest', 'curve', 'length', 'traffic',
                 'positions', '_conflict_points', '_sorted')

    node_ref: EntityRef[Node]
    source: Lane
    dest: Lane
    curve: bezier.Curve
    length: float
    traffic: Traffic
    positions: Dict[TrafficAgent, float]
    _conflict_points: List[Tuple[float, ConflictPoint]]
    _sorted: bool

    def __init__(self, node_ref: EntityRef[Node], lanes: LaneConnection,
                 curve: bezier.Curve):
        self.node_ref = node_ref
        self.source, self.dest = (lane_ref() for lane_ref in lanes)
        self.curve = curve
        self.length = curve.length
        self.traffic = LinkedList()
        self.positions = {}
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
        self.positions[point] = param * self.length
        point.curves.add(self)

    def replace_conflict_point(self, old: ConflictPoint, new: ConflictPoint):
        """Replace conflict point, for use when merging conflict points."""
        index, param = None, None
        for i, (param_, point) in enumerate(self._conflict_points):
            if point is old:
                index, param = i, param_
                break
        if index is not None:
            self._conflict_points[index] = (param, new)
            self.positions[new] = param * self.length
            del self.positions[old]
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
            selected = median_low(params)
            params.remove(selected)
            self.positions[cpoint] = selected * self.length
            for param in params:
                self._conflict_points.remove((param, cpoint))

    def remove_redundant_conflict_point(self):
        """Remove points that are neighbors of a point in the same curve.

        The conflict points in a curve are always locked in order, so if a
        conflict point has a neighbor that is in the same curve, one of them
        will always be locked first and will lock the other as a consequence,
        so this second one must be removed from the curve conflict points.
        Elsewhere, after doing this to all curves, the conflict points that
        don't exist in any curve anymore must be removed from the neighborhood
        of all conflict points, because it should not exist.
        """
        to_remove = set()
        for i, (_, cpoint) in enumerate(self.conflict_points):
            if i in to_remove:
                continue
            for j, (_, cpoint_) in islice(enumerate(self.conflict_points),
                                          i + 1, None):
                if cpoint_ in cpoint.neighbors:
                    to_remove.add(j)
                else:
                    break
        for i in sorted(to_remove, reverse=True):
            del self._conflict_points[i]

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
        return Point(*chain.from_iterable(
            curve.evaluate(position / self.length)))

    def init_traffic(self):
        """Initialize traffic containing only the conflict points."""
        self.traffic = LinkedList(p[1] for p in self.conflict_points)

    def __repr__(self):
        return f'{Curve.__name__}(source={self.source}, dest={self.dest})'


@with_slots
@dataclass(frozen=True)
class CurvePosition(NetworkPosition):
    """A position in a `Curve`.

    The position is in meters from the start of the curve.
    """

    curve: Curve
    position: float

    @property
    def location(self) -> NetworkLocation:
        """Get the `NetworkLocation` of this curve position."""
        return self.curve

    def world_and_segment_position(self) -> WorldAndSegmentPosition:
        """Get world and segment position at this curve position.

        Curves don't have segments, so the segment is always `0` and the
        segment's end is the length of the curve.
        """
        world = self.curve.world_position(self.position)
        return WorldAndSegmentPosition(*world, 0, self.curve.length)


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
        return int(abs(PI - angle_a) < abs(PI - angle_b))

    def rotate_angles():
        angles[0] = 2 * PI
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
        outwards = angles[index] > PI * 2.0 / 3.0
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
            point_vector_1 = (points[lane1],
                              vectors[lane1.oriented_way.flipped()])
            point_vector_2 = (points[lane2], vectors[lane2.oriented_way])
            crossing = line_intersection_safe(*point_vector_1, *point_vector_2)
            closest = min((point_vector_1, point_vector_2),
                          key=lambda pv, c=crossing: pv[0].distance(c))
            if closest[1].dot_product(crossing - closest[0]) >= 0.0:
                crossing = closest[0] - closest[1]
            crossings[(lane1, lane2)] = crossing

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
    id_generator = count()
    points: Dict[int, ConflictPoint] = {}
    diverge = defaultdict(set)
    merge = defaultdict(set)
    for (lanes1, curve1), (lanes2, curve2) in combinations(curves.items(), 2):
        if lanes1[0] == lanes2[0]:
            diverge[lanes1[0]].update((curve1, curve2))
        if lanes1[1] == lanes2[1]:
            merge[lanes1[1]].update((curve1, curve2))
        _add_crossing_conflict_points(id_generator, points, curve1, curve2)

    _add_diverge_merge_conflict_points(id_generator, points, diverge, merge)

    if len(points) > 1:
        rtree = Rtree((id_, p.point.bounding_rect, None)
                      for id_, p in points.items())
        _merge_points(points, rtree)
        _fill_neighbors(points, rtree)

    for point in points.values():
        point.create_lock_order()
        # This makes the conflict point positions relative to the node.
        point.point = point.point - node.position

    for curve in curves.values():
        curve.remove_redundant_conflict_point()

    # Remove from neighbors points that aren't in any curve
    points = {p for _, p in chain.from_iterable(c.conflict_points
                                                for c in curves.values())}
    for point in points:
        for neighbor in list(point.neighbors):
            if neighbor not in points:
                point.neighbors.remove(neighbor)


def _add_crossing_conflict_points(id_generator: Iterator[int],
                                  points: Dict[int, ConflictPoint],
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

        point = ConflictPoint(next(id_generator),
                              curve1.evaluate(intersection[0]),
                              ConflictPointType.CROSSING)
        curve1.add_conflict_point(intersection[0], point)
        curve2.add_conflict_point(intersection[1], point)
        points[id(point)] = point
    except IndexError:
        pass


def _add_diverge_merge_conflict_points(id_generator: Iterator[int],
                                       points: Dict[int, ConflictPoint],
                                       diverge: Dict[LaneRef, Set[Curve]],
                                       merge: Dict[LaneRef, Set[Curve]]):
    """Create diverge and merge points from given dicts."""
    for collection, type_, param in zip(
            (diverge, merge),
            (ConflictPointType.DIVERGE, ConflictPointType.MERGE),
            (0.0, 1.0)):
        for curves_ in collection.values():
            point = ConflictPoint(next(id_generator),
                                  next(iter(curves_)).evaluate(param), type_)
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


def _fill_neighbors(points: Dict[int, ConflictPoint], rtree: Rtree,
                    skip_in_same_curve: bool = True):
    """Add conflict points closer than NEIGHBOR_RADIUS as neighbors."""
    for id_, point in points.items():
        for other_id in rtree.intersection(
                point.point.enclosing_rect(NEIGHBOR_RADIUS)):
            if other_id == id_:
                continue
            other = points[other_id]
            if skip_in_same_curve and (point.curves & other.curves):
                continue
            distance_squared = point.point.distance_squared(other.point)
            if distance_squared <= NEIGHBOR_RADIUS_SQUARED:
                point.neighbors.add(other)
