"""Network serialization configuration."""

from __future__ import annotations

from collections import deque
from functools import partialmethod
from itertools import chain

from tsim.model.entity import EntityRef
from tsim.model.network.intersection import ConflictPoint, Curve, Intersection
from tsim.model.network.lane import Lane, LaneSegment
from tsim.utils import pickling
from tsim.utils.linkedlist import LinkedList


def configure():
    """Configure serialization of network related classes."""
    Intersection.__getstate__ = _intersection_getstate
    Intersection.__setstate__ = _intersection_setstate
    ConflictPoint.__getstate__ = _conflict_point_getstate
    ConflictPoint.__setstate__ = _conflict_point_setstate
    Curve.__getstate__ = _curve_getstate
    Curve.__setstate__ = _curve_setstate
    Lane.__getstate__ = _lane_getstate
    Lane.__setstate__ = _lane_setstate
    LaneSegment.__getstate__ = partialmethod(pickling.getstate, add_dict=False)
    LaneSegment.__setstate__ = partialmethod(pickling.setstate, add_dict=False)


# pylint: disable=protected-access


def _intersection_getstate(self: Intersection):
    points = {p.id: p for _, p in
              chain.from_iterable(c.conflict_points
                                  for c in self.curves.values())}
    neighbors = {k: {p.id for p in v.neighbors}
                 for k, v in points.items()}
    return (self.node_ref.id, self.lane_connections, self.way_connections,
            self.connection_map, self.curves, points, neighbors)


def _intersection_setstate(self: Intersection, state):
    (node_id, self.lane_connections, self.way_connections,
     self.connection_map, self.curves, points, neighbors) = state
    self.node_ref = EntityRef(node_id)
    for id_, neighbor_ids in neighbors.items():
        points[id_].neighbors.update(points[i] for i in neighbor_ids)
    for point in points.values():
        point.create_lock_order()


def _conflict_point_getstate(self: ConflictPoint):
    return self.id, self.point, self.type


def _conflict_point_setstate(self: ConflictPoint, state):
    self.id, self.point, self.type = state
    self.neighbors = set()
    self.curves = set()
    self.owner = None
    self.owner_secondary = None
    self.acquiring = None
    self.queue = deque()
    self.waiting = 0


def _curve_getstate(self: Curve):
    return (self.node_ref.id, self.source, self.dest, self.curve,
            self.length, self._conflict_points, self._sorted)


def _curve_setstate(self: Curve, state):
    (node_id, self.source, self.dest, self.curve, self.length,
     self._conflict_points, self._sorted) = state
    self.node_ref = EntityRef(node_id)
    self.positions = {}
    for param, point in self._conflict_points:
        point.curves.add(self)
        self.positions[point] = param * self.length
    self.init_traffic()


def _lane_getstate(self: Lane):
    return (self.lane_ref, self.distance_from_center, self.length,
            self.segments)


def _lane_setstate(self: Lane, state):
    (self.lane_ref, self.distance_from_center,
     self.length, self.segments) = state
    self.traffic = LinkedList()
