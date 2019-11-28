"""Network serialization configuration."""

from functools import partialmethod
from itertools import chain

from tsim.model.entity import EntityRef
from tsim.model.network.intersection import ConflictPoint, Curve, Intersection
from tsim.model.network.lane import LaneSegment
from tsim.utils import pickling


def configure():
    """Configure serialization of network related classes."""
    Intersection.__getstate__ = _intersection_getstate
    Intersection.__setstate__ = _intersection_setstate
    ConflictPoint.__getstate__ = _conflict_point_getstate
    ConflictPoint.__setstate__ = _conflict_point_setstate
    Curve.__getstate__ = _curve_getstate
    Curve.__setstate__ = _curve_setstate
    LaneSegment.__getstate__ = partialmethod(pickling.getstate, add_dict=False)
    LaneSegment.__setstate__ = partialmethod(pickling.setstate, add_dict=False)


# pylint: disable=protected-access


def _intersection_getstate(self):
    points = {id(p): p for _, p in
              chain.from_iterable(c.conflict_points
                                  for c in self.curves.values())}
    neighbors = {k: {id(p) for p in v.neighbors}
                 for k, v in points.items()}
    return (self.node_ref.id, self.lane_connections, self.way_connections,
            self.connection_map, self.curves, points, neighbors)


def _intersection_setstate(self, state):
    (node_id, self.lane_connections, self.way_connections,
     self.connection_map, self.curves, points, neighbors) = state
    self.node_ref = EntityRef(node_id)
    for id_, neighbor_ids in neighbors.items():
        points[id_].neighbors.update(points[i] for i in neighbor_ids)


def _conflict_point_getstate(self):
    return self.point, self.type


def _conflict_point_setstate(self, state):
    self.point, self.type = state
    self.neighbors = set()
    self.curves = set()


def _curve_getstate(self):
    return (self.node_ref.id, self.source, self.dest, self.curve,
            self.length, self._conflict_points, self._sorted)


def _curve_setstate(self, state):
    (node_id, self.source, self.dest, self.curve, self.length,
     self._conflict_points, self._sorted) = state
    self.node_ref = EntityRef(node_id)
    for _, point in self._conflict_points:
        point.curves.add(self)
