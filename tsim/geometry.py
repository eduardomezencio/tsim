"""Geometry related classes and functions for tsim."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Tuple

from dataslots import with_slots


BoundingRect = Tuple[float, float, float, float]


def distance(point_a: Point, point_b: Point) -> float:
    """Euclidean distance between two points."""
    return ((point_b.x - point_a.x) ** 2 + (point_b.y - point_a.y) ** 2) ** 0.5


def distance_squared(point_a: Point, point_b: Point) -> float:
    """Square of the euclidean distance between two points."""
    return (point_b.x - point_a.x) ** 2 + (point_b.y - point_a.y) ** 2


def manhattan_distance(point_a: Point, point_b: Point) -> float:
    """Manhattan distance between two points."""
    return abs(point_b.x - point_a.x + point_b.y - point_a.y)


@with_slots
@dataclass(frozen=False)
class Point:
    """A point in space."""

    x: float
    y: float

    def calc_bounding_rect(self,
                           accumulated: BoundingRect = None) -> BoundingRect:
        """Calculate bounding rectangle.

        If accumulated is None (default), it just turns the point coordinates
        into a bounding box tuple. If accumulated is a bounding box, it extends
        the bounding box to contain this point.
        """
        if accumulated is None:
            return (self.x, self.y, self.x, self.y)
        return (min(self.x, accumulated[0]), min(self.y, accumulated[1]),
                max(self.x, accumulated[2]), max(self.y, accumulated[3]))

    bounding_rect = property(calc_bounding_rect)
    distance = distance
    distance_squared = distance_squared
