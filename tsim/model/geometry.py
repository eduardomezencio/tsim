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


def sec(vector_a: Vector, vector_b: Vector) -> float:
    """Get secant of angle between vectors."""
    return ((vector_a.norm() * vector_b.norm())
            / (vector_a.dot_product(vector_b)))


@with_slots
@dataclass
class Vector:
    """A vector in space, that doubles as a point."""

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

    def norm(self) -> float:
        """Calculate norm of the vector."""
        return (self.x ** 2 + self.y ** 2) ** 0.5

    def normalized(self) -> Vector:
        """Get this vector normalized."""
        norm = self.norm()
        return Vector(self.x / norm, self.y / norm)

    def add(self, other: Vector) -> Vector:
        """Sum of this vector and another."""
        return Vector(self.x + other.x, self.y + other.y)

    def subtract(self, other: Vector) -> Vector:
        """Difference of this vector by another."""
        return Vector(self.x - other.x, self.y - other.y)

    def multiply(self, scalar: float) -> Vector:
        """Multiplication by scalar."""
        return Vector(self.x * scalar, self.y * scalar)

    def dot_product(self, other: Vector) -> float:
        """Dot product of this vector by another."""
        return self.x * other.x + self.y * other.y

    def rotated_right(self) -> Vector:
        """Get vector rotated clockwise by 90 degrees."""
        return Vector(-self.y, self.x)

    bounding_rect = property(calc_bounding_rect)
    distance = distance
    distance_squared = distance_squared
    sec = sec

    __abs__ = norm
    __add__ = add
    __mul__ = multiply
    __rmul__ = multiply
    __sub__ = subtract


Point = Vector
