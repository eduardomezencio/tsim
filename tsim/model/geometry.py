"""Geometry related classes and functions for tsim."""

from __future__ import annotations

from dataclasses import dataclass
from math import sqrt
from typing import Tuple

from dataslots import with_slots


BoundingRect = Tuple[float, float, float, float]


def distance(point_a: Point, point_b: Point) -> float:
    """Euclidean distance between two points."""
    return sqrt((point_b.x - point_a.x) ** 2 + (point_b.y - point_a.y) ** 2)


def distance3(point_a: Point3, point_b: Point3) -> float:
    """Euclidean distance between two points."""
    return sqrt((point_b.x - point_a.x) ** 2 + (point_b.y - point_a.y)
                + (point_b.z - point_a.z) ** 2)


def distance_squared(point_a: Point, point_b: Point) -> float:
    """Square of the euclidean distance between two points."""
    return (point_b.x - point_a.x) ** 2 + (point_b.y - point_a.y) ** 2


def distance_squared3(point_a: Point3, point_b: Point3) -> float:
    """Square of the euclidean distance between two points."""
    return ((point_b.x - point_a.x) ** 2 + (point_b.y - point_a.y) ** 2
            + (point_b.z - point_a.z) ** 2)


def manhattan_distance(point_a: Point, point_b: Point) -> float:
    """Manhattan distance between two points."""
    return abs(point_b.x - point_a.x + point_b.y - point_a.y)


def manhattan_distance3(point_a: Point3, point_b: Point3) -> float:
    """Manhattan distance between two points."""
    return abs(point_b.x - point_a.x + point_b.y - point_a.y
               + point_b.z - point_a.z)


def sec(vector_a: Vector, vector_b: Vector) -> float:
    """Get secant of angle between vectors."""
    return ((vector_a.norm() * vector_b.norm())
            / (vector_a.dot_product(vector_b)))


sec3 = sec  # pylint: disable=invalid-name


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
        return sqrt(self.x ** 2.0 + self.y ** 2.0)

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


@with_slots
@dataclass
class Vector3(Vector):
    """A vector in 3d space, that doubles as a point."""

    z: float

    def norm(self) -> float:
        """Calculate norm of the vector."""
        return sqrt(self.x ** 2.0 + self.y ** 2.0 + self.z ** 2.0)

    def normalized(self) -> Vector3:
        """Get this vector normalized."""
        norm = self.norm()
        return Vector3(self.x / norm, self.y / norm, self.z / norm)

    def add(self, other: Vector3) -> Vector3:
        """Sum of this vector and another."""
        return Vector3(self.x + other.x, self.y + other.y, self.z + other.z)

    def subtract(self, other: Vector3) -> Vector3:
        """Difference of this vector by another."""
        return Vector3(self.x - other.x, self.y - other.y, self.z - other.z)

    def multiply(self, scalar: float) -> Vector3:
        """Multiplication by scalar."""
        return Vector3(self.x * scalar, self.y * scalar, self.z * scalar)

    def dot_product(self, other: Vector3) -> float:
        """Dot product of this vector by another."""
        return self.x * other.x + self.y * other.y + self.z * other.z

    def rotated_right(self) -> Vector3:
        """Get vector rotated clockwise by 90 degrees."""
        raise NotImplementedError()

    distance = distance3
    distance_squared = distance_squared3
    sec = sec3

    __abs__ = norm
    __add__ = add
    __mul__ = multiply
    __rmul__ = multiply
    __sub__ = subtract


Point3 = Vector3
