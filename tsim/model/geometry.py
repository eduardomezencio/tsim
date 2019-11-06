"""Geometry related classes and functions for tsim."""

from __future__ import annotations

from dataclasses import dataclass
from functools import partialmethod
from math import acos, copysign, cos, pi, sin, sqrt
from typing import Iterable, Sequence, Tuple

from dataslots import with_slots

from tsim.utils import pickling
from tsim.utils.iterators import window_iter

BoundingRect = Tuple[float, float, float, float]


def distance(point_a: Point, point_b: Point) -> float:
    """Euclidean distance between two points."""
    return sqrt((point_b.x - point_a.x) ** 2 + (point_b.y - point_a.y) ** 2)


def distance_squared(point_a: Point, point_b: Point) -> float:
    """Square of the euclidean distance between two points."""
    return (point_b.x - point_a.x) ** 2 + (point_b.y - point_a.y) ** 2


def manhattan_distance(point_a: Point, point_b: Point) -> float:
    """Manhattan distance between two points."""
    return abs(point_b.x - point_a.x + point_b.y - point_a.y)


def midpoint(point_a: Point, point_b: Point) -> Point:
    """Get the midpoint of two points."""
    return point_a + (point_b - point_a) * 0.5


def angle(vector_a: Vector, vector_b: Vector) -> float:
    """Get angle between vectors, counterclockwise from a to b."""
    a_norm, b_norm = vector_a.normalized(), vector_b.normalized()
    if a_norm.rotated_right().dot_product(b_norm) > 0.0:
        return 2 * pi - acos(max(-1.0, min(1.0, a_norm.dot_product(b_norm))))
    return acos(max(-1.0, min(1.0, a_norm.dot_product(b_norm))))


def sec(vector_a: Vector, vector_b: Vector) -> float:
    """Get secant of angle between vectors."""
    return ((vector_a.norm() * vector_b.norm())
            / (vector_a.dot_product(vector_b)))


def line_intersection(point1: Point, vector1: Vector,
                      point2: Point, vector2: Vector) -> Point:
    """Calculate the intersection of two lines.

    Raises ZeroDivisionError if parallel.
    """
    if vector1.x != 0.0:
        slope1 = vector1.y / vector1.x
        if vector2.x != 0.0:
            slope2 = vector2.y / vector2.x
            x = ((-slope2 * point2.x + point2.y + slope1 * point1.x - point1.y)
                 / (slope1 - slope2))
        else:
            x = point2.x
        y = slope1 * (x - point1.x) + point1.y
    else:
        if vector2.y != 0.0:
            slope2 = vector2.x / vector2.y
            y = (slope2 * point2.y - point2.x + point1.x) / slope2
        else:
            y = point2.y
        x = point1.x
    return Point(x, y)


def line_intersection_safe(point1: Point, normal_vector1: Vector,
                           point2: Point, normal_vector2: Vector,
                           threshold: float = 0.9) -> Point:
    """Calculate the intersection of two lines with no exceptions.

    Vectors passed as arguments must be normalized already to help checking for
    parallelism. If the two lines are parallel or almost parallel, returns the
    midpoint of given points instead. Lines are almost parallel if the absolute
    value if the dot product of the vectors is above the threshold.
    """
    if abs(normal_vector1.dot_product(normal_vector2)) >= threshold:
        return midpoint(point1, point2)
    return line_intersection(point1, normal_vector1, point2, normal_vector2)


def calc_bounding_rect(points: Iterable[Point],
                       accumulated: BoundingRect = None) -> BoundingRect:
    """Calculate the bounding rectangle of given points."""
    for point in points:
        accumulated = point.calc_bounding_rect(accumulated)
    return accumulated


def bounding_rect_center(rect: BoundingRect) -> Point:
    """Calculate the center of a bounding rectangle."""
    return Point((rect[0] + rect[2]) / 2, (rect[1] + rect[3]) / 2)


def point_in_polygon(point: Point, polygon: Polygon) -> bool:
    """Calculate if point is inside the given polygon."""
    crossings = 0
    for point1, point2 in window_iter(polygon, extend=True):
        if point1.x >= point.x and point2.x >= point.x:
            continue
        if point1.y > point2.y:
            point1, point2 = point2, point1
        if point1.y <= point.y < point2.y:
            if point1.x < point.x and point2.x < point.x:
                crossings += 1
                continue
            vector = point2 - point1
            vector_x = vector.x * ((point.y - point1.y) / vector.y)
            if point1.x + vector_x < point.x:
                crossings += 1
    return bool(crossings % 2)


# pylint: disable=invalid-name
midpoint3 = midpoint
sec3 = sec
# pylint: enable=invalid-name


@with_slots
@dataclass(frozen=True)
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

    def enclosing_rect(self, radius: float) -> BoundingRect:
        """Get rectangle with sides at 'radius' distance from point."""
        return (self.x - radius, self.y - radius,
                self.x + radius, self.y + radius)

    def y_flipped(self):
        """Get vector with y coordinate flipped."""
        return Vector(self.x, -self.y)

    def norm(self) -> float:
        """Calculate norm of the vector."""
        return sqrt(self.x ** 2.0 + self.y ** 2.0)

    def norm_squared(self) -> float:
        """Calculate norm of the vector squared."""
        return self.x ** 2.0 + self.y ** 2.0

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

    def close_to(self, other: Vector, threshold: float = 0.001) -> bool:
        """Test for proximity to another point."""
        return (abs(self.x - other.x) < threshold and
                abs(self.y - other.y) < threshold)

    def dot_product(self, other: Vector) -> float:
        """Dot product of this vector by another."""
        return self.x * other.x + self.y * other.y

    def scalar_projection_on(self, other: Vector) -> float:
        """Calculate scalar projection of this vector onto another."""
        return self.dot_product(other.normalized())

    def projection_on(self, other: Vector, clamp: bool = False) -> Vector:
        """Calculate the projection of this vector onto another."""
        norm = other.norm()
        other_hat = Vector(other.x / norm, other.y / norm)
        scalar = self.dot_product(other_hat)
        if clamp:
            scalar = max(0.0, min(scalar, norm))
        return other_hat * scalar

    def reflection_over(self, axis: Vector) -> Vector:
        """Calculate vector reflected over an axis."""
        return self + (self.projection_on(axis) - self) * 2.0

    def projection_reflection(self, axis: Vector) -> Tuple[Vector, Vector]:
        """Calculate both projection and reflection over an axis."""
        projection = self.projection_on(axis)
        return projection, self + (projection - self) * 2.0

    def rotated_left(self) -> Vector:
        """Get vector rotated counterclockwise by 90 degrees."""
        return Vector(-self.y, self.x)

    def rotated_right(self) -> Vector:
        """Get vector rotated clockwise by 90 degrees."""
        return Vector(self.y, -self.x)

    def rotated(self, radians: float):
        """Get vector rotated counterclockwise by given angle."""
        return Vector(self.x * cos(radians) - self.y * sin(radians),
                      self.x * sin(radians) + self.y * cos(radians))

    def sorting_key(self) -> float:
        """Calculate a key for sorting vectors by angle (by direction)."""
        return copysign(1.0 - self.x / (abs(self.x) + abs(self.y)), self.y)

    def distance_to_segment(self, seg_start: Point, seg_vector: Vector,
                            squared: bool = False) -> Tuple[float, Point]:
        """Calculate the smallest distance to line segment.

        Calculate the smallest distance from the point to a line segment
        represented by its starting point and a vector. Returns a tuple with
        the distance and the closest point.
        """
        projection = (self - seg_start).projection_on(seg_vector, clamp=True)
        closest = seg_start + projection
        if squared:
            return self.distance_squared(closest), closest
        return self.distance(closest), closest

    def __neg__(self):
        return Vector(-self.x, -self.y)

    def __iter__(self):
        yield self.x
        yield self.y

    angle = angle
    bounding_rect = property(calc_bounding_rect)
    distance = distance
    distance_squared = distance_squared
    midpoint = midpoint
    sec = sec

    __abs__ = norm
    __add__ = add
    __mul__ = multiply
    __rmul__ = multiply
    __sub__ = subtract

    __getstate__ = partialmethod(pickling.getstate, add_dict=False)
    __setstate__ = partialmethod(pickling.setstate, add_dict=False)


Point = Vector
Polygon = Sequence[Point]
