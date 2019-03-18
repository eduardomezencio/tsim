"""Unit test for geometry."""
from math import sqrt
import unittest

from tsim.model.geometry import line_intersection, midpoint, Point, Vector


class GeometryTest(unittest.TestCase):
    """Unit test for geometry."""

    def test_line_intersection(self):
        """Unit test for line_intersection."""
        tests = (((0, 0), (3, 2), (1, 5), (-3, -4), (-11/2, -11/3)),
                 ((-2, 0), (1, 0), (5, 5), (0, 1), (5, 0)),
                 ((-1, -1), (0, 3), (0, 0), (2, 1), (-1, -1/2)),
                 ((-1, -3), (0, 5), (0, 1), (2, 0), (-1, 1)))
        for test in tests:
            result = line_intersection(Point(*test[0]), Vector(*test[1]),
                                       Point(*test[2]), Vector(*test[3]))
            self.assertAlmostEqual(result, Point(*test[4]))
        tests = (((0, 0), (1, 2), (1, 0), (1, 2)),
                 ((-1, 0), (0, 2), (0, 0), (0, -1)),
                 ((-1, 0), (1, 0), (0, 5), (5, 0)))
        for test in tests:
            with self.assertRaises(ZeroDivisionError):
                line_intersection(Point(*test[0]), Vector(*test[1]),
                                  Point(*test[2]), Vector(*test[3]))

    def test_midpoint(self):
        """Unit test for midpoint."""
        tests = (((-1, 2), (1, -2), (0, 0)),
                 ((1, 1), (1, 1), (1, 1)),
                 ((-4, 2), (1, 1), (-3/2, 3/2)))
        for test in tests:
            result = midpoint(Point(*test[0]), Point(*test[1]))
            self.assertAlmostEqual(result, Point(*test[2]))

    def test_projection(self):
        """Unit test for Vector.projection_on."""
        tests = (((3, -8), (1, 2), (-13/5, -26/5)),
                 ((-1, 5), (2, 4), (9/5, 18/5)))
        for test in tests:
            result = Point(*test[0]).projection_on(Point(*test[1]))
            self.assertAlmostEqual(result, Point(*test[2]))

    def test_scalar_projection(self):
        """Unit test for Vector.scalar_projection_on."""
        tests = (((3, -8), (1, 2), (-13 * sqrt(1/5))),
                 ((-1, 5), (2, 4), (9 * sqrt(1/5))))
        for test in tests:
            result = Point(*test[0]).scalar_projection_on(Point(*test[1]))
            self.assertAlmostEqual(result, test[2])
