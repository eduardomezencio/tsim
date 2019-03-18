"""Unit test for geometry."""
import unittest

from tsim.model.geometry import line_intersection, midpoint, Point, Vector


class GeometryTest(unittest.TestCase):
    """Unit test for geometry."""

    def test_midpoint(self):
        """Unit test for midpoint."""
        tests = (((-1, 2), (1, -2), (0, 0)),
                 ((1, 1), (1, 1), (1, 1)))
        for test in tests:
            result = midpoint(Point(*test[0]), Point(*test[1]))
            self.assertAlmostEqual(result, Point(*test[2]))

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
