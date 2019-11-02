"""Unit test for iterators."""
import unittest
from itertools import islice

from tsim.utils.iterators import drop_duplicates, window_iter


class IteratorsTest(unittest.TestCase):
    """Unit test for iterators."""

    def test_drop_duplicates(self):
        """Unit test for drop_duplicates."""
        list_ = [1, 2, 3, 4, 5]
        self.assertEqual(list(drop_duplicates(list_)), list_)

        list_ = [1, 2, 3, 2]
        self.assertEqual(list(drop_duplicates([1, 2, 2, 3, 2])), list_)
        self.assertEqual(list(drop_duplicates([1, 1, 2, 3, 3, 3, 2])), list_)

    def test_window_iter(self):
        """Unit test for window_iter."""
        iterable = [1, 2, 3, 4, 5]
        self.assertEqual(list(window_iter(iterable)),
                         list(zip(iterable, islice(iterable, 1, None))))

        self.assertEqual(list(window_iter([1, 2, 3, 4], size=3)),
                         [(1, 2, 3), (2, 3, 4)])

        self.assertFalse(list(window_iter([1], size=2)))

        self.assertEqual(list(window_iter([1], size=1)), [(1,)])
