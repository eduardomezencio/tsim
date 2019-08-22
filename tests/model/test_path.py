"""Unit tests for path finding."""
from itertools import product
import unittest

from tsim.model.geometry import Point
from tsim.model.index import INSTANCE as INDEX
from tsim.model.network import Node, Way
from tsim.model.path import dijkstra

INDEX.name = 'test'


class PathTest(unittest.TestCase):
    """Unit tests for path finding."""

    def test_dijkstra(self):
        """Test the dijkstra algorithm implementation.

        Generate a size x size grid of ways and find use dijkstra on the first
        node. Check that the first node points to None and the others all reach
        the first node with a valid path.
        """
        size = 3
        position = [p for p in product(range(0, 100 * size, 100), repeat=2)]
        nodes = [Node(Point(*p)) for p in position]
        ways = []
        for i, node in enumerate(nodes):
            if i % size != size - 1:
                ways.append(Way(node, nodes[i + 1], (1, 1)))
            if i < len(nodes) - size:
                ways.append(Way(node, nodes[i + size], (1, 1)))
        paths = dijkstra(nodes[0])
        for node, way in paths.items():
            node_, way_ = node, way
            while node_ is not nodes[0]:
                self.assertIs(node_, way_.end)
                node_ = way_.start
                way_ = paths[node_]
            self.assertIsNone(way_)
        self.assertTrue(paths)
