"""Unit tests for path finding."""
import unittest
from itertools import product

from tsim.core.geometry import Point
from tsim.core.index import INSTANCE as INDEX
from tsim.core.network.node import Node
from tsim.core.network.path import dijkstra
from tsim.core.network.way import Endpoint, Way

INDEX.name = 'test'


class PathTest(unittest.TestCase):
    """Unit tests for path finding."""

    def test_dijkstra(self):
        """Test the dijkstra algorithm implementation.

        Generate a size x size grid of ways and find use dijkstra on the first
        oriented way. Check that the first oriented way points to None and the
        others all reach the first node with a valid path.
        """
        size = 3
        position = list(product(range(0, 100 * size, 100), repeat=2))
        nodes = [Node(Point(*p)) for p in position]
        ways = []
        for i, node in enumerate(nodes):
            if i % size != size - 1:
                ways.append(Way(node, nodes[i + 1], (1, 1)))
            if i < len(nodes) - size:
                ways.append(Way(node, nodes[i + size], (1, 1)))
        oriented_ways = [w.oriented() for w in ways]
        oriented_ways.extend(w.oriented(Endpoint.END) for w in ways
                             if w.lane_count[1] > 0)
        single_source = dijkstra(oriented_ways[0])
        for way, (_, depth) in single_source.items():
            depth += 1
            way_ = way
            while way_ is not oriented_ways[0]:
                way_, depth_ = single_source[way_]
                self.assertEqual(depth_, depth - 1)
                depth = depth_
            self.assertEqual(depth, 1)
            way_, depth_ = single_source[way_]
            self.assertEqual(depth_, 0)
            self.assertIsNone(way_)
