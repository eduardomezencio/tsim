"""Extra network functions."""

from itertools import chain

from tsim.model.entity import EntityIndex
from tsim.model.network import Node, Way


def dissolve_node(index: EntityIndex, node: Node):
    """Remove a node joining the two ways it connects."""
    two_ways = len(node.starts) + len(node.ends) == 2
    loops = (node.starts and node.ends and
             node.starts[0].value is node.ends[0].value)
    if not two_ways or loops:
        raise ValueError(
            'Can only dissolve nodes connected to exactly two ways.')

    ways = [r.value for r in chain(node.ends, node.starts)]
    assert len(ways) == 2
    start, end = (w.other(node) for w in ways)

    if not node.level == start.level == end.level:
        raise ValueError('Can not dissolve nodes in different levels.')

    if not ways[0].lanes == ways[1].lanes:
        raise ValueError('Can not dissolve nodes with lane changes.')

    waypoints = []
    waypoints.extend(ways[0].waypoints if ways[0].end is node
                     else reversed(ways[0].waypoints))
    waypoints.append(node.position)
    waypoints.extend(ways[1].waypoints if ways[1].start is node
                     else reversed(ways[1].waypoints))

    index.delete(node)

    way = Way(start, end, lanes=ways[0].lanes, waypoints=tuple(waypoints))
    index.add(way)
