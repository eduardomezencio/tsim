#!/usr/bin/env python3
"""Reader for custom network format.

This format is a simple text file with this format:

network* -> 'p' (point_entry)* 'n' (node_entry)* 'w' (way_entry)*
point_entry -> index point
node_entry -> index node
way_entry -> index way
point ->  coord coord
node -> index
way -> index index (index)*
index -> integer
coord -> float
"""

from copy import copy
import sys

from tsim.model.entity import Entity, EntityIndex
from tsim.model.geometry import Point
from tsim.model.network import Node, Way


def main():
    """Custom reader main function."""
    data = []
    index = EntityIndex(sys.argv[1])
    token = get_tokens(sys.argv[1])
    assert next(token) == 'p'
    while next(token) != 'n':
        data.append(Point(float(next(token)), float(next(token))))
    while next(token) != 'w':
        data.append(Node(copy(data[int(next(token))])))
    try:
        last = int(next(token))
        while True:
            start = data[int(next(token))]
            end = data[int(next(token))]
            points = []
            i = int(next(token))
            while i != (last + 1):
                points.append(copy(data[i]))
                i = int(next(token))
            last = i
            data.append(Way(start, end, tuple(points)))
    except StopIteration:
        data.append(Way(start, end, tuple(points)))

    for item in data:
        if isinstance(item, Entity):
            index.add(item)
    index.save()


def get_tokens(filename):
    """Generator of tokens from file."""
    with open(filename, 'r') as file:
        for line in file:
            yield from line.split()


if __name__ == '__main__':
    main()
