#!/usr/bin/env python3
"""Reader for openstreetmap xml files."""

from __future__ import annotations

import logging as log
import sys
from collections import namedtuple
from itertools import islice
from math import asin, cos, radians, sin, sqrt
from textwrap import wrap
from typing import Dict, Tuple
from xml.etree import ElementTree

from tsim.model.geometry import Point
from tsim.model.index import INSTANCE as INDEX
from tsim.model.network.node import Node
from tsim.model.network.way import Way


def main():
    """Call osm_reader for each name in command line arguments."""
    log_config()
    for name in sys.argv[1:]:
        osm_reader(name)


def osm_reader(name):
    """Run the osm reader."""
    INDEX.reset(name)

    tree = ElementTree.parse(name)
    root = tree.getroot()
    try:
        bounds = {k: float(v) for k, v in root.find('bounds').attrib.items()
                  if k in ('maxlat', 'maxlon', 'minlat', 'minlon')}
    except AttributeError:
        bounds = calculate_bounds(root)
    center = ((bounds['maxlat'] - bounds['minlat']) / 2 + bounds['minlat'],
              (bounds['maxlon'] - bounds['minlon']) / 2 + bounds['minlon'])

    nodes = {int(n.get('id')):
             Node(Point(*coord_meters(*center, float(n.get('lat')),
                                      float(n.get('lon')))))
             for n in root.iterfind('node')}
    nodes_inv = {node: osm_id for osm_id, node in nodes.items()}
    for xid, node in nodes.items():
        node.xid = xid

    Highway = namedtuple('Highway', ('nodes', 'level', 'one_way', 'lane_count',
                                     'xid', 'max_speed'))
    highways = [
        Highway(nodes=[int(n.get('ref')) for n in w.iterfind('nd')],
                level=(1 if any((t.get('k') == 'bridge' and
                                 t.get('v') == 'viaduct')
                                for t in w.iterfind('tag'))
                       else 0),
                one_way=any((t.get('k') == 'oneway' and
                             t.get('v') in ('yes', 'true', '1'))
                            for t in w.iterfind('tag')),
                lane_count=next((int(t.get('v')) for t in w.iterfind('tag')
                                 if t.get('k') == 'lanes'),
                                None),
                xid=int(w.get('id')),
                max_speed=next((float(t.get('v').split()[0])
                                for t in w.iterfind('tag')
                                if t.get('k') == 'maxspeed'),
                               None))
        for w in root.iterfind('way')
        if any(t.get('k') == 'highway' and t.get('v') != 'footway'
               for t in w.iterfind('tag'))]

    for highway in highways:
        for start, end in zip(highway.nodes, islice(highway.nodes, 1, None)):
            if highway.level != 0:
                nodes[start].level = nodes[end].level = highway.level
            if highway.one_way:
                lane_count = ((highway.lane_count, 0)
                              if highway.lane_count is not None else (2, 0))
            else:
                lane_count = ((highway.lane_count // 2,
                               highway.lane_count - highway.lane_count // 2)
                              if highway.lane_count is not None else (1, 1))
            way = (Way(nodes[start], nodes[end], lane_count)
                   if highway.max_speed is None
                   else Way(nodes[start], nodes[end], lane_count,
                            max_speed=highway.max_speed))
            way.xid = highway.xid

    dissolve_nodes(nodes_inv)
    filter_waypoints()

    INDEX.save()


def calculate_bounds(root: ElementTree.Element) -> Dict[str, float]:
    """Calculate map bounds from node coordinates."""
    bounds = {'minlon': 180.0, 'minlat': 90.0,
              'maxlon': -180.0, 'maxlat': -90.0}
    for node in root.iterfind('node'):
        for node_key, bounds_key in zip(('lon', 'lat'), ('minlon', 'minlat')):
            if float(node.get(node_key)) < bounds[bounds_key]:
                bounds[bounds_key] = float(node.get(node_key))
        for node_key, bounds_key in zip(('lon', 'lat'), ('maxlon', 'maxlat')):
            if float(node.get(node_key)) > bounds[bounds_key]:
                bounds[bounds_key] = float(node.get(node_key))
    return bounds


def coord_meters(center_lat: float, center_lon: float,
                 lat: float, lon: float) -> Tuple[float, float]:
    """Coordinates in meters relative to coordinates given as center."""
    x = distance_meters(center_lat, center_lon, center_lat, lon)
    if lon < center_lon:
        x = -x
    y = distance_meters(center_lat, center_lon, lat, center_lon)
    if lat < center_lat:
        y = -y
    return x, y


def distance_meters(lat1: float, lon1: float,
                    lat2: float, lon2: float) -> float:
    """Calculate the distance in meters between two points on the earth."""
    lon1, lat1, lon2, lat2 = map(radians, (lon1, lat1, lon2, lat2))
    dlon, dlat = lon2 - lon1, lat2 - lat1
    return 12742000 * asin(sqrt(sin(dlat / 2) ** 2 +
                                sin(dlon / 2) ** 2 * cos(lat1) * cos(lat2)))


def dissolve_nodes(nodes_inv: Dict[int, int]):
    """Dissolve all possible nodes on the network."""
    node_ids = [k for k, v in INDEX.entities.items() if isinstance(v, Node)]
    dissolved = []
    for node in filter(bool, map(INDEX.entities.get, node_ids)):
        try:
            node.dissolve(delete_if_dissolved=True)
            dissolved.append(nodes_inv[node])
        except ValueError:
            pass
    log.info('Nodes dissolved:\n%s',
             '\n'.join(wrap(', '.join(map(str, dissolved)), 79)))


def filter_waypoints():
    """Remove waypoints that are too close to nodes."""
    cleared = sum(n.clear_intersecting_waypoints()
                  for n in INDEX.get_all(of_type=Node))
    log.info('Waypoints cleared: %d', cleared)


def log_config():
    """Configure logging."""
    log.basicConfig(level=log.INFO)


if __name__ == '__main__':
    main()
