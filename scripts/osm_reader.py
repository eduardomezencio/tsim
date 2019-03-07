"""Reader for openstreetmap xml files."""

from itertools import islice
from math import cos, sin, asin, sqrt, radians
from xml.etree import ElementTree
import sys

from tsim.model.entity import EntityIndex
from tsim.model.geometry import Point
from tsim.model.network import Node, Way


def main():
    """Osm reader main function."""
    tree = ElementTree.parse(sys.argv[1])
    root = tree.getroot()
    bounds = {k: float(v) for k, v in root.find('bounds').attrib.items()}
    center = ((bounds['maxlat'] - bounds['minlat']) / 2 + bounds['minlat'],
              (bounds['maxlon'] - bounds['minlon']) / 2 + bounds['minlon'])

    nodes = {int(n.get('id')):
             Node(Point(*coord_meters(*center, float(n.get('lat')),
                                      float(n.get('lon')))))
             for n in root.iterfind('node')}
    highways = {w.get('id'): [int(n.get('ref')) for n in w.iterfind('nd')]
                for w in root.iterfind('way')
                if any(t.get('k') == 'highway' and t.get('v') != 'footway'
                       for t in w.iterfind('tag'))}

    index = EntityIndex(sys.argv[1])
    for way in highways.values():
        for start, end in zip(way, islice(way, 1, None)):
            index.add(nodes[start])
            index.add(nodes[end])
            index.add(Way(nodes[start], nodes[end]))
    index.save()


def coord_meters(center_lat, center_lon, lat, lon):
    """Coordinates in meters relative to coordinates given as center."""
    x = distance_meters(center_lat, center_lon, center_lat, lon)
    if lon < center_lon:
        x = -x
    y = distance_meters(center_lat, center_lon, lat, center_lon)
    if lat < center_lat:
        y = -y
    return x, y


def distance_meters(lat1, lon1, lat2, lon2):
    """Calculate the distance in meters between two points on the earth."""
    lon1, lat1, lon2, lat2 = map(radians, (lon1, lat1, lon2, lat2))
    dlon, dlat = lon2 - lon1, lat2 - lat1
    return 12742000 * asin(sqrt(sin(dlat / 2) ** 2 +
                                sin(dlon / 2) ** 2 * cos(lat1) * cos(lat2)))


if __name__ == '__main__':
    main()
