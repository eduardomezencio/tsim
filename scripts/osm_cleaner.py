#!/usr/bin/env python3
"""Remove unecessary information from osm xml files.

Usage: python osm_cleaner in_file out_file
"""

from itertools import chain
from xml.etree import ElementTree
import sys

BAD_HIGHWAYS = {'bridleway', 'cycleway', 'footway', 'path', 'pedestrian',
                'raceway', 'service', 'steps'}
GOOD_ATTRIBS = {'id', 'lat', 'lon'}
NODE_GOOD_TAGS = {'highway'}
RELATION_GOOD_TAGS = {'restriction', 'type'}


def main():
    """Osm cleaner main function."""
    tree = ElementTree.parse(sys.argv[1])
    root = tree.getroot()
    used_nodes = set()
    clean_ways(root, used_nodes)
    clean_relations(root, used_nodes)
    clean_nodes(root, used_nodes)
    remove_invalid_nodes(root)
    tree.write(f'{sys.argv[2]}', 'UTF-8', xml_declaration=True)


def clean_ways(root, used_nodes):
    """Remove unwanted ways and way attribs."""
    trash = []
    for way in root.iterfind('way'):
        for node in way.iterfind('nd'):
            used_nodes.add(node.get('ref'))
        if all(tag.get('k') != 'highway' or tag.get('v') in BAD_HIGHWAYS
               for tag in way.iterfind('tag')):
            trash.append(way)
        else:
            attribs = [a for a in way.attrib if a not in GOOD_ATTRIBS]
            for attrib in attribs:
                del way.attrib[attrib]
    for item in trash:
        root.remove(item)


def clean_relations(root, used_nodes):
    """Remove unwanted relations and relation attribs."""
    trash = []
    for rel in root.iterfind('relation'):
        for node in rel.iterfind('nd'):
            used_nodes.add(node.get('ref'))
        if all(tag.get('k') != 'restriction' for tag in rel.iterfind('tag')):
            trash.append(rel)
        else:
            attribs = [a for a in rel.attrib if a not in GOOD_ATTRIBS]
            for attrib in attribs:
                del rel.attrib[attrib]
            tag_trash = [t for t in rel.iterfind('tag')
                         if t.get('k') not in RELATION_GOOD_TAGS]
            for tag in tag_trash:
                rel.remove(tag)
    for item in trash:
        root.remove(item)


def clean_nodes(root, used_nodes):
    """Remove unwanted or unused nodes and node attribs."""
    trash = [n for n in root.iterfind('node') if n.get('id') not in used_nodes]
    for item in trash:
        root.remove(item)

    for node in root.iterfind('node'):
        attribs = [a for a in node.attrib if a not in GOOD_ATTRIBS]
        for attrib in attribs:
            del node.attrib[attrib]
        tag_trash = [t for t in node.iterfind('tag')
                     if t.get('k') not in NODE_GOOD_TAGS]
        for tag in tag_trash:
            node.remove(tag)


def remove_invalid_nodes(root):
    """Remove invalid node references from ways and relations."""
    nodes = {n.get('id') for n in root.iterfind('node')}
    for thing in chain(root.iterfind('way'), root.iterfind('relation')):
        trash = [n for n in thing.iterfind('nd') if n.get('ref') not in nodes]
        for node in trash:
            thing.remove(node)


if __name__ == '__main__':
    main()
