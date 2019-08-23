"""Factory for creating nodes for entities."""

from __future__ import annotations

from panda3d.core import NodePath

from tsim.model.entity import Entity
from tsim.model.network import Node, Way
from tsim.ui.objects.node import create as create_node
from tsim.ui.objects.way import create as create_way

MAP = {Node: create_node,
       Way: create_way}


def create(parent: NodePath, entity: Entity) -> NodePath:
    """Create node for given entity and attach it to the parent."""
    MAP[type(entity)](parent, entity)
