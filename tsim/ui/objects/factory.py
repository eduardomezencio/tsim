"""Factory for creating nodes for entities."""

from __future__ import annotations

from typing import TYPE_CHECKING

from panda3d.core import NodePath

from tsim.core.network.node import Node
from tsim.core.network.path import Path
from tsim.core.network.way import Way
from tsim.core.simulation.car import Car
from tsim.ui.objects.car import create as create_car
from tsim.ui.objects.node import create as create_node
from tsim.ui.objects.path import create as create_path
from tsim.ui.objects.way import create as create_way

if TYPE_CHECKING:
    from tsim.core.network.entity import Entity

MAP = {Car: create_car,
       Node: create_node,
       Path: create_path,
       Way: create_way}


def create(parent: NodePath, entity: Entity) -> NodePath:
    """Create node for given entity and attach it to the parent."""
    return MAP[type(entity)](parent, entity)
