"""Factory for creating nodes for entities."""

from __future__ import annotations

from typing import TYPE_CHECKING

from panda3d.core import NodePath

from tsim.model.network.node import Node
from tsim.model.network.way import Way
from tsim.model.simulation.agent import Agent
from tsim.ui.objects.agent import create as create_agent
from tsim.ui.objects.node import create as create_node
from tsim.ui.objects.way import create as create_way

if TYPE_CHECKING:
    from tsim.model.network.entity import Entity

MAP = {Agent: create_agent,
       Node: create_node,
       Way: create_way}


def create(parent: NodePath, entity: Entity) -> NodePath:
    """Create node for given entity and attach it to the parent."""
    return MAP[type(entity)](parent, entity)
