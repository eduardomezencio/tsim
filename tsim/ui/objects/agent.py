"""Agent actor creation."""

from __future__ import annotations

import os
from random import random

from panda3d.core import NodePath

import tsim.ui.panda3d as p3d
from tsim.model.simulation.agent import Agent
from tsim.utils.color import hsv_to_rgb


def create(parent: NodePath, agent: Agent) -> NodePath:
    """Create actor for given `agent` and reparent it to `parent`."""
    node_path: NodePath = p3d.LOADER.load_model(f'{os.getcwd()}/models/car')

    if agent.position is not None:
        node_path.set_pos(*agent.position, 0.0)
        node_path.look_at(*(agent.position + agent.direction), 0.0)

    node_path.set_color(*hsv_to_rgb(random() * 360.0, 0.7, 1.0, 1.0))

    node_path.reparent_to(parent)
    return node_path
