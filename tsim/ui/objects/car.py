"""Car actor creation."""

from __future__ import annotations

from random import random

from panda3d.core import CollisionBox, CollisionNode, NodePath  # , PointLight
from pkg_resources import resource_filename

import tsim.ui.panda3d as p3d
from tsim.core.simulation.car import Car
from tsim.utils.color import hsv_to_rgb


def create(parent: NodePath, car: Car) -> NodePath:
    """Create actor for given `car` and reparent it to `parent`."""
    model_filename = resource_filename('tsim', 'data/models/car')
    node_path = p3d.loader.load_model(model_filename)
    node_path.name = f'car-{car.id}'
    node_path.tags['id'] = str(car.id)

    collision_solid = CollisionBox((0.0, 0.0, 0.0), 1.0, 1.0, 1.0)
    collision_np = node_path.attach_new_node(CollisionNode('cnode'))
    collision_np.node().add_solid(collision_solid)

    if car.position is not None:
        node_path.set_pos(*car.position, 0.0)
        node_path.look_at(*(car.position + car.direction), 0.0)

    node_path.set_color(*hsv_to_rgb(random() * 360.0, 0.7, 1.0, 1.0))
    node_path.reparent_to(parent)

    # light = PointLight(f'car-{car.id}-light')
    # light.set_color((1.0, 1.0, 0.6, 1.0))
    # light.set_attenuation((1.0, 0.5, 0.0))
    # light_np = node_path.attach_new_node(light)
    # light_np.set_pos((0.0, 3.0, 1.0))
    # p3d.render.set_light(light_np)

    return node_path
