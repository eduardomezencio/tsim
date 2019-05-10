"""App class implementation, the graphic UI main class."""

from __future__ import annotations

import logging

from direct.task import Task
from panda3d.core import (AmbientLight, AntialiasAttrib, ConfigVariableColor,
                          DirectionalLight, Fog, NodePath, RigidBodyCombiner)

from tsim.model.index import INSTANCE as INDEX
from tsim.model.network import Node, Way
from tsim.ui.camera import Camera
from tsim.ui.cursor import Cursor
from tsim.ui.grid import Grid
from tsim.ui.objects import factory, world
import tsim.ui.panda3d as p3d
import tsim.ui.input as INPUT


class App:
    """Graphic UI application, using Panda3D."""

    def __init__(self):
        log_config()
        panda3d_config()

        INPUT.init()

        self.world = world.create(p3d.RENDER, 10000, 16)
        self.camera = Camera()
        self.cursor = Cursor(self.world)
        self.grid = Grid(50.0, 1000.0, self.world, self.cursor.cursor)

        # self.roads = self.world.attach_new_node(PandaNode('roads'))
        self.roads = self.world.attach_new_node(RigidBodyCombiner('roads'))

        init_lights()
        init_fog()
        init_objects(self.roads)

        self.roads.node().collect()

        p3d.BASE.accept('entities_changed', self.on_entities_changed)
        p3d.TASK_MGR.add(self.update)

    def run(self):
        """Start the main loop."""
        p3d.BASE.run()

    def update(self, _task: Task):
        """Update task, to run every frame."""
        self.camera.update()
        self.cursor.update()
        self.grid.update()
        INPUT.clear()
        return Task.cont

    def update_entities(self):
        """Update graphics for changed entities."""
        for id_ in INDEX.consume_updates():
            self._update_entity(id_)

    def _update_entity(self, id_):
        node_path = self.roads.find(str(id_))
        if not node_path.is_empty():
            node_path.remove_node()
        entity = INDEX.entities.get(id_, None)
        if entity is not None:
            factory.create(self.roads, entity)

    def on_entities_changed(self):
        """Update entities."""
        self.update_entities()
        self.roads.node().collect()


def init_objects(parent: NodePath):
    """Create all objects on the index."""
    for node in filter(lambda e: isinstance(e, Node),
                       INDEX.entities.values()):
        factory.create_node(parent, node)
    for way in filter(lambda e: isinstance(e, Way),
                      INDEX.entities.values()):
        factory.create_way(parent, way)


def init_lights():
    """Create lights."""
    light = DirectionalLight('light')
    light_np = p3d.RENDER.attach_new_node(light)
    light_np.set_p(240.0)
    alight = AmbientLight('alight')
    alight.set_color((0.3, 0.3, 0.3, 1.0))
    alight_np = p3d.RENDER.attach_new_node(alight)
    p3d.RENDER.set_light(light_np)
    p3d.RENDER.set_light(alight_np)


def init_fog():
    """Create distance fog."""
    fog = Fog('fog')
    fog.set_color(ConfigVariableColor('background-color'))
    fog.set_linear_range(2000.0, 7000.0)
    p3d.RENDER.set_fog(fog)


def log_config():
    """Initialize log configuration."""
    logging.basicConfig(format='%(levelname)s: %(message)s',
                        level=logging.ERROR)


def panda3d_config():
    """Initialize Panda3D global configurations."""
    p3d.TASK_MGR.remove('audioLoop')
    p3d.TASK_MGR.remove('collisionLoop')
    # print(task_mgr)  # to print all tasks

    p3d.RENDER.set_antialias(AntialiasAttrib.M_auto)
    # render.set_shader_auto()

    # win.set_clear_color_active(True)
    # win.set_clear_color(ConfigVariableColor('background-color'))
