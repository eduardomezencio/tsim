"""App class implementation, the graphic UI main class."""

from __future__ import annotations

import logging

from direct.task import Task
from panda3d.core import (AntialiasAttrib, ConfigVariableBool, NodePath,
                          RigidBodyCombiner)

import tsim.ui.input as INPUT
import tsim.ui.panda3d as p3d
from tsim.model.index import INSTANCE as INDEX
from tsim.model.network.node import Node
from tsim.model.network.way import Way
from tsim.ui.camera import Camera
from tsim.ui.cursor import Cursor
from tsim.ui.grid import Grid
from tsim.ui.objects import factory, world
from tsim.ui.sky import Sky


class App:
    """Graphic UI application, using Panda3D."""

    def __init__(self):
        log_config()
        panda3d_config()

        INPUT.init()

        self.camera = Camera()
        self.world = world.create(p3d.RENDER, 10000, 16)
        self.sky = Sky(p3d.RENDER, self.camera)
        self.sky.set_time(8.0)
        self.cursor = Cursor(self.world)
        self.grid = Grid(50.0, 1000.0, self.world, self.cursor.cursor)

        # self.roads = self.world.attach_new_node(PandaNode('roads'))
        self.roads = self.world.attach_new_node(RigidBodyCombiner('roads'))
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
        self.sky.update()
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


def log_config():
    """Initialize log configuration."""
    logging.basicConfig(format='%(levelname)s: %(message)s',
                        level=logging.DEBUG if __debug__ else logging.ERROR)


def panda3d_config():
    """Initialize Panda3D global configurations."""
    p3d.TASK_MGR.remove('audioLoop')
    p3d.TASK_MGR.remove('collisionLoop')
    # print(task_mgr)  # to print all tasks

    p3d.RENDER.set_antialias(AntialiasAttrib.M_auto)
    if ConfigVariableBool('use-shaders'):
        p3d.RENDER.set_shader_auto()
