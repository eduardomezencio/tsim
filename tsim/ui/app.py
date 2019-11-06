"""App class implementation, the graphic UI main class."""

from __future__ import annotations

import logging as log
from typing import Dict, Tuple


from direct.task import Task
from panda3d.core import (AntialiasAttrib, ConfigVariableBool, NodePath,
                          RigidBodyCombiner)

import tsim.ui.input as INPUT
import tsim.ui.panda3d as p3d
from tsim.model.index import INSTANCE as INDEX
from tsim.model.geometry import Point, bounding_rect_center
from tsim.model.network.node import Node
from tsim.model.network.way import Way
from tsim.ui.camera import Camera
from tsim.ui.cursor import Cursor
from tsim.ui.grid import Grid
from tsim.ui.objects import factory, world
from tsim.ui.sky import Sky


class App:
    """Graphic UI application, using Panda3D."""

    entities: Dict[int, NodePath]
    roads: Dict[Tuple[int, int], NodePath]

    def __init__(self, index_name: str):
        log_config()
        panda3d_config()

        init_index(index_name)

        INPUT.init()

        self.scene = NodePath('scene')
        self.camera = Camera()
        self.world = world.create(self.scene, 10000, 16)
        self.sky = Sky(self.scene, self.camera)
        self.sky.set_time(8.0)
        self.cursor = Cursor(self.world)
        self.grid = Grid(50.0, 1000.0, self.world, self.cursor.cursor)

        # self.roads = self.world.attach_new_node(PandaNode('roads'))
        # self.roads = self.world.attach_new_node(RigidBodyCombiner('roads'))
        self.entities = {}
        self.roads = {}
        self.init_objects()
        for node_path in self.roads.values():
            node_path.node().collect()

        p3d.BASE.accept('entities_changed', self.on_entities_changed)
        p3d.TASK_MGR.add(self.update)

        self.scene.reparent_to(p3d.RENDER)

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
        node_path = self.entities.get(id_, None)
        if node_path is not None and not node_path.is_empty():
            node_path.remove_node()
        entity = INDEX.entities.get(id_, None)
        if entity is not None:
            point = bounding_rect_center(entity.bounding_rect)
            parent = self.get_roads_parent(point)
            self.entities[id_] = factory.create(parent, entity)

    def on_entities_changed(self):
        """Update entities."""
        self.update_entities()
        for node_path in self.roads.values():
            node_path.node().collect()

    def get_roads_parent(self, point: Point) -> NodePath:
        """Get the correct parent for roads on the given point.

        The world is divided into sections, each section with a node path
        holding a part of the network nodes. This is to make the combiner
        `collect` method faster when changing the network, while keeping the
        number of nodes small.
        """
        key = (point.x // 1024, point.y // 1024)
        parent = self.roads.get(key, None)
        if parent is None:
            combiner = RigidBodyCombiner(f'roads{key}')
            parent = self.world.attach_new_node(combiner)
            self.roads[key] = parent
        return parent

    def init_objects(self):
        """Create all objects on the index."""
        for node in filter(lambda e: isinstance(e, Node),
                           INDEX.entities.values()):
            parent = self.get_roads_parent(node.position)
            self.entities[node.id] = factory.create_node(parent, node)

        for way in filter(lambda e: isinstance(e, Way),
                          INDEX.entities.values()):
            center = bounding_rect_center(way.bounding_rect)
            parent = self.get_roads_parent(center)
            self.entities[way.id] = factory.create_way(parent, way)


def log_config():
    """Initialize log configuration."""
    log.basicConfig(format='%(levelname)s: %(message)s',
                    level=log.DEBUG if __debug__ else log.ERROR)


def panda3d_config():
    """Initialize Panda3D global configurations."""
    p3d.TASK_MGR.remove('audioLoop')
    p3d.TASK_MGR.remove('collisionLoop')
    # print(task_mgr)  # to print all tasks

    p3d.RENDER.set_antialias(AntialiasAttrib.M_auto)
    if ConfigVariableBool('use-shaders'):
        p3d.RENDER.set_shader_auto()


def init_index(name: str):
    """Initialize index from given index name."""
    INDEX.load(name)
    INDEX.register_updates = True
