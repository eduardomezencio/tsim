"""App class implementation, the graphic UI main class."""

from __future__ import annotations

import logging as log

from direct.task import Task
from panda3d.core import (AmbientLight, AntialiasAttrib, ConfigVariableColor,
                          DirectionalLight, Fog, NodePath, RigidBodyCombiner)

from tsim.model.index import INSTANCE as INDEX
from tsim.model.network import Node, Way
from tsim.ui.camera import Camera
from tsim.ui.cursor import Cursor
from tsim.ui.grid import Grid
from tsim.ui.objects import factory, world
from tsim.ui.panda3d import P3D_BASE, P3D_RENDER, P3D_TASK_MGR
import tsim.ui.input as INPUT


class App:
    """Graphic UI application, using Panda3D."""

    def __init__(self):
        log.basicConfig(format='%(levelname)s: %(message)s', level=log.ERROR)
        panda3d_init()
        INPUT.init()

        self.world = world.create(P3D_RENDER, 10000, 16)
        self.camera = Camera()
        self.cursor = Cursor(self.world)
        self.grid = Grid(50.0, 1000.0, self.world, self.cursor.cursor)

        # self.roads = self.world.attach_new_node(PandaNode('roads'))
        self.roads = self.world.attach_new_node(RigidBodyCombiner('roads'))

        init_lights()
        init_fog()
        init_objects(self.roads)

        self.roads.node().collect()

        P3D_TASK_MGR.add(self.update)

    def run(self):
        """Start the main loop."""
        P3D_BASE.run()

    def update(self, _task: Task):
        """Update task, to run every frame."""
        self.camera.update()
        self.cursor.update()
        self.grid.update()

        # if INPUT.pressed('select'):
        #     selected = INDEX.get_at(self.cursor.position, of_type=Node)
        #     if selected:
        #         print(f'{selected[0].xurl}\n')
        #     else:
        #         selected = INDEX.get_at(self.cursor.position, of_type=Way)
        #         if selected:
        #             selected = selected[0]
        #             INDEX.delete(selected)
        #             self.update_entities()
        #             self.roads.node().collect()
        #             print(f'{selected.xurl} {selected.lanes}\n')

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
    light_np = P3D_RENDER.attach_new_node(light)
    light_np.set_p(240.0)
    alight = AmbientLight('alight')
    alight.set_color((0.3, 0.3, 0.3, 1.0))
    alight_np = P3D_RENDER.attach_new_node(alight)
    P3D_RENDER.set_light(light_np)
    P3D_RENDER.set_light(alight_np)


def init_fog():
    """Create distance fog."""
    fog = Fog('fog')
    fog.set_color(ConfigVariableColor('background-color'))
    fog.set_linear_range(2000.0, 7000.0)
    P3D_RENDER.set_fog(fog)


def panda3d_init():
    """Initialize Panda3D global configurations."""
    P3D_TASK_MGR.remove('audioLoop')
    P3D_TASK_MGR.remove('collisionLoop')
    # print(task_mgr)  # to print all tasks

    P3D_RENDER.set_antialias(AntialiasAttrib.M_auto)
    # render.set_shader_auto()

    # win.set_clear_color_active(True)
    # win.set_clear_color(ConfigVariableColor('background-color'))
