"""App class implementation, the graphic UI main class."""

from __future__ import annotations

import logging as log

from direct.showbase.ShowBase import ShowBase
from direct.task import Task
from panda3d.core import (AmbientLight, AntialiasAttrib, ConfigVariableColor,
                          DirectionalLight, Fog, NodePath, RigidBodyCombiner,
                          load_prc_file)

from tsim.model.index import INSTANCE as INDEX
from tsim.model.network import Node, Way
from tsim.ui import textures
from tsim.ui.camera import Camera
from tsim.ui.cursor import Cursor
from tsim.ui.grid import Grid
from tsim.ui.objects import factory, world
import tsim.ui.input as INPUT


load_prc_file('config.prc')


class App:
    """Graphic UI application, using Panda3D."""

    base: ShowBase

    def __init__(self):
        log.basicConfig(format='%(levelname)s: %(message)s', level=log.DEBUG)

        self.base = ShowBase()
        self.base.task_mgr.remove('audioLoop')
        self.base.task_mgr.remove('collisionLoop')
        # print(self.base.task_mgr)  # to print all tasks

        self.render = self.base.render
        self.render.set_antialias(AntialiasAttrib.M_auto)
        # self.render.set_shader_auto()

        INPUT.init(self.base)
        textures.set_loader(self.base.loader)

        self.world = world.create(self.render, 10000, 16)
        self.camera = Camera(self.base)
        self.cursor = Cursor(self.base, self.world)
        self.grid = Grid(50.0, 1000.0, self.world, self.cursor.cursor)

        # self.roads = self.world.attach_new_node(PandaNode('roads'))
        self.roads = self.world.attach_new_node(RigidBodyCombiner('roads'))

        self.init_lights()
        self.init_fog()
        self.init_objects(self.roads)

        self.roads.node().collect()

        self.base.task_mgr.add(self.update)

    def run(self):
        """Start the main loop."""
        self.base.run()

    def update(self, _task: Task):
        """Update task, to run every frame."""
        self.camera.update()
        self.cursor.update()
        self.grid.update()

        if INPUT.pressed('select'):
            selected = INDEX.get_at(self.cursor.position, of_type=Node)
            if selected:
                print(f'{selected[0].xurl} {selected[0].lane_connections}\n')
                # INDEX.delete(selected[0])
                # self.update_entities()
                # self.roads.node().collect()
            else:
                selected = INDEX.get_at(self.cursor.position, of_type=Way)
                if selected:
                    print(f'{selected[0].xurl} {selected[0].lanes}')

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

    def init_objects(self, parent: NodePath):
        """Create all objects on the index."""
        for node in filter(lambda e: isinstance(e, Node),
                           INDEX.entities.values()):
            factory.create_node(parent, node)
        for way in filter(lambda e: isinstance(e, Way),
                          INDEX.entities.values()):
            factory.create_way(parent, way)

    def init_lights(self):
        """Create lights."""
        light = DirectionalLight('light')
        light_np = self.render.attach_new_node(light)
        light_np.set_p(240.0)
        alight = AmbientLight('alight')
        alight.set_color((0.3, 0.3, 0.3, 1.0))
        alight_np = self.render.attach_new_node(alight)
        self.cursor.cursor.set_light(light_np)
        self.cursor.cursor.set_light(alight_np)

    def init_fog(self):
        """Create distance fog."""
        fog = Fog('fog')
        fog.set_color(ConfigVariableColor('background-color'))
        fog.set_linear_range(2000.0, 7000.0)
        self.render.set_fog(fog)
