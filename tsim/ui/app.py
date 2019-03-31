"""App class implementation, the graphic UI main class."""

from direct.showbase.ShowBase import ShowBase
from direct.task import Task
from panda3d.core import (AmbientLight, AntialiasAttrib, ConfigVariableColor,
                          DirectionalLight, Fog, PandaNode, load_prc_file)

from tsim.model.entity import EntityIndex
from tsim.ui import textures
from tsim.ui.camera import Camera
from tsim.ui.cursor import Cursor
from tsim.ui.grid import Grid
from tsim.ui.input import clear_input, init_input
from tsim.ui.meshgen.ground import create_and_attach_ground
from tsim.ui.meshgen.network import (create_and_attach_nodes,
                                     create_and_attach_ways)

load_prc_file('config.prc')


class App:
    """Graphic UI application, using Panda3D."""

    base: ShowBase
    index: EntityIndex

    def __init__(self, index: EntityIndex):
        self.index = index

        self.base = ShowBase()
        self.base.task_mgr.remove('audioLoop')
        self.base.task_mgr.remove('collisionLoop')
        # print(self.base.task_mgr)  # to print all tasks

        init_input(self.base)
        textures.set_loader(self.base.loader)

        self.render = self.base.render
        self.render.set_antialias(AntialiasAttrib.M_auto)
        # self.render.set_shader_auto()

        self.scene = self.base.render.attach_new_node(PandaNode('scene'))

        self.camera = Camera(self.base)
        self.cursor = Cursor(self.base, self.scene)
        self.grid = Grid(50.0, 1000.0, self.render, self.cursor.cursor)

        self.init_lights()
        self.init_fog()
        ground_np = create_and_attach_ground(self.scene, 10000.0, 16)
        create_and_attach_nodes(self.index, ground_np)
        create_and_attach_ways(self.index, ground_np)

        self.scene.flatten_strong()

        self.base.task_mgr.add(self.update)

    def run(self):
        """Start the main loop."""
        self.base.run()

    def update(self, _task: Task):
        """Update task, to run every frame."""
        self.camera.update()
        self.cursor.update()
        self.grid.update()
        clear_input()
        return Task.cont

    def init_lights(self):
        """Create lights for the scene."""
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
