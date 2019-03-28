"""App class implementation, the graphic UI main class."""

from itertools import product

from direct.showbase.ShowBase import ShowBase
from direct.task import Task
from panda3d.core import (AmbientLight, AntialiasAttrib, ConfigVariableColor,
                          DecalEffect, DirectionalLight, Fog, Geom, GeomNode,
                          GeomTristrips, GeomVertexData, GeomVertexFormat,
                          GeomVertexWriter, NodePath, PandaNode, SamplerState,
                          load_prc_file)

from tsim.model.entity import EntityIndex
from tsim.ui.camera import Camera
from tsim.ui.cursor import Cursor
from tsim.ui.grid import Grid
from tsim.ui.input import clear_input, init_input
from tsim.ui.network import create_and_attach_nodes, create_and_attach_ways

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

        self.render = self.base.render
        self.render.set_antialias(AntialiasAttrib.M_auto)
        # self.render.set_shader_auto()

        self.scene = self.base.render.attach_new_node(PandaNode('scene'))

        self.camera = Camera(self.base)
        self.cursor = Cursor(self.base, self.scene)
        self.grid = Grid(50.0, 1000.0, self.render, self.cursor.cursor)

        inters_tex = self.base.loader.load_texture('textures/intersection.png')
        road_tex = self.base.loader.load_texture('textures/road.png')
        road_tex.set_minfilter(SamplerState.FT_linear_mipmap_linear)
        ground_tex = self.base.loader.load_texture('textures/ground.jpg')
        ground_tex.set_minfilter(SamplerState.FT_linear_mipmap_nearest)

        self.init_lights()
        self.init_fog()
        ground_np = create_and_attach_ground(self.scene, 10000.0, 16,
                                             ground_tex)
        ground_np.set_effect(DecalEffect.make())
        create_and_attach_nodes(self.index, ground_np, inters_tex)
        create_and_attach_ways(self.index, ground_np, road_tex)

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


def create_and_attach_ground(parent: NodePath, radius, count, texture):
    """Create and add the ground plane to the scene."""
    vertex_format = GeomVertexFormat.get_v3t2()
    vertex_data = GeomVertexData('ground', vertex_format, Geom.UH_static)
    vertex_data.set_num_rows(count ** 2)
    vertex_writer = GeomVertexWriter(vertex_data, 'vertex')
    texcoord_writer = GeomVertexWriter(vertex_data, 'texcoord')

    step = 2 * radius / count
    for i, j in product(range(count + 1), repeat=2):
        vertex_writer.add_data3f(i * step - radius, j * step - radius, 0.0)
        texcoord_writer.add_data2f(i * 512, j * 512)

    geom = Geom(vertex_data)
    primitive = GeomTristrips(Geom.UH_static)
    for j in range(count):
        rows = range(count + 1) if j % 2 else reversed(range(count + 1))
        for i in rows:
            primitive.add_vertex((j + (j + 1) % 2) * (count + 1) + i)
            primitive.add_vertex((j + j % 2) * (count + 1) + i)
    primitive.close_primitive()
    geom.add_primitive(primitive)

    node = GeomNode('ground_node')
    node.add_geom(geom)

    node_path = parent.attach_new_node(node)
    node_path.set_texture(texture)
    return node_path
