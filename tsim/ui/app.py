"""App class implementation, the graphic UI main class."""

from dataclasses import astuple

from direct.showbase.ShowBase import ShowBase
from direct.task import Task
from panda3d.core import (AntialiasAttrib, ConfigVariableColor, Fog, GeomNode,
                          load_prc_file, LODNode, NodePath)

from tsim.model.entity import EntityIndex
from tsim.model.network import Node, Way
from tsim.ui.camera import Camera
from tsim.ui.grid import Grid
from tsim.ui.input import init_input
from tsim.ui.network import build_node_geom, build_way_geom_node

load_prc_file('config.prc')


class App:
    """Graphic UI application, using Panda3D."""

    base: ShowBase
    index: EntityIndex

    def __init__(self, index: EntityIndex):
        self.index = index

        self.base = ShowBase()
        self.render = self.base.render
        self.render.set_antialias(AntialiasAttrib.M_none)
        self.base.task_mgr.remove('audioLoop')
        self.base.task_mgr.remove('collisionLoop')
        # print(self.base.task_mgr)  # to print all tasks

        fog = Fog('fog')
        fog.set_color(ConfigVariableColor('background-color'))
        fog.set_linear_range(4000.0, 5000.0)
        self.render.set_fog(fog)

        self.camera = Camera(self.base)

        init_input(self.base)

        self.create_and_add_nodes()
        self.create_and_add_ways()

        self.grid = Grid(100.0, 10000.0)
        self.grid.attach_node('grid', self.render)
        self.grid.node_path.set_z(-0.1)

        self.base.task_mgr.add(self.update)

    def run(self):
        """Start the main loop."""
        self.base.run()

    def update(self, _task: Task):
        """Update task, to run every frame."""
        self.camera.update()
        return Task.cont

    def create_and_add_nodes(self):
        """Create and add network nodes to the scene."""
        geoms = (build_node_geom(16), build_node_geom(8))
        nodes = (v for v in self.index.entities.values()
                 if isinstance(v, Node))
        for node in nodes:
            geom_nodes = [GeomNode('node{node.id}_{i}_node') for i in range(2)]
            for geom, geom_node in zip(geoms, geom_nodes):
                geom_node.add_geom(geom)
            node_paths = [NodePath(g) for g in geom_nodes]
            lod = LODNode('node{node.id}_lod')
            lod_np = NodePath(lod)
            lod_np.reparent_to(self.render)
            levels = [(250.0, 0.0), (1000.0, 250.0)]
            for bounds, node_path in zip(levels, node_paths):
                lod.add_switch(*bounds)
                node_path.set_color(0.15, 0.7, 0.95, 1.0)
                node_path.reparent_to(lod_np)
            lod_np.set_pos(*astuple(node.position), 0.0)

    def create_and_add_ways(self):
        """Create and add network ways to the scene."""
        ways = ((f'way_{k}', v) for k, v in self.index.entities.items()
                if isinstance(v, Way))
        for name, way in ways:
            node = build_way_geom_node(name, way)
            node_path = self.render.attach_new_node(node)
            node_path.set_color(0.15, 0.7, 0.95, 1.0)
