"""App class implementation, the graphic UI main class."""

from dataclasses import astuple
from typing import Tuple

from direct.showbase.ShowBase import ShowBase
from direct.task import Task
from panda3d.core import (AntialiasAttrib, Geom, GeomNode, load_prc_file,
                          NodePath)

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
        self.base.render.set_antialias(AntialiasAttrib.M_none)
        self.base.task_mgr.remove('audioLoop')
        self.base.task_mgr.remove('collisionLoop')
        # print(self.base.task_mgr)  # to print all tasks

        self.camera = Camera(self.base)

        init_input(self.base)

        self.create_and_add_nodes()
        self.create_and_add_ways()

        self.grid = Grid(10.0, 3000.0)
        self.grid.attach_node('grid', self.base.render)
        self.grid.node_path.set_z(-0.1)

        self.base.task_mgr.add(self.update)

    def run(self):
        """Start the main loop."""
        self.base.run()

    def update(self, _task: Task):
        """Main update task, to run every frame."""
        self.camera.update()
        return Task.cont

    def create_and_add_nodes(self):
        """Create and add network nodes to the scene."""
        geom = build_node_geom()
        nodes = (v for v in self.index.entities.values()
                 if isinstance(v, Node))
        for node in nodes:
            self.attach_node_from_geom(f'node{node.id}_node', geom,
                                       astuple(node.position))

    def create_and_add_ways(self):
        """Create and add network ways to the scene."""
        ways = ((f'way_{k}', v) for k, v in self.index.entities.items()
                if isinstance(v, Way))
        for name, way in ways:
            node = build_way_geom_node(name, way)
            self.base.render.attach_new_node(node)

    def attach_node_from_geom(self, name: str, geom: Geom,
                              position: Tuple[float], parent: NodePath = None):
        """Create a node from a Geom object and attach it to render."""
        if parent is None:
            parent = self.base.render
        node = GeomNode(name)
        node.add_geom(geom)
        node_path = parent.attach_new_node(node)
        node_path.set_pos(*position, 0.0)
        return node_path
