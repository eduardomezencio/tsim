"""Cursor object for the UI."""

import os

from direct.actor.Actor import Actor
from direct.showbase.ShowBase import ShowBase
from panda3d.core import NodePath, PandaNode


class Cursor:
    """Cursor object for the UI."""

    def __init__(self, base: ShowBase, parent: NodePath):
        self.base = base
        self.camera = self.base.camera
        self.lens = self.base.camLens
        self.parent = parent

        self.mouse_watcher_node = self.base.mouseWatcherNode
        self.mouse_np = self.camera.attach_new_node(PandaNode('mouse'))
        self.mouse_np.set_y(self.lens.get_near())

        self.cursor = Actor(f'{os.getcwd()}/models/cursor', {
            'spin': f'{os.getcwd()}/models/cursor-spin'
        })
        self.cursor.loop('spin')
        self.cursor.reparent_to(parent)
        self.cursor.set_pos(0.0, 0.0, 0.0)

    def update(self):
        """Update callback."""
        self.cursor.set_scale(self.camera.get_z() ** 0.6 / 4)

        if self.mouse_watcher_node.has_mouse():
            film = self.lens.get_film_size() * 0.5
            self.mouse_np.set_x(self.mouse_watcher_node.get_mouse_x() * film.x)
            self.mouse_np.set_y(self.lens.get_focal_length())
            self.mouse_np.set_z(self.mouse_watcher_node.get_mouse_y() * film.y)
            mouse_pos = self.mouse_np.get_pos(self.parent)
            cam_pos = self.camera.get_pos(self.parent)
            mouse_vec = mouse_pos - cam_pos
            if mouse_vec.z < 0.0:
                scale = -mouse_pos.z / mouse_vec.z
                self.cursor.set_pos(mouse_pos + mouse_vec * scale)
