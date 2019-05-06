"""Cursor object for the UI."""

from itertools import islice
from typing import Iterable, Union
import os

from direct.actor.Actor import Actor
from panda3d.core import NodePath, PandaNode

from tsim.model.geometry import Point
from tsim.ui.panda3d import P3D_CAMERA, P3D_LENS, P3D_MOUSE_WATCHER


class Cursor:
    """Cursor object for the UI."""

    def __init__(self, parent: NodePath):
        self.parent = parent

        self.mouse_np = P3D_CAMERA.attach_new_node(PandaNode('mouse'))
        self.mouse_np.set_y(P3D_LENS.get_near())

        self.cursor = Actor(f'{os.getcwd()}/models/cursor',
                            {'spin': f'{os.getcwd()}/models/cursor-spin'})
        self.cursor.loop('spin')
        self.cursor.reparent_to(parent)
        self.cursor.set_pos(0.0, 0.0, 0.0)

        self._position = Point(0.0, 0.0)
        self.last_position = self._position
        self.moved = False

    @property
    def position(self) -> Point:
        """Get the cursor position."""
        return self._position

    @position.setter
    def position(self, value: Union[Point, Iterable]):
        if not isinstance(value, Point):
            value = Point(*islice(value, 2))
        self.cursor.set_x(value.x)
        self.cursor.set_y(value.y)
        self._position = value

    def update(self):
        """Update callback."""
        self.cursor.set_scale(P3D_CAMERA.get_z() ** 0.6 / 10)
        self.moved = False

        if P3D_MOUSE_WATCHER.has_mouse():
            self.last_position = self._position

            film = P3D_LENS.get_film_size() * 0.5
            self.mouse_np.set_x(P3D_MOUSE_WATCHER.get_mouse_x() * film.x)
            self.mouse_np.set_y(P3D_LENS.get_focal_length())
            self.mouse_np.set_z(P3D_MOUSE_WATCHER.get_mouse_y() * film.y)
            mouse_pos = self.mouse_np.get_pos(self.parent)
            cam_pos = P3D_CAMERA.get_pos(self.parent)
            mouse_vec = mouse_pos - cam_pos
            if mouse_vec.z < 0.0:
                scale = -mouse_pos.z / mouse_vec.z
                self.cursor.set_pos(mouse_pos + mouse_vec * scale)
                self.position = self.cursor.get_pos()
                if self._position != self.last_position:
                    self.moved = True
