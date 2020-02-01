"""Cursor object for the UI."""

from __future__ import annotations

from functools import partial
from itertools import islice
from typing import Iterable, Type, Union
import logging as log
import os

from direct.showbase.DirectObject import DirectObject
from direct.actor.Actor import Actor
from panda3d.core import (CollisionHandlerQueue, CollisionNode, CollisionRay,
                          CollisionTraverser, NodePath, PandaNode)

from tsim.model.geometry import Point
from tsim.ui.tools import Tool, TOOLS
import tsim.ui.input as INPUT
import tsim.ui.panda3d as p3d


class Cursor(DirectObject):
    """Cursor object for the UI."""

    def __init__(self, parent: NodePath):
        super().__init__()
        self.parent = parent

        self.mouse_np = p3d.CAMERA.attach_new_node(PandaNode('mouse'))
        self.mouse_np.set_y(p3d.LENS.get_near())

        picker_node = CollisionNode('mouse_ray')
        picker_np = p3d.CAMERA.attach_new_node(picker_node)
        self._picker_ray = CollisionRay()
        picker_node.add_solid(self._picker_ray)
        self._collision_handler = CollisionHandlerQueue()
        self._traverser = CollisionTraverser('mouse_traverser')
        self._traverser.add_collider(picker_np, self._collision_handler)

        self.cursor = Actor(f'{os.getcwd()}/models/cursor',
                            {'spin': f'{os.getcwd()}/models/cursor-spin'})
        self.cursor.loop('spin')
        self.cursor.reparent_to(parent)
        self.cursor.set_pos(0.0, 0.0, 0.0)
        self.cursor.set_shader_off()

        self._position = Point(0.0, 0.0)
        self.last_position = self._position
        self.moved = False

        self._tool: Tool = None
        self._register_tool_change_events()

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

    @property
    def tool(self):
        """Get current tool."""
        return self._tool

    @tool.setter
    def tool(self, value: Tool):
        if self._tool is not None:
            self._tool.cleanup()
        self.ignore_all()
        self._register_tool_change_events()
        self._tool = value
        if value is not None:
            for key in INPUT.keys_for('tool_1'):
                self.accept(key, self._tool.on_button1_press)
                self.accept(f'{key}-up', self._tool.on_button1_release)
            for key in INPUT.keys_for('tool_2'):
                self.accept(key, self._tool.on_button2_press)
                self.accept(f'{key}-up', self._tool.on_button2_release)
            for key in INPUT.keys_for('tool_3'):
                self.accept(key, self._tool.on_button3_press)
                self.accept(f'{key}-up', self._tool.on_button3_release)
            self.accept('cursor_move', self._tool.on_cursor_move)

    def update(self):
        """Update callback."""
        self.cursor.set_scale(p3d.CAMERA.get_z() ** 0.6 / 10)
        self.moved = False

        if not p3d.MOUSE_WATCHER.has_mouse():
            return

        mouse_x, mouse_y = p3d.MOUSE_WATCHER.get_mouse()
        self._picker_ray.set_from_lens(p3d.CAM_NODE, mouse_x, mouse_y)
        self._traverser.traverse(p3d.RENDER)
        if self._collision_handler.get_num_entries():
            self._collision_handler.sort_entries()
            self._on_point_at_object(self._collision_handler.get_entry(0)
                                     .get_into_node_path())
            return

        film = p3d.LENS.get_film_size() * 0.5
        self.mouse_np.set_x(mouse_x * film.x)
        self.mouse_np.set_y(p3d.LENS.get_focal_length())
        self.mouse_np.set_z(mouse_y * film.y)
        self.last_position = self._position
        mouse_pos = self.mouse_np.get_pos(self.parent)
        cam_pos = p3d.CAMERA.get_pos(self.parent)
        mouse_vec = mouse_pos - cam_pos
        if mouse_vec.z < 0.0:
            scale = -mouse_pos.z / mouse_vec.z
            self.cursor.set_pos(mouse_pos + mouse_vec * scale)
            self.position = self.cursor.get_pos()
            if self._position != self.last_position:
                self.moved = True
                p3d.MESSENGER.send('cursor_move')

    def _register_tool_change_events(self):
        def set_tool(tool: Type[Tool]):
            self.tool = tool(self)
            log.debug('Changing tool to %s', tool.__name__)
        for tool in TOOLS:
            try:
                self.accept(tool.KEY, partial(set_tool, tool))
            except AttributeError:
                log.debug('No KEY set for tool %s', tool.__name__)

    def _on_point_at_object(self, obj):
        if self._tool:
            self._tool.on_point_at_object(obj)
