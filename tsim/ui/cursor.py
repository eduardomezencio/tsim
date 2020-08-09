"""Cursor object for the UI."""

from __future__ import annotations

from functools import partial
from itertools import islice
from typing import Iterable, Optional, Type, Union
import logging as log
import os

from direct.showbase.DirectObject import DirectObject
from direct.actor.Actor import Actor
from panda3d.core import (CollisionHandlerQueue, CollisionNode, CollisionRay,
                          CollisionTraverser, NodePath, PandaNode)

from tsim.model.geometry import Point
from tsim.model.units import Duration
from tsim.ui.tools import Tool, TOOLS
import tsim.ui.input as INPUT
import tsim.ui.panda3d as p3d


class Cursor(DirectObject):
    """Cursor object for the UI."""

    parent: NodePath
    mouse_np: NodePath
    actor: Actor
    last_position: Point
    moved: bool
    pointed_at: Optional[NodePath]

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

        self.actor = Actor(f'{os.getcwd()}/models/cursor',
                           {'spin': f'{os.getcwd()}/models/cursor-spin'})
        self.actor.loop('spin')
        self.actor.reparent_to(parent)
        self.actor.set_pos(0.0, 0.0, 0.0)
        self.actor.set_shader_off()

        self._position = Point(0.0, 0.0)
        self.last_position = self._position
        self.moved = False
        self.pointed_at = None

        self._tool: Optional[Tool] = None
        self._register_events()

    @property
    def position(self) -> Point:
        """Get the cursor position."""
        return self._position

    @position.setter
    def position(self, value: Union[Point, Iterable]):
        if not isinstance(value, Point):
            value = Point(*islice(value, 2))
        self.actor.set_x(value.x)
        self.actor.set_y(value.y)
        self._position = value

    @property
    def tool(self) -> Optional[Tool]:
        """Get current tool."""
        return self._tool

    @tool.setter
    def tool(self, value: Tool):
        if self._tool is not None:
            self._tool.cleanup()
        self.ignore_all()
        self._register_events()
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
        self.actor.set_scale(p3d.CAMERA.get_z() ** 0.6 / 10)
        self.moved = False

        if p3d.MOUSE_WATCHER.has_mouse():
            mouse_x, mouse_y = p3d.MOUSE_WATCHER.get_mouse()
            self._picker_ray.set_from_lens(p3d.CAM_NODE, mouse_x, mouse_y)
            self._traverser.traverse(p3d.RENDER)

            if self._collision_handler.get_num_entries():
                self._collision_handler.sort_entries()
                node_path = (self._collision_handler.get_entry(0)
                             .get_into_node_path())
                self.position = node_path.get_pos(p3d.RENDER)
                self.actor.set_z(2.0)
                self.pointed_at = node_path
            else:
                self.pointed_at = None
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
                    self.actor.set_pos(mouse_pos + mouse_vec * scale)
                    self.position = self.actor.get_pos()
                    if self._position != self.last_position:
                        self.moved = True
                        p3d.MESSENGER.send('cursor_move')

        if self._tool is not None:
            self._tool.on_update()

    def _on_simulation_step(self, dt: Duration):
        if self._tool is not None:
            self._tool.on_simulation_step(dt)

    def _register_events(self):
        def set_tool(tool: Type[Tool]):
            self.tool = tool(self)
            log.info('[%s] Changing tool to %s',
                     __name__, tool.__name__)
        for tool in TOOLS:
            try:
                self.accept(tool.KEY, partial(set_tool, tool))
            except AttributeError:
                log.warning('[%s] No KEY set for tool %s',
                            __name__, tool.__name__)
        self.accept('simulation_step', self._on_simulation_step)
