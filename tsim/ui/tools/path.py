"""Path tool."""

from __future__ import annotations

import logging as log
from typing import TYPE_CHECKING, Optional

from direct.gui.OnscreenText import OnscreenText
from panda3d.core import NodePath, TextNode

import tsim.ui.panda3d as p3d
from tsim.model.index import INSTANCE as INDEX
from tsim.model.network.lane import LanePosition
from tsim.model.network.way import Way
from tsim.model.simulation.car import Car
from tsim.ui.objects import factory as Factory
from tsim.ui.tools.tool import Tool

if TYPE_CHECKING:
    from tsim.model.network.path import Path

FONT = p3d.LOADER.load_font('cmtt12.egg')


class PathTool(Tool):
    """Tool for finding and showing pathes."""

    source: Optional[LanePosition]
    dest: Optional[LanePosition]
    path: Path
    hud_text: OnscreenText
    path_np: NodePath

    KEY = 'p'

    def prepare(self):
        """Initialize tool."""
        self.source = None
        self.dest = None
        self.path = None
        self.source_way_position = 0.0
        self.dest_way_position = 0.0
        self.hud_text = OnscreenText(text='', pos=(5, -20), scale=22.0,
                                     fg=(1.0, 1.0, 1.0, 0.9),
                                     shadow=(0.0, 0.0, 0.0, 0.9),
                                     align=TextNode.A_left, font=FONT,
                                     parent=p3d.PIXEL2D, mayChange=True)
        self.path_np = None
        self._update_hud_text()

    def on_button1_press(self):
        """Button 1 pressed callback."""
        if self.cursor.pointed_at:
            id_ = int(self.cursor.pointed_at.parent.tags['id'])
            agent = INDEX.entities.get(id_, None)
            if agent is not None and hasattr(agent, 'debug_str'):
                log.debug("[%s] %s", __name__, agent.debug_str())
                return
        self._change_way('source')

    def on_button2_press(self):
        """Button 2 pressed callback."""
        self._swap_ways()

    def on_button3_press(self):
        """Button 3 pressed callback."""
        self._change_way('dest')

    def cleanup(self):
        """Clean up before changing tool."""
        self._clear_path_np()
        self.hud_text.destroy()

    def _change_way(self, way: str):
        selected = next(iter(INDEX.get_at(self.cursor.position, of_type=Way)),
                        None)
        position = (None if selected is None else
                    selected.lane_position_at(self.cursor.position))
        setattr(self, way, position)
        self._update_path()
        self._update_hud_text()

    def _swap_ways(self):
        self.source, self.dest = self.dest, self.source
        self._update_path()
        self._update_hud_text()

    def _update_path(self):
        if self.source and self.dest:
            self.path = INDEX.path_map.path(self.source.oriented_way_position,
                                            self.dest.oriented_way_position)
            self._update_path_np()
            self._create_agent()

    def _update_path_np(self):
        self._clear_path_np()
        if self.path is not None:
            self.path_np = Factory.create_path(p3d.RENDER, self.path)

    def _clear_path_np(self):
        if self.path_np is not None:
            self.path_np.remove_node()
            self.path_np = None

    def _update_hud_text(self):
        source_text = (f'{self.source.oriented_way.way_id}'
                       f'{self.source.oriented_way.endpoint.name[0]}'
                       if self.source else '_')
        dest_text = (f'{self.dest.oriented_way.way_id}'
                     f'{self.dest.oriented_way.endpoint.name[0]}'
                     if self.dest else '_')
        info_text = (f'len: {self.path.length:6.1f}    '
                     f'weight: {self.path.weight:6.1f}    '
                     f'ways: {len(self.path.ways)}    '
                     if self.path is not None else '')
        text = f'{info_text}({source_text}, {dest_text})'
        self.hud_text.text = text

        if self.source and self.dest:
            log.debug("[%s] %s", __name__, text)

    def _create_agent(self):
        if self.path is None:
            return
        p3d.MESSENGER.send('add_car', [Car(), self.source,
                                       self.dest.oriented_way_position])
