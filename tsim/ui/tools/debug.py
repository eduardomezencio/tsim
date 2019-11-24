"""Debug tool."""

from __future__ import annotations

import logging as log
from itertools import islice
from typing import List

from direct.gui.OnscreenText import OnscreenText
from panda3d.core import NodePath, TextNode

from tsim.model.geometry import Point
from tsim.model.index import INSTANCE as INDEX
from tsim.model.network.node import Node
from tsim.model.network.way import Way
from tsim.ui.objects.node import create_lane_connections_card
from tsim.ui.panda3d import LOADER, PIXEL2D, RENDER
from tsim.ui.tools.tool import Tool

FONT = LOADER.load_font('cmtt12.egg')
LINES = 4


class Debug(Tool):
    """Tool for debugging."""

    pressed: bool
    last_point: Point
    hud_text: OnscreenText
    card: NodePath
    text_lines: List[str]

    KEY = 'i'

    def prepare(self):
        """Initialize tool."""
        self.pressed = False
        self.last_point = None
        self.hud_text = OnscreenText(text='', pos=(5, -20), scale=22.0,
                                     fg=(1.0, 1.0, 1.0, 0.9),
                                     shadow=(0.0, 0.0, 0.0, 0.9),
                                     align=TextNode.A_left, font=FONT,
                                     parent=PIXEL2D, mayChange=True)
        self.card = None
        self.text_lines = [None] * LINES
        self._update_position()
        self._update_hud_text()

    def on_button1_press(self):
        """Button 1 pressed callback."""
        self.pressed = True
        self._update_selection()
        point = self.cursor.position

        if self.last_point is not None:
            distance = self.last_point.distance(point)
            self.text_lines[3] = f'Distance: {distance:.2f}'
        self.last_point = point

        log.debug('%.2f, %.2f', point.x, point.y)
        log.debug('\n'.join(l for l in islice(self.text_lines, 1, None)
                            if l is not None))
        self._update_hud_text()

    def on_button1_release(self):
        """Button 1 released callback."""
        self.pressed = False

    def on_cursor_move(self):
        """Cursor moved callback."""
        if self.pressed:
            self._update_selection(False)
        self._update_position()
        self._update_hud_text()

    def cleanup(self):
        """Clean up before changing tool."""
        self.hud_text.destroy()
        self._clear_selection()

    def _update_selection(self, draw_intersection: bool = True):
        self._clear_selection()
        selected = INDEX.get_at(self.cursor.position, of_type=Node)
        if selected and draw_intersection:
            self.card = create_lane_connections_card(selected[0], RENDER)
        else:
            selected = INDEX.get_at(self.cursor.position, of_type=Way)

        try:
            selected = selected[0]
        except IndexError:
            return

        self.text_lines[1] = f'{selected} - {selected.xurl}'
        if isinstance(selected, Way):
            position = selected.way_position_raw(self.cursor.position)
            way_position, lane_index = position
            lane = selected.lanes[lane_index]

            try:
                lane_position = lane.way_to_lane_position(way_position)
            except ValueError:
                lane_position = 0.0

            oriented_position = (lane.lane_to_oriented_position(lane_position)
                                 if lane_position is not None else 0.0)

            self.text_lines[2] = (
                f'way_position={way_position:.2f}, '
                f'oriented_position={oriented_position:.2f}, '
                f'lane_position={lane_position:.2f}, '
                f'lane={lane_index}')

    def _clear_selection(self):
        """Clear current selection and free related resources."""
        if self.card is not None:
            self.card.get_texture().clear()
            self.card.remove_node()
            self.card = None
        self.text_lines = [None] * LINES
        self._update_position()

    def _update_position(self):
        self.text_lines[0] = (f'x: {self.cursor.position.x:8.2f}  '
                              f'y: {self.cursor.position.y:8.2f}')

    def _update_hud_text(self):
        self.hud_text.text = '\n'.join(l for l in self.text_lines
                                       if l is not None)
