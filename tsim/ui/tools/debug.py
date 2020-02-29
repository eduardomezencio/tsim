"""Debug tool."""

from __future__ import annotations

from typing import Dict, Optional

from direct.gui.OnscreenText import OnscreenText
from panda3d.core import NodePath, TextNode

from tsim.model.geometry import Point
from tsim.model.index import INSTANCE as INDEX
from tsim.model.network.node import Node
from tsim.model.network.traffic import TrafficAgent
from tsim.model.network.way import Way
from tsim.model.simulation.car import Car
from tsim.model.units import mps_to_kph
from tsim.ui.objects import factory as Factory
from tsim.ui.objects.node import create_lane_connections_card
from tsim.ui import panda3d as p3d
from tsim.ui.tools.tool import Tool

FONT = p3d.LOADER.load_font('cmtt12.egg')


class Debug(Tool):
    """Tool for debugging."""

    pressed: bool
    last_point: Point
    hud_text: OnscreenText
    card: NodePath
    text_dict: Dict[str, str]
    selected_agent: Optional[TrafficAgent]
    path_np: NodePath

    KEY = 'i'

    def prepare(self):
        """Initialize tool."""
        self.pressed = False
        self.last_point = None
        self.hud_text = OnscreenText(text='', pos=(5, -20), scale=22.0,
                                     fg=(1.0, 1.0, 1.0, 0.9),
                                     shadow=(0.0, 0.0, 0.0, 0.9),
                                     align=TextNode.A_left, font=FONT,
                                     parent=p3d.PIXEL2D, mayChange=True)
        self.card = None
        self.text_dict = {'position': None, 'network_position': None,
                          'distance': None, 'agent': None}
        self.selected_agent = None
        self.path_np = None
        self._update_position()
        self._update_hud_text()

    def on_update(self):
        """Update callback called every frame."""
        if self.selected_agent is not None:
            self.text_dict['agent'] = \
                self.selected_agent.debug_str().replace('\n', '\n' + ' ' * 4)
            self._update_hud_text()

    def on_button1_press(self):
        """Button 1 pressed callback."""
        self.pressed = True
        self._update_selection(just_pressed=True)
        point = self.cursor.position

        if self.last_point is not None:
            distance = self.last_point.distance(point)
            self.text_dict['distance'] = f'{distance:.2f}'
        self.last_point = point

        self._update_hud_text()

    def on_button1_release(self):
        """Button 1 released callback."""
        self.pressed = False

    def on_button2_press(self):
        """Button 2 pressed callback."""
        self._clear_agent()
        self._update_selection(agents_only=True, follow=False,
                               just_pressed=True, show_path=False)
        if isinstance(self.selected_agent, Car):
            self.selected_agent.remove(INDEX.simulation.ready_buffer,
                                       INDEX.simulation.ready_buffer, False)
        self._clear_agent()

    def on_button3_press(self):
        """Button 3 pressed callback."""
        self._clear_selection()
        self._update_hud_text()

    def on_cursor_move(self):
        """Cursor moved callback."""
        if self.pressed:
            self._update_selection()
        self._update_position()
        self._update_hud_text()

    def cleanup(self):
        """Clean up before changing tool."""
        self.hud_text.destroy()
        self._clear_selection()

    def _update_selection(self, *, agents_only: bool = False,
                          follow: bool = True, just_pressed: bool = False,
                          show_path: bool = True):
        agent_selected = False
        if just_pressed and self.cursor.pointed_at:
            id_ = int(self.cursor.pointed_at.parent.tags['id'])
            agent = INDEX.entities.get(id_, None)
            if agent is not None and hasattr(agent, 'debug_str'):
                self.selected_agent = agent
                agent_selected = True
                if show_path:
                    self._update_path_np()
                if follow:
                    p3d.MESSENGER.send('follow', [agent])
        if agents_only:
            return

        selected = INDEX.get_at(self.cursor.position, of_type=Node)
        if selected and just_pressed:
            if not agent_selected:
                self._clear_intersection()
                self.card = create_lane_connections_card(
                    selected[0], p3d.RENDER)
        else:
            selected = INDEX.get_at(self.cursor.position, of_type=Way)

        try:
            selected = selected[0]
        except IndexError:
            return

        self.text_dict['network_position'] = f'{selected} - {selected.xurl}'

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

            self.text_dict['network_position'] += (
                f'\n    way_position={way_position:.2f}, '
                f'oriented_position={oriented_position:.2f},'
                f'\n    lane_position={lane_position:.2f}, '
                f'lane={lane_index}, '
                f'max_speed={mps_to_kph(selected.max_speed):.1f} kph')

    def _clear_selection(self):
        """Clear all seleted objects."""
        self._clear_agent()
        self._clear_intersection()
        self.text_dict['network_position'] = None
        self.text_dict['distance'] = None

    def _clear_agent(self):
        """Clear current agent selection."""
        self.selected_agent = None
        self.text_dict['agent'] = None
        self._clear_path_np()

    def _clear_path_np(self):
        """Clear current path mesh and node path."""
        if self.path_np is not None:
            self.path_np.remove_node()
            self.path_np = None

    def _clear_intersection(self):
        """Clear current intersection selection."""
        if self.card is not None:
            self.card.get_texture().clear()
            self.card.remove_node()
            self.card = None

    def _update_position(self):
        self.text_dict['position'] = (f'{self.cursor.position.x:8.2f}, '
                                      f'{self.cursor.position.y:8.2f}')

    def _update_path_np(self):
        self._clear_path_np()
        try:
            path = self.selected_agent.path
            self.path_np = Factory.create_path(p3d.RENDER, path)
        except AttributeError:
            pass

    def _update_hud_text(self):
        self.hud_text.text = '\n'.join(f"\n['{k}']: {v}"
                                       for k, v in self.text_dict.items()
                                       if v is not None)
