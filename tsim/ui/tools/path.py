"""Path tool."""

from __future__ import annotations

import logging as log
from typing import TYPE_CHECKING

from direct.gui.OnscreenText import OnscreenText
from panda3d.core import NodePath, TextNode

from tsim.model.index import INSTANCE as INDEX
from tsim.model.network.node import Node
from tsim.ui.objects.path import create as create_path
from tsim.ui.panda3d import LOADER, PIXEL2D, RENDER
from tsim.ui.tools.tool import Tool

if TYPE_CHECKING:
    from tsim.model.network.path import Path

FONT = LOADER.load_font('cmtt12.egg')


class PathTool(Tool):
    """Tool for finding and showing pathes."""

    source: Node
    dest: Node
    path: Path
    hud_text: OnscreenText
    path_np: NodePath

    KEY = 'p'

    def prepare(self):
        """Initialize tool."""
        self.source = None
        self.dest = None
        self.path = None
        self.hud_text = OnscreenText(text='', pos=(5, -20), scale=22.0,
                                     fg=(1.0, 1.0, 1.0, 1.0),
                                     shadow=(0.0, 0.0, 0.0, 1.0),
                                     align=TextNode.A_left, font=FONT,
                                     parent=PIXEL2D, mayChange=True)
        self.path_np = None
        self._update_hud_text()

    def on_button1_press(self):
        """Button 1 pressed callback."""
        self._change_node('source')

    def on_button2_press(self):
        """Button 2 pressed callback."""
        self._swap_nodes()

    def on_button3_press(self):
        """Button 3 pressed callback."""
        self._change_node('dest')

    def cleanup(self):
        """Clean up before changing tool."""
        self._clear_path_np()
        self.hud_text.destroy()

    def _change_node(self, node: str):
        selected = next(iter(INDEX.get_at(self.cursor.position, of_type=Node)),
                        None)
        setattr(self, node, selected)
        self._update_path_np()
        self._update_hud_text()

    def _swap_nodes(self):
        self.source, self.dest = self.dest, self.source
        self._update_path_np()
        self._update_hud_text()

    def _clear_path_np(self):
        if self.path_np is not None:
            self.path_np.remove_node()
            self.path_np = None

    def _update_path_np(self):
        if self.source and self.dest:
            self._clear_path_np()
            self.path = INDEX.path_map.path(self.source, self.dest)
            if self.path is not None:
                self.path_np = create_path(RENDER, self.path)

    def _update_hud_text(self):
        source_text = self.source.id if self.source else '_'
        dest_text = self.dest.id if self.dest else '_'
        info_text = (f'len: {self.path.length:6.1f}    '
                     f'weight: {self.path.weight:6.1f}    '
                     f'ways: {len(self.path.ways)}    '
                     if self.path is not None else '')
        text = f'{info_text}({source_text}, {dest_text})'
        self.hud_text.text = text

        if self.source and self.dest:
            log.debug(text)
