"""Debug tool."""

from direct.gui.OnscreenText import OnscreenText
from panda3d.core import NodePath, TextNode

from tsim.model.index import INSTANCE as INDEX
from tsim.model.network import Node, Way
from tsim.ui.objects.node import create_lane_connections_card
from tsim.ui.panda3d import LOADER, PIXEL2D, RENDER
from tsim.ui.tools.tool import Tool

FONT = LOADER.load_font('cmtt12.egg')


class Debug(Tool):
    """Tool for debugging."""

    hud_text: OnscreenText
    card: NodePath

    KEY = 'i'

    def prepare(self):
        """Initialize tool."""
        self.hud_text = OnscreenText(text='', pos=(10, -20), scale=22.0,
                                     align=TextNode.A_left, font=FONT,
                                     parent=PIXEL2D, mayChange=True)
        self.card = None

    def on_button1_press(self):
        """Button 1 pressed callback."""
        print(f'{self.cursor.position.x:.2f}, {self.cursor.position.y:.2f}')
        self._clear_selection()
        selected = INDEX.get_at(self.cursor.position, of_type=Node)
        if selected:
            print(selected[0].xurl)
            self.card = create_lane_connections_card(selected[0], RENDER)
        else:
            selected = INDEX.get_at(self.cursor.position, of_type=Way)
            if selected:
                print(selected[0].xurl)
                print(selected[0])

    def on_cursor_move(self):
        """Cursor moved callback."""
        self.hud_text.setText(f'{self.cursor.position.x:.2f} '
                              f'{self.cursor.position.y:.2f}')

    def cleanup(self):
        """Clean up before changing tool."""
        self.hud_text.destroy()
        self._clear_selection()

    def _clear_selection(self):
        """Clear current selection and free related resources."""
        if self.card is not None:
            self.card.get_texture().clear()
            self.card.remove_node()
            self.card = None
