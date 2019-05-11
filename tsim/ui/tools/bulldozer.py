"""Bulldozer tool."""

from tsim.model.index import INSTANCE as INDEX
from tsim.model.network import Way
from tsim.ui.tools.tool import Tool
import tsim.ui.panda3d as p3d


class Bulldozer(Tool):
    """Tool for removing ways."""

    KEY = 'b'

    def on_button1_press(self):
        """Button 1 pressed callback."""
        selected = INDEX.get_at(self.cursor.position, of_type=Way)
        if selected:
            selected = selected[0]
            INDEX.delete(selected)
            p3d.MESSENGER.send('entities_changed')
