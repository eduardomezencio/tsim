"""Debug tool."""

from tsim.model.index import INSTANCE as INDEX
from tsim.model.network import Node, Way
from tsim.ui.tools.tool import Tool


class Debug(Tool):
    """Tool for debugging."""

    KEY = 'i'

    def on_button1_press(self):
        """Button 1 pressed callback."""
        selected = INDEX.get_at(self.cursor.position, of_type=Node)
        if selected:
            print(selected[0].xurl)
        else:
            selected = INDEX.get_at(self.cursor.position, of_type=Way)
            if selected:
                print(selected[0].xurl)
