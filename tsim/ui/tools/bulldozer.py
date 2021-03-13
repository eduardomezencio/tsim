"""Bulldozer tool."""

import tsim.ui.panda3d as p3d
from tsim.core.index import INSTANCE as INDEX
from tsim.core.network.way import Way
from tsim.ui.tools.tool import Tool


class Bulldozer(Tool):
    """Tool for removing ways."""

    KEY = 'b'

    def on_button1_press(self):
        """Button 1 pressed callback."""
        selected = next(iter(INDEX.get_at(self.cursor.position, of_type=Way)),
                        None)
        if selected is not None:
            selected.delete()
            p3d.messenger.send('network_entities_changed')
