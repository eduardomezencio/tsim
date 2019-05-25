"""Tool base class."""

from __future__ import annotations

from abc import ABC
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from tsim.ui.cursor import Cursor


class Tool(ABC):
    """Base class for cursor tools."""

    cursor: Cursor

    def __init__(self, cursor: Cursor):
        self.cursor = cursor
        self.prepare()

    def prepare(self):
        """Initialize tool."""

    def on_button1_press(self):
        """Button 1 pressed callback."""

    def on_button1_release(self):
        """Button 1 released callback."""

    def on_button2_press(self):
        """Button 2 pressed callback."""

    def on_button2_release(self):
        """Button 2 released callback."""

    def on_cursor_move(self):
        """Cursor moved callback."""

    def cleanup(self):
        """Clean up before changing tool."""
