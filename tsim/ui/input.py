"""Input module."""

from __future__ import annotations

from typing import Iterator
import sys

import tsim.ui.panda3d as p3d

ACTIONS = ('left', 'right', 'up', 'down', 'zoom_in', 'zoom_out',
           'rot_right', 'rot_left', 'pitch_up', 'pitch_down',
           'tool_1', 'tool_2', 'tool_3')

MAPPING = {
    'w': 'up', 'a': 'left', 's': 'down', 'd': 'right',
    '+': 'zoom_in', '-': 'zoom_out',
    'q': 'rot_left', 'e': 'rot_right',
    'r': 'pitch_up', 'f': 'pitch_down',
    'mouse1': 'tool_1', 'mouse2': 'tool_2', 'mouse3': 'tool_3'
}

KEYS = {k: False for k in ACTIONS}
PRESSED = set()
RELEASED = set()

INV_MAPPING = {}


def clear():
    """Reset state of pressed/released keys."""
    PRESSED.clear()
    RELEASED.clear()


def is_down(action: str) -> bool:
    """Check if key for given action is pressed.

    Must be called at the very end of each frame.
    """
    return KEYS.get(action, False)


def pressed(action: str) -> bool:
    """Check if key for given action was just pressed."""
    return action in PRESSED


def released(action: str) -> bool:
    """Check if key for given action was just released."""
    return action in RELEASED


def keys_for(action: str) -> Iterator[str]:
    """Return keys that map to the given action."""
    yield from INV_MAPPING.get(action, ())


def init():
    """Initialize the input module."""
    _fill_inv_mapping()

    def set_key(key: str, value: bool):
        KEYS[key] = value
        if value:
            PRESSED.add(key)
        else:
            RELEASED.add(key)

    p3d.BASE.disable_mouse()

    # base.messenger.toggleVerbose()  # to print all events
    p3d.BASE.accept('escape', sys.exit)
    for suffix, is_pressed in (('', True), ('-up', False)):
        for key, action in MAPPING.items():
            p3d.BASE.accept(f'{key}{suffix}', set_key, [action, is_pressed])


def _fill_inv_mapping():
    INV_MAPPING.clear()
    for key, value in MAPPING.items():
        keyset = INV_MAPPING.get(value)
        if keyset is None:
            keyset = set()
            INV_MAPPING[value] = keyset
        keyset.add(key)
