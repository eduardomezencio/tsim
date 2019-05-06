"""Input module."""

import sys

from tsim.ui.panda3d import P3D_BASE

ACTIONS = ('left', 'right', 'up', 'down', 'zoom_in', 'zoom_out',
           'rot_right', 'rot_left', 'pitch_up', 'pitch_down')

MAPPING = {
    'w': 'up', 'a': 'left', 's': 'down', 'd': 'right',
    '+': 'zoom_in', '-': 'zoom_out',
    'q': 'rot_left', 'e': 'rot_right',
    'r': 'pitch_up', 'f': 'pitch_down'
}

KEYS = {k: False for k in ACTIONS}
PRESSED = set()
RELEASED = set()


def clear():
    """Reset state of pressed/released keys."""
    PRESSED.clear()
    RELEASED.clear()


def is_down(action: str):
    """Check if key for given action is pressed.

    Must be called at the very end of each frame.
    """
    return KEYS.get(action, False)


def pressed(action: str):
    """Check if key for given action was just pressed."""
    return action in PRESSED


def released(action: str):
    """Check if key for given action was just released."""
    return action in RELEASED


def init():
    """Initialize the input module."""
    def set_key(key: str, value: bool):
        KEYS[key] = value
        if value:
            PRESSED.add(key)
        else:
            RELEASED.add(key)

    P3D_BASE.disable_mouse()

    # base.messenger.toggleVerbose()  # to print all events
    P3D_BASE.accept('escape', sys.exit)
    for suffix, is_pressed in (('', True), ('-up', False)):
        for key, action in MAPPING.items():
            P3D_BASE.accept(f'{key}{suffix}', set_key, [action, is_pressed])
