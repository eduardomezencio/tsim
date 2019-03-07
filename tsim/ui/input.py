"""Input module."""

import sys

from direct.showbase.ShowBase import ShowBase

ACTIONS = ('left', 'right', 'up', 'down', 'zoom_in', 'zoom_out',
           'rot_right', 'rot_left', 'pitch_up', 'pitch_down')

MAPPING = {
    'w': 'up', 'a': 'left', 's': 'down', 'd': 'right',
    '+': 'zoom_in', '-': 'zoom_out',
    'q': 'rot_left', 'e': 'rot_right',
    'r': 'pitch_up', 'f': 'pitch_down'
}

KEYS = {k: False for k in ACTIONS}


def is_down(action: str):
    """Check if key for given action is pressed."""
    return KEYS.get(action, False)


def init_input(base: ShowBase):
    """Initialize the input module."""

    def set_key(key: str, value: bool):
        KEYS[key] = value

    base.disable_mouse()

    # self.base.messenger.toggleVerbose()  # to print all events
    base.accept('escape', sys.exit)
    for suffix, pressed in (('', True), ('-up', False)):
        for key, action in MAPPING.items():
            base.accept(f'{key}{suffix}', set_key, [action, pressed])
