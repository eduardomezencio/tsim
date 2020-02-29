"""Definition of module main funcion."""

import os

from tsim.serialization.config import configure_serialization
from tsim.ui.app import App

LAST_INDEX_NAME = '_last.osm'


def main(index_name: str = LAST_INDEX_NAME):
    """Tsim module main funcion."""
    configure_serialization()
    register_last_index(index_name)
    try:
        App(index_name).run()
    except SystemExit:
        pass


def register_last_index(index_name: str):
    """Save the last index loaded as a symlink to use again later."""
    if index_name == LAST_INDEX_NAME:
        return

    for name, last in ((index_name, LAST_INDEX_NAME),
                       (f'{index_name}.shelf', f'{LAST_INDEX_NAME}.shelf')):
        try:
            os.symlink(name, last)
        except FileExistsError:
            os.remove(last)
            os.symlink(name, last)
