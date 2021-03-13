"""Definition of module main funcion."""

import os
import sys
from pathlib import Path

from tsim.serialization import configure_serialization
from tsim.ui.app import App

LAST_PATH = Path.home().joinpath('.tsim', 'last.osm')


def main():
    """Tsim entry point."""
    start_app(*sys.argv[1:])


def start_app(index: str = None):
    """Tsim module main funcion."""
    configure_serialization()
    index_path = Path(index).absolute() if index else LAST_PATH
    try:
        App(str(index_path)).run()
    except SystemExit:
        pass
    register_last_index(index_path)


def register_last_index(index_path: Path):
    """Save the last index loaded as a symlink to use again later."""
    if index_path == LAST_PATH:
        return

    for index, last in ((index_path, LAST_PATH),
                       (f'{index_path}.shelf', f'{LAST_PATH}.shelf')):
        try:
            os.remove(last)
        except FileNotFoundError:
            pass
        os.symlink(index, last)
