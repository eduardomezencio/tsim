"""Definition of module main funcion."""

import sys

from tsim.model.index import INSTANCE as INDEX
from tsim.ui.app import App


def main():
    """Tsim module main funcion."""
    init_index(sys.argv[1])
    try:
        App().run()
    except SystemExit:
        pass


def init_index(name: str):
    """Initialize index from given index name."""
    INDEX.name = name
    INDEX.load()
    INDEX.register_updates = True
