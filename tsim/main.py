"""Definition of module main funcion."""

import sys

from tsim.model.index import INSTANCE as INDEX
from tsim.ui.app import App


def main():
    """Tsim module main funcion."""
    INDEX.name = sys.argv[1]
    INDEX.load()
    try:
        App().run()
    except SystemExit:
        pass
