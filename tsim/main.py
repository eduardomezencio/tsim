"""Definition of module main funcion."""

import sys

from tsim.model.entity import EntityIndex, EntityRef
from tsim.ui.app import App


def main():
    """Tsim module main funcion."""
    index = EntityIndex(sys.argv[1])
    index.load()
    EntityRef.index = index
    try:
        App(index).run()
    except SystemExit:
        pass
