"""Definition of module main funcion."""

from tsim.model.index import INSTANCE as INDEX
from tsim.ui.app import App


def main(index_name: str):
    """Tsim module main funcion."""
    init_index(index_name)
    try:
        App().run()
    except SystemExit:
        pass


def init_index(name: str):
    """Initialize index from given index name."""
    INDEX.load(name)
    INDEX.register_updates = True
