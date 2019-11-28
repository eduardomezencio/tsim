"""Definition of module main funcion."""

from tsim.serialization.config import configure_serialization
from tsim.ui.app import App


def main(index_name: str):
    """Tsim module main funcion."""
    configure_serialization()
    try:
        App(index_name).run()
    except SystemExit:
        pass
