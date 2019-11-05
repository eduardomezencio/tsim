"""Definition of module main funcion."""

from tsim.ui.app import App


def main(index_name: str):
    """Tsim module main funcion."""
    try:
        App(index_name).run()
    except SystemExit:
        pass
