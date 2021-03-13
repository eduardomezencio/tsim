"""Global serialization configuration."""

from importlib import import_module
import os


def configure_serialization():
    """Configure serialization for all classes in folder."""
    for name in filter(
            lambda s: not s.startswith('_') and s.endswith('.py'),
            os.listdir(os.path.dirname(os.path.abspath(__file__)))):

        module_name = os.path.splitext(name)[0]
        module = import_module(f'.{module_name}', 'tsim.serialization')
        module.configure()
