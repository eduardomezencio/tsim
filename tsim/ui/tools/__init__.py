"""Import all tools inspecting files from package."""

from importlib import import_module
import os

from tsim.ui.tools.tool import Tool

for name in filter(lambda s: (not s.startswith('_') and s.endswith('.py')
                              and s != 'tool.py'),
                   os.listdir(os.path.dirname(os.path.abspath(__file__)))):

    module_name = os.path.splitext(name)[0]
    import_module(f'.{module_name}', 'tsim.ui.tools')

TOOLS = Tool.__subclasses__()
