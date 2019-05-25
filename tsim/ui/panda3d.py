"""Module to contain Panda3D globals."""

from typing import TYPE_CHECKING

from direct.showbase import ShowBase
from panda3d.core import load_prc_file

load_prc_file('config.prc')

if TYPE_CHECKING:
    # pylint: disable=ungrouped-imports
    from direct.showbase import Loader, Messenger
    from direct.task import Task
    from panda3d.core import GraphicsWindow, Lens, MouseWatcher, NodePath
    # pylint: enable=ungrouped-imports

    BASE: ShowBase.ShowBase
    CAMERA: NodePath
    LENS: Lens
    LOADER: Loader.Loader
    MESSENGER: Messenger.Messenger
    MOUSE_WATCHER: MouseWatcher
    RENDER: NodePath
    RENDER2D: NodePath
    ASPECT2D: NodePath
    PIXEL2D: NodePath
    TASK_MGR: Task.TaskManager
    WIN: GraphicsWindow

BASE = ShowBase.ShowBase()
CAMERA = BASE.camera
LENS = BASE.cam.node().get_lens()
LOADER = BASE.loader
MESSENGER = BASE.messenger
MOUSE_WATCHER = BASE.mouseWatcherNode
RENDER = BASE.render
RENDER2D = BASE.render2d
ASPECT2D = BASE.aspect2d
PIXEL2D = BASE.pixel2d
TASK_MGR = BASE.task_mgr
WIN = BASE.win
