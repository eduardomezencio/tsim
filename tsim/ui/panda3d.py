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

    P3D_BASE: ShowBase.ShowBase
    P3D_CAMERA: NodePath
    P3D_LENS: Lens
    P3D_LOADER: Loader.Loader
    P3D_MESSENGER: Messenger.Messenger
    P3D_MOUSE_WATCHER: MouseWatcher
    P3D_RENDER: NodePath
    P3D_RENDER2D: NodePath
    P3D_TASK_MGR: Task.TaskManager
    P3D_WIN: GraphicsWindow

P3D_BASE = ShowBase.ShowBase()
P3D_CAMERA = P3D_BASE.camera
P3D_LENS = P3D_BASE.cam.node().get_lens()
P3D_LOADER = P3D_BASE.loader
P3D_MESSENGER = P3D_BASE.messenger
P3D_MOUSE_WATCHER = P3D_BASE.mouseWatcherNode
P3D_RENDER = P3D_BASE.render
P3D_RENDER2D = P3D_BASE.render2d
P3D_TASK_MGR = P3D_BASE.task_mgr
P3D_WIN = P3D_BASE.win
