"""Module to contain Panda3D globals."""

from __future__ import annotations

# Importing bezier to get around a bug causing a crash when bezier is imported
# after ShowBase is instantiated. Remove this import when the bug is fixed.
import bezier  # noqa  # pylint: disable=unused-import

from direct.showbase import Loader, Messenger, ShowBase
from direct.task import Task
from panda3d.core import (load_prc_file, Camera, GraphicsWindow, Lens,
                          MouseWatcher, NodePath)

load_prc_file('config.prc')

BASE: ShowBase.ShowBase
CAM: NodePath
CAM_NODE: Camera
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
CAM = BASE.cam
CAM_NODE = BASE.camNode
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
