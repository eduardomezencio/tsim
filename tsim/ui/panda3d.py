"""Module to contain Panda3D globals."""

from __future__ import annotations

from direct.showbase import Loader, Messenger, ShowBase
from direct.task import Task
from panda3d.core import (load_prc_file, Camera, GraphicsWindow, Lens,
                          MouseWatcher, NodePath, TextFont)
from pkg_resources import resource_filename

# pylint: disable=invalid-name
base: ShowBase.ShowBase
cam: NodePath
cam_node: Camera
camera: NodePath
lens: Lens
loader: Loader.Loader
messenger: Messenger.Messenger
mouse_watcher: MouseWatcher
render: NodePath
render2d: NodePath
aspect2d: NodePath
pixel2d: NodePath
task_mgr: Task.TaskManager
win: GraphicsWindow
ttf: TextFont


def init():
    """Initialize ShowBase and Panda3D globals."""
    # pylint: disable=global-statement
    global base, cam, cam_node, camera, lens, loader, messenger, \
           mouse_watcher, render, render2d, aspect2d, pixel2d, task_mgr, \
           win, ttf

    load_prc_file(resource_filename('tsim', 'data/config.prc'))

    base = ShowBase.ShowBase()
    cam = base.cam
    cam_node = base.camNode
    camera = base.camera
    lens = base.cam.node().get_lens()
    loader = base.loader
    messenger = base.messenger
    mouse_watcher = base.mouseWatcherNode
    render = base.render
    render2d = base.render2d
    aspect2d = base.aspect2d
    pixel2d = base.pixel2d
    task_mgr = base.task_mgr
    win = base.win
    ttf = loader.load_font('cmtt12.egg')
