"""Camera class implementation."""

from direct.showbase.ShowBase import ShowBase
from panda3d.core import NodePath, PandaNode

from tsim.ui.input import is_down


class Camera:
    """Camera wrapper and logic implementation."""

    DEFAULT_HEIGHT = 100.0
    DEFAULT_PITCH = 60.0
    DEFAULT_ROTATION = 30.0
    DISTANCE_MIN = 5.0
    DISTANCE_MAX = 2000.0
    FOCUS_HEIGHT = 2.0
    PITCH_MIN = 0.0
    PITCH_MAX = 90.0
    ROTATION_SPEED = 1.0
    SPEED = 0.02
    ZOOM_RATIO = 1.04

    def __init__(self, base: ShowBase):
        self.base = base
        self.camera = self.base.camera

        self.focus = NodePath(PandaNode('focus'))
        self.focus.reparent_to(self.base.render)
        self.focus.set_pos(0.0, 0.0, Camera.FOCUS_HEIGHT)
        self.rotator = NodePath(PandaNode('rotator'))
        self.rotator.reparent_to(self.focus)
        self.rotator.set_pos(0.0, 0.0, 0.0)
        self.camera.set_pos(0.0, 0.0, Camera.DEFAULT_HEIGHT)
        self.camera.look_at(self.rotator)
        self.camera.reparent_to(self.rotator)
        self.focus.set_h(Camera.DEFAULT_ROTATION)
        self.rotator.set_p(Camera.DEFAULT_PITCH)

    def update(self):
        """Update method to run every frame."""
        move, dx, dy = False, 0.0, 0.0
        if is_down('left'):
            move, dx = True, -1.0
        elif is_down('right'):
            move, dx = True, 1.0
        if is_down('down'):
            move, dy = True, -1.0
        elif is_down('up'):
            move, dy = True, 1.0
        if move:
            scale = self.camera.get_z() * Camera.SPEED
            self.focus.set_pos(self.focus, dx * scale, dy * scale, 0.0)
        if is_down('zoom_in'):
            self.camera.set_pos(
                0.0, 0.0, max(self.camera.get_z() / Camera.ZOOM_RATIO,
                              Camera.DISTANCE_MIN))
        elif is_down('zoom_out'):
            self.camera.set_pos(
                0.0, 0.0, min(self.camera.get_z() * Camera.ZOOM_RATIO,
                              Camera.DISTANCE_MAX))

        if is_down('rot_right'):
            self.focus.set_h(self.focus, Camera.ROTATION_SPEED)
        elif is_down('rot_left'):
            self.focus.set_h(self.focus, -Camera.ROTATION_SPEED)

        if is_down('pitch_down'):
            self.rotator.set_p(
                min(self.rotator.get_p() + Camera.ROTATION_SPEED,
                    Camera.PITCH_MAX))
        elif is_down('pitch_up'):
            self.rotator.set_p(
                max(self.rotator.get_p() - Camera.ROTATION_SPEED,
                    Camera.PITCH_MIN))
