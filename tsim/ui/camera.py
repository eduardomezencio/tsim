"""Camera class implementation."""

from panda3d.core import NodePath, PandaNode

from tsim.ui.input import is_down
from tsim.ui.panda3d import P3D_CAMERA, P3D_LENS, P3D_RENDER


class Camera:
    """Camera wrapper and logic implementation."""

    DEFAULT_HEIGHT = 100.0
    DEFAULT_PITCH = 60.0
    DEFAULT_ROTATION = 30.0
    DISTANCE_MIN = 1.0
    DISTANCE_MAX = 4000.0
    FAR = 7000.0
    FOCUS_HEIGHT = 2.0
    PITCH_MIN = 0.0
    PITCH_MAX = 90.0
    ROTATION_SPEED = 1.0
    SPEED = 0.02
    ZOOM_RATIO = 1.04

    def __init__(self):
        P3D_LENS.set_far(Camera.FAR)

        self.focus = NodePath(PandaNode('focus'))
        self.focus.reparent_to(P3D_RENDER)
        self.focus.set_pos(0.0, 0.0, Camera.FOCUS_HEIGHT)
        self.rotator = NodePath(PandaNode('rotator'))
        self.rotator.reparent_to(self.focus)
        self.rotator.set_pos(0.0, 0.0, 0.0)
        P3D_CAMERA.set_pos(0.0, 0.0, Camera.DEFAULT_HEIGHT)
        P3D_CAMERA.look_at(self.rotator)
        P3D_CAMERA.reparent_to(self.rotator)
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
            scale = (P3D_CAMERA.get_z() + 10.0) * Camera.SPEED
            self.focus.set_pos(self.focus, dx * scale, dy * scale, 0.0)
        if is_down('zoom_in'):
            P3D_CAMERA.set_pos(
                0.0, 0.0, max(P3D_CAMERA.get_z() / Camera.ZOOM_RATIO - 1.0,
                              Camera.DISTANCE_MIN))
        elif is_down('zoom_out'):
            P3D_CAMERA.set_pos(
                0.0, 0.0, min(P3D_CAMERA.get_z() * Camera.ZOOM_RATIO + 1.0,
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
