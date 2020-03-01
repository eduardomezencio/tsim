"""Camera class implementation."""

from functools import partial
from typing import Optional

from panda3d.core import NodePath, PandaNode

from tsim.ui.input import is_down
import tsim.ui.panda3d as p3d

ACCEL = 0.003
DECCEL = 0.8

CINEMATIC_ACCEL = 0.0002
CINEMATIC_DECCEL = 0.98

DEFAULT_HEIGHT = 100.0
DEFAULT_PITCH = 60.0
DEFAULT_ROTATION = 0.0
DISTANCE_MIN = 4.0
DISTANCE_MAX = 4000.0
FAR = 7000.0
FOCUS_HEIGHT = 2.0
PITCH_MIN = 0.0
PITCH_MAX = 90.0
ZOOM_RATIO = 4.0


class Camera:
    """Camera wrapper and logic implementation."""

    def __init__(self):
        p3d.LENS.set_far(FAR)

        self.focus = NodePath(PandaNode('focus'))
        self.focus.set_compass()
        self.focus.reparent_to(p3d.RENDER)
        self.focus.set_pos(0.0, 0.0, FOCUS_HEIGHT)
        self.rotator = NodePath(PandaNode('rotator'))
        self.rotator.reparent_to(self.focus)
        self.rotator.set_pos(0.0, 0.0, 0.0)
        self.following = None
        p3d.CAMERA.set_pos(0.0, 0.0, DEFAULT_HEIGHT)
        p3d.CAMERA.look_at(self.rotator)
        p3d.CAMERA.reparent_to(self.rotator)
        self.focus.set_h(DEFAULT_ROTATION)
        self.rotator.set_p(DEFAULT_PITCH)

        self.speed = [0.0, 0.0]
        self.rot_speed = [0.0, 0.0]
        self.zoom_speed = 0.0
        self._cinematic = None
        self.set_cinematic_mode(False)

        def _toggle_cinematic():
            self.set_cinematic_mode(not self._cinematic)

        def _zoom(amount):
            self.zoom_speed += amount * self._zoom_accel

        p3d.BASE.accept('f8', _toggle_cinematic)
        p3d.BASE.accept('wheel_up', partial(_zoom, -10.0))
        p3d.BASE.accept('wheel_down', partial(_zoom, 10.0))

    def set_cinematic_mode(self, cinematic: bool = True):
        """Activate or deactivate camera cinematic mode."""
        if cinematic != self._cinematic:
            if cinematic:
                self._accel = CINEMATIC_ACCEL
                self._deccel = CINEMATIC_DECCEL
            else:
                self._accel = ACCEL
                self._deccel = DECCEL
            self._rot_accel = self._accel * 100.0
            self._zoom_accel = self._accel ** 0.75 / 5.0
            self._cinematic = cinematic

    def update(self):
        """Update method to run every frame."""
        self._translation_movement()
        self._rotation_movement()
        self._zoom_movement()

    def _translation_movement(self):
        """Update camera x and y movement."""
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
            if self.following is not None:
                self.unfollow()
            scale = (p3d.CAMERA.get_z() + 10.0) * self._accel
            self.speed[0] += dx * scale
            self.speed[1] += dy * scale
        if any(self.speed):
            self.focus.set_pos(self.focus, *self.speed, 0.0)
            self.speed[:] = map(
                lambda x: x * self._deccel if abs(x) > 0.001 else 0.0,
                self.speed)

    def _rotation_movement(self):
        """Update camera heading and pitch."""
        if is_down('rot_right'):
            self.rot_speed[0] += self._rot_accel
        elif is_down('rot_left'):
            self.rot_speed[0] -= self._rot_accel
        if is_down('pitch_down'):
            self.rot_speed[1] += self._rot_accel
        elif is_down('pitch_up'):
            self.rot_speed[1] -= self._rot_accel

        if any(self.rot_speed):
            self.focus.set_h(self.focus.get_h() + self.rot_speed[0])
            self.rotator.set_p(
                min(max(PITCH_MIN, self.rotator.get_p() + self.rot_speed[1]),
                    PITCH_MAX))
            self.rot_speed[:] = map(
                lambda x: x * self._deccel if abs(x) > 0.001 else 0.0,
                self.rot_speed)

    def _zoom_movement(self):
        """Update camera zoom."""
        if is_down('zoom_in'):
            self.zoom_speed -= self._zoom_accel
        elif is_down('zoom_out'):
            self.zoom_speed += self._zoom_accel

        if self.zoom_speed != 0.0:
            speed = self.zoom_speed
            speed, negative = min(abs(speed), 0.02), speed < 0.0
            cam_z = p3d.CAMERA.get_z()
            if negative:
                p3d.CAMERA.set_z(max(
                    (cam_z + 1.0) * (1.0 - ZOOM_RATIO * speed) - 1.0,
                    DISTANCE_MIN))
            else:
                p3d.CAMERA.set_z(min(
                    (cam_z + 1.0) * (1.0 + ZOOM_RATIO * speed) - 1.0,
                    DISTANCE_MAX))
            new_speed = self.zoom_speed * self._deccel
            self.zoom_speed = new_speed if abs(new_speed) > 0.00001 else 0.0

    def follow(self, node_path: NodePath):
        """Start following given node path."""
        self.following = node_path
        self.focus.reparent_to(node_path)
        self.focus.set_pos(0.0, 0.0, FOCUS_HEIGHT)

    def unfollow(self, node_path: Optional[NodePath] = None):
        """Stop following given node path.

        Stops following given node path or any node path if not specified.
        """
        if node_path is None or self.focus.parent == node_path:
            self.following = None
            self.focus.wrt_reparent_to(p3d.RENDER)
            self.focus.set_z(FOCUS_HEIGHT)
