"""Implementation of Sky class."""

from __future__ import annotations

from typing import Tuple

from panda3d.core import (AmbientLight, ConfigVariableBool, DirectionalLight,
                          Fog, NodePath)

from tsim.ui.camera import Camera
import tsim.ui.panda3d as p3d


class Sky:
    """Encapsulates global illumination and fog."""

    ambient: AmbientLight
    ambient_np: NodePath
    sun: DirectionalLight
    sun_np: NodePath
    fog: Fog

    def __init__(self, parent: NodePath, camera: Camera):
        self._camera = camera
        self._parent = parent
        self.hours = 0.0
        self._init_sun()
        self._init_ambient()
        self._init_fog()

    def update(self):
        """Update lights."""
        self.sun_np.set_pos(self._camera.focus, 0.0, 0.0, 0.0)
        film = p3d.CAMERA.get_z() + self._camera.rotator.get_p() ** 2.0 * 0.01
        self.sun.get_lens().set_film_size(film, film)
        self.sun.get_lens().set_near_far(-film, film)

    def set_time(self, hours: float):
        """Update lighting according to time in range 0.0 <= t < 24.0."""
        hours %= 24
        self.hours = hours
        self.sun_np.set_p(360.0 * (hours + 6.0) / 24.0)
        color = calculate_sky_color(hours)
        self.sun.set_color((5.0 * color[0], 5.0 * color[1],
                            5.0 * color[2], color[3]))
        self.fog.set_color(color)
        p3d.WIN.set_clear_color(color)

    def _init_sun(self):
        """Initialize sun light."""
        self.sun = DirectionalLight('sun')
        self.sun.set_shadow_caster(True, 1024, 1024)
        self.sun.get_lens().set_film_size(1024, 1024)
        self.sun.get_lens().set_near_far(-1024, 1024)
        self.sun.set_camera_mask(0x00010000)
        self.sun.set_color((1.0, 1.0, 1.0, 1.0))
        # self.sun.show_frustum()
        self.sun_np = self._parent.attach_new_node(self.sun)
        self.sun_np.set_hpr(270.0, 0.0, 0.0)
        self.sun_np.set_pos(self._camera.focus, 0.0, 0.0, 512.0)
        self._parent.set_light(self.sun_np)

    def _init_ambient(self):
        """Initialize ambient light."""
        self.ambient = AmbientLight('ambient')
        self.ambient.set_color((0.4, 0.4, 0.6, 1.0))
        self.ambient_np = self._parent.attach_new_node(self.ambient)
        self._parent.set_light(self.ambient_np)

    def _init_fog(self):
        """Initialize distance fog."""
        self.fog = Fog('fog')
        self.fog.set_linear_range(2000.0, 7000.0)
        if not ConfigVariableBool('use-shaders', False):
            self._parent.set_fog(self.fog)


COLORS = ((0.1, 0.1, 0.2, 1.0),
          (1.0, 0.7, 0.6, 1.0),
          (0.8, 0.8, 1.0, 1.0))

Color = Tuple[float, float, float, float]


def calculate_sky_color(hours: float):
    """Calculate sky color based on hour."""
    if hours < 12.0:
        if hours < 6.0:
            result = (COLORS[0] if hours < 5.0 else
                      interpolate_colors(COLORS[0], COLORS[1], hours - 5.0))
        else:
            result = (interpolate_colors(COLORS[1], COLORS[2], hours - 6.0)
                      if hours < 7.0 else COLORS[2])
    else:
        if hours < 18.0:
            result = (COLORS[2] if hours < 17.0 else
                      interpolate_colors(COLORS[2], COLORS[1], hours - 17.0))
        else:
            result = (interpolate_colors(COLORS[1], COLORS[0], hours - 18.0)
                      if hours < 19.0 else COLORS[0])
    return result


def interpolate_colors(color1: Color, color2: Color, t: float) -> Color:
    """Interpolate two colors, with 0.0 <= t <= 1.0."""
    return tuple(a + t * (b - a) for a, b in zip(color1, color2))
