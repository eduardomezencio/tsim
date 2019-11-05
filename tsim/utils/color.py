"""Color related functions.

HSV colors have 0.0 <= hue <= 360.0 and other components 0.0 <= c <= 1.0. RGB
has all components 0.0 <= c <= 1.0. Both HSV and RGB colors have alpha as the
fouth component.
"""

from __future__ import annotations

from typing import Iterable, List


def hsv_to_rgb(hue: float, saturation: float, value: float,
               alpha: float = 1.0) -> List[float]:
    """Convert color from HSV to RGB."""
    chroma = value * saturation
    hue_ = (hue % 360) / 60
    x = chroma * (1 - abs((6 * hue_) % 2 - 1))

    if hue_ <= 1:
        rgb = (chroma, x, 0.0)
    elif hue_ <= 2:
        rgb = (x, chroma, 0.0)
    elif hue_ <= 3:
        rgb = (0.0, chroma, x)
    elif hue_ <= 4:
        rgb = (0.0, x, chroma)
    elif hue_ <= 5:
        rgb = (x, 0.0, chroma)
    elif hue_ <= 6:
        rgb = (chroma, 0.0, x)
    else:
        rgb = (0.0, 0.0, 0.0)

    return [c + value - chroma for c in rgb] + [alpha]


def interpolate_hsv(start: Iterable[float], end: Iterable[float],
                    param: float) -> List[float]:
    """Interpolate two HSV colors with given parameter.

    The parameter is a value between 0.0, meaning the pure start color, and
    1.0, meaning the pure end color.
    """
    result = [s + param * (e - s)
              for i, (s, e) in enumerate(zip(start, end))]
    result[0] = result[0] % 360
    return result


def interpolate_rgb(start: Iterable[float], end: Iterable[float],
                    param: float) -> List[float]:
    """Interpolate two RGB colors with given parameter.

    The parameter is a value between 0.0, meaning the pure start color, and
    1.0, meaning the pure end color.
    """
    result = [s + param * (e - s)
              for i, (s, e) in enumerate(zip(start, end))]
    return result
