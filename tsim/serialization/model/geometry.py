"""Geometry serialization configuration."""

from functools import partialmethod

from tsim.model.geometry import Vector
from tsim.utils import pickling


def configure():
    """Configure serialization for geometry related classes."""
    Vector.__getstate__ = partialmethod(pickling.getstate, add_dict=False)
    Vector.__setstate__ = partialmethod(pickling.setstate, add_dict=False)
