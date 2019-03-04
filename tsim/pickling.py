"""Functions for use with pickle or shelve."""

from itertools import chain

from cached_property import cached_property


def getstate(self):
    """Get state for use with pickle/shelve.

    Get state considering slots and removing cached properties.
    """
    cls = type(self)
    dict_copy = self.__dict__.copy()
    for key in self.__dict__:
        if hasattr(cls, key):
            if isinstance(getattr(cls, key), cached_property):
                del dict_copy[key]
    slots = chain.from_iterable(
        getattr(c, '__slots__', ()) for c in cls.mro())
    slots_dict = {s: getattr(self, s) for s in slots if s != '__dict__'}
    return dict_copy, slots_dict


def setstate(self, state):
    """Restore state for use with pickle/shelve.

    Restore state considering slots.
    """
    dict_copy, slots = state
    self.__dict__.update(dict_copy)
    for key, value in slots.items():
        setattr(self, key, value)
