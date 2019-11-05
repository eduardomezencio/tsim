"""Functions for use with pickle or shelve."""

from itertools import chain


def getstate(self, add_dict=True, add_slots=True):
    """Get state for use with pickle/shelve.

    Get state considering slots.
    """
    dict_copy, slots_dict = None, None

    if add_dict:
        dict_copy = self.__dict__.copy()

    if add_slots:
        slots = chain.from_iterable(getattr(c, '__slots__', ())
                                    for c in type(self).mro())
        slots_dict = {s: getattr(self, s) for s in slots
                      if s not in ('__dict__', '__weakref__')}

    return dict_copy, slots_dict


def setstate(self, state, add_dict=True, add_slots=True):
    """Restore state for use with pickle/shelve.

    Restore state considering slots. Uses `object.__setattr__` to work with
    frozen classes.
    """
    dict_copy, slots = state

    if add_dict:
        self.__dict__.update(dict_copy)

    if add_slots:
        for key, value in slots.items():
            object.__setattr__(self, key, value)
