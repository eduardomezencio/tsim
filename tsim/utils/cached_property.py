"""Cached property implementation."""

from functools import partial


class CachedProperty:
    """A cached property.

    This property is computed once and stored on the instance's `__dict__` with
    the function return value. Deleting this attribute resets the cache, so
    that the property is computed again next time it is accessed.
    """

    def __init__(self, func):
        self.__doc__ = getattr(func, "__doc__")
        self.func = func
        self._on_update = None

    def __get__(self, instance, owner=None):
        if instance is None:
            return self

        value = instance.__dict__[self.func.__name__] = self.func(instance)
        if self._on_update is not None:
            self._on_update(instance)
        return value

    def on_update(self, func):
        """Decorate a function with this to set as post update callback.

        The decorated function is returned unchanged. The callback is called
        only when the property is accessed for the first time, or for the first
        time after a reset.

        Example::
            @cached_property
            def rect(self):
                return Rect(self)

            @rect.on_update
            def on_rect_update(self):
                self.parent.update_rect(self)

        """
        self._on_update = func
        return func


def cached_property(func):
    """Get a cached property from `func` (CachedProperty wrapper)."""
    return CachedProperty(func)


def add_cached(cls=None, attr_name='_cached'):
    """Add attribute to the class with all cached property names."""
    if cls is None:
        return partial(add_cached, attr_name=attr_name)
    setattr(cls, attr_name,
            tuple(a for a in dir(cls) if not a.startswith('__')
                  and isinstance(getattr(cls, a), CachedProperty)))
    return cls


def clear_cache(instance, attr_name='_cached'):
    """Delete all cached values of given instance.

    Assumes the name of all cached properties are kept in the attribute with
    name `attr_name`. This attribute can be filled by decorating the class with
    `@add_cached`.
    """
    for key in getattr(type(instance), attr_name, ()):
        instance.__dict__.pop(key, None)
