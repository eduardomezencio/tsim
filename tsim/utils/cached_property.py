"""Cached property implementation."""


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

    def __get__(self, obj, cls):
        if obj is None:
            return self

        value = obj.__dict__[self.func.__name__] = self.func(obj)
        if self._on_update is not None:
            self._on_update(obj)
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
