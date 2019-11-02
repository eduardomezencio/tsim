"""A complement to itertools."""

from __future__ import annotations

from collections import deque
from itertools import chain
from typing import Iterable, Iterator, Tuple, TypeVar

_NULL = object()
T = TypeVar('T')  # pylint: disable=invalid-name


def drop_duplicates(iterable: Iterable[T]) -> Iterator[T]:
    """Create an iterator that drops consecutive duplicate values.

    Examples::
        drop_duplicates([1, 1, 2, 3, 3, 3, 2]) --> 1 2 3 2
    """
    iterator = iter(iterable)
    last = next(iterator, _NULL)
    if last is not _NULL:
        yield last
    for item in iterator:
        if item != last:
            yield item
        last = item


def window_iter(iterable: Iterable[T], size: int = 2,
                extend: bool = False) -> Iterator[Tuple[T]]:
    """Create a moving window iterator of given size.

    Each element returned by this iterator is a tuple of length size (default
    2). The next item is equivalent to dropping the first item of the last
    returned tuple and appending the next item of the iterable.

    If extend is True (default is False), the `size - 1` first items will
    repeat at the end.

    Examples:
        window_iter([1, 2, 3, 4, 5]) --> (1, 2) (2, 3) (3, 4) (4, 5)
        window_iter([1, 2, 3, 4], size=3) --> (1, 2, 3) (2, 3, 4)
        window_iter([1, 2, 3], extend=True) --> (1, 2) (2, 3) (3, 1)

    """
    window = deque(maxlen=size)
    iterator = iter(iterable)
    extension = []

    for item in iterator:
        window.append(item)
        if len(window) >= size:
            yield tuple(window)
            break
        if extend:
            extension.append(item)

    for item in chain(iterator, extension):
        window.append(item)
        yield tuple(window)
