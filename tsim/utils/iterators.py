"""A complement to itertools."""

from __future__ import annotations

from collections import deque
from typing import Iterable, Tuple, TypeVar

T = TypeVar('T')  # pylint: disable=invalid-name


def window_iter(iterable: Iterable[T], size: int = 2) -> Iterable[Tuple[T]]:
    """Create a moving window iterator of given size.

    Each element returned by this iterator is a tuple of length size (default
    2). The next item is equivalent to dropping the first item of the last
    returned tuple and appending the next item of the iterable.

    Examples:
        window_iter([1, 2, 3, 4, 5]) --> (1, 2) (2, 3) (3, 4) (4, 5)
        window_iter([1, 2, 3, 4], size=3) --> (1, 2, 3) (2, 3, 4)

    """
    window = deque(maxlen=size)
    iterator = iter(iterable)

    for item in iterator:
        window.append(item)
        if len(window) >= size:
            yield tuple(window)
            break

    for item in iterator:
        window.append(item)
        yield tuple(window)
