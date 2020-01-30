"""`MapToValue` implementation."""

from typing import Any


class MapToValue:
    """Fake dict that maps every key to the same value."""

    value: Any

    def __init__(self, value: Any):
        self.value = value

    def __getitem__(self, _):
        return self.value
