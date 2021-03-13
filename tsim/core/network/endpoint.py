"""Endpoint class."""

from __future__ import annotations

from enum import Enum


class Endpoint(Enum):
    """Endpoints of a Way."""

    START = 0
    END = 1

    @property
    def other(self) -> Endpoint:
        """Get the opposite endpoint."""
        return Endpoint.END if self is Endpoint.START else Endpoint.START
