"""Open street map related functions."""

from __future__ import annotations

from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from tsim.model.entity import Entity

BASE_URL = 'https://www.openstreetmap.org'


def get_url(entity: Entity) -> str:
    """Get openstreetmap url for given entity."""
    if type(entity).__name__ in ('Node', 'Way') and entity.xid is not None:
        return f'{BASE_URL}/{type(entity).__name__.lower()}/{entity.xid}'
    return ''
