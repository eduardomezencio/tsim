"""Open street map related functions."""

from __future__ import annotations

from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from tsim.model.entity import Entity

BASE_URL = 'https://www.openstreetmap.org'


def get_url(entity: Entity) -> str:
    """Get openstreetmap url for given entity."""
    if not type(entity).__name__ in ('Node', 'Way'):
        return f'No url for {type(entity).__name__}'

    if entity.xid is None:
        return f'No xid in this {type(entity).__name__}'

    if isinstance(entity.xid, int) and entity.xid > 0:
        return f'{BASE_URL}/{type(entity).__name__.lower()}/{entity.xid}'
    return f'Invalid xid for url: {entity.xid}'
