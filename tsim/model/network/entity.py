"""NetworkEntity base class."""

from __future__ import annotations

from abc import abstractmethod, abstractproperty
from dataclasses import dataclass, field
from typing import Iterable

from dataslots import with_slots

from tsim.model.entity import Entity
from tsim.model.geometry import BoundingRect, Point, Polygon
from tsim.utils import osm as xurl_provider
from tsim.utils.cached_property import add_cached, cached_property, clear_cache
import tsim.model.index as Index


@add_cached
@with_slots
@dataclass(eq=False)
class NetworkEntity(Entity):
    """Base class for network entities.

    Network entities are static entities, that don't change position
    frequently, so they provide geometrical imformation to be stored on a
    spatial index for fast spatial queries.
    """

    def __post_init__(self):
        super(NetworkEntity, self).__post_init__()
        Index.INSTANCE.add_static(self)
        Index.INSTANCE.rebuild_path_map()

    xid: int = field(init=False, default_factory=type(None))

    @cached_property
    def bounding_rect(self) -> BoundingRect:
        """Get the bounding rectangle for the entity."""
        return self.calc_bounding_rect()

    @bounding_rect.on_update
    def update_index_bounding_rect(self):
        """Update the index with the entity's current bounding rectangle."""
        Index.INSTANCE.update_bounding_rect(self)

    @property
    def neighbors(self) -> Iterable[NetworkEntity]:
        """Get iterable with entities directly connected to this entity."""
        return ()

    @property
    def xurl(self) -> str:
        """Get external url for the entity."""
        return xurl_provider.get_url(self)

    @abstractproperty
    def polygon(self) -> Polygon:
        """Get polygon of the entity."""

    @abstractmethod
    def calc_bounding_rect(self,
                           accumulated: BoundingRect = None) -> BoundingRect:
        """Calculate the bounding rectangle of the entity."""

    @abstractmethod
    def distance(self, point: Point, squared: bool = False) -> float:
        """Calculate smallest distance from the entity to a point."""

    def clear_cache(self, clear_neighbors: bool = False):
        """Delete cached properties listed in class var `_cached`."""
        clear_cache(self)
        if clear_neighbors:
            for neighbor in self.neighbors:
                clear_cache(neighbor)
