"""Entity base class."""

from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from itertools import count
from typing import ClassVar, Dict, Tuple
import shelve

from cached_property import cached_property
from dataslots import with_slots
from rtree.index import Rtree

from tsim.model.geometry import BoundingRect
from tsim.utils import pickling


@with_slots(add_dict=True)
@dataclass
class Entity(ABC):
    """Base class for spatial entities."""

    id: int = field(init=False, default_factory=type(None))

    @cached_property
    def bounding_rect(self) -> BoundingRect:
        """Bounding rectangle cached property."""
        return self.calc_bounding_rect()

    @abstractmethod
    def calc_bounding_rect(self,
                           accumulated: BoundingRect = None) -> BoundingRect:
        """Calculate the bounding rectangle of the entity."""
        ...

    __getstate__ = pickling.getstate
    __setstate__ = pickling.setstate


class EntityIndex:
    """Index of spatial entities.

    When an entity is added to the index, it gets an unique id and is kept in
    a way than can be queried by id or by spatial coordinates.
    """

    __slots__ = ('name', 'id_count', 'entities', 'rtree')

    extension: ClassVar[str] = 'shelf'
    storage_fields: ClassVar[Tuple[str]] = ('id_count', 'entities')

    name: str
    id_count: count
    entities: Dict[int, Entity]
    rtree: Rtree

    def __init__(self, name: str):
        self.name = name
        self.id_count = count()
        self.entities = {}
        self.rtree = Rtree()

    @property
    def filename(self) -> str:
        """Name with extension added."""
        if self.name.endswith('.' + EntityIndex.extension):
            return self.name
        return '.'.join((self.name, EntityIndex.extension))

    def add(self, entity: Entity):
        """Add entity to index."""
        if entity.id is None:
            entity.id = next(self.id_count)
            self.entities[entity.id] = entity
            self.rtree.insert(entity.id, entity.bounding_rect)

    def delete(self, entity: Entity):
        """Delete entity from index."""
        assert self.entities[entity.id] is entity
        del self.entities[entity.id]
        self.rtree.delete(entity.id, entity.bounding_rect)

    def generate_rtree_from_entities(self):
        """Create empty rtree and add all entities to it."""
        self.rtree = Rtree()
        for id_, entity in self.entities.items():
            self.rtree.add(id_, entity.bounding_rect)

    def load(self):
        """Load entities from shelf."""
        with shelve.open(self.filename) as data:
            for key in EntityIndex.storage_fields:
                value = data.get(key, None)
                if value:
                    setattr(self, key, value)
        self.generate_rtree_from_entities()

    def save(self):
        """Save entities to shelf."""
        with shelve.open(self.filename) as data:
            for key in EntityIndex.storage_fields:
                data[key] = getattr(self, key)
