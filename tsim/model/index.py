"""Implementation and global instance of EntityIndex."""

from __future__ import annotations

from itertools import count
from typing import (Callable, ClassVar, Dict, Iterator, List, Tuple, Type,
                    Union)
import logging as log
import shelve

from rtree.index import Rtree

from tsim.model.entity import Entity
from tsim.model.geometry import Point


class EntityIndex:
    """Index of spatial entities.

    When an entity is added to the index, it gets an unique id and is kept in
    a way than can be queried by id or by spatial coordinates.
    """

    __slots__ = ('name', 'id_count', 'entities', 'rtree', 'register_updates',
                 '_updates')

    extension: ClassVar[str] = 'shelf'
    storage_fields: ClassVar[Tuple[str]] = ('id_count', 'entities')

    name: str
    id_count: count
    entities: Dict[int, Entity]
    rtree: Rtree

    def __init__(self, name: str = None):
        self.name = name
        self.id_count = count()
        self.entities = {}
        self.rtree = Rtree()
        self.register_updates = False
        self._updates = set()

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
            self.updated(entity)
            log.debug('[index] Added %s', entity)

    def delete(self, entity: Entity):
        """Delete entity from index."""
        to_remove = {entity}
        while to_remove:
            entity = to_remove.pop()
            assert self.entities[entity.id] is entity
            del self.entities[entity.id]
            self.rtree.delete(entity.id, entity.bounding_rect)
            to_remove.update(entity.on_delete() or ())
            self.updated(entity)
            log.debug('[index] Removed %s', entity)

    def updated(self, entity: Union[Entity, int]):
        """Mark entity as updated."""
        if self.register_updates:
            try:
                self._updates.add(entity.id)
            except AttributeError:
                self._updates.add(entity)

    def clear_updates(self):
        """Clear entity updates."""
        self._updates.clear()

    def consume_updates(self) -> Iterator[int]:
        """Get generator that pops and returns updates."""
        while self._updates:
            yield self._updates.pop()

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

    def get_at(self, point: Point, radius: float = 10.0,
               of_type: Type[Entity] = None,
               where: Callable[[Entity], bool] = None) -> List[Entity]:
        """Get entities at given coordinates.

        Get a list with entities within radius from given point, sorted from
        closest to farthest. If of_type is not None, will return only entities
        of the given type. If where is not None, where must be a function that
        receives an Entity and returns True or False, meaning whether the
        entity will be returned.
        """
        def get_distance(entity, point):
            result = entity.distance(point, squared=True)
            distances[entity] = result
            return result

        def distance_filter(entity):
            return get_distance(entity, point) <= radius

        def type_filter(entity):
            return isinstance(entity, of_type)

        distances = {}
        filters = [distance_filter]
        if where is not None:
            filters.append(where)
        if of_type is not None:
            filters.append(type_filter)

        return sorted(filter(lambda e: all(f(e) for f in filters),
                             map(self.entities.get,
                                 self.rtree.intersection(
                                     (point.x - radius, point.y - radius,
                                      point.x + radius, point.y + radius)))),
                      key=distances.get)


INSTANCE = EntityIndex()
