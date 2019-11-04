"""Implementation and global instance of EntityIndex."""

from __future__ import annotations

from itertools import count
from typing import (Callable, ClassVar, Dict, Iterator, List, Optional, Set,
                    Tuple, Type, Union)
import logging as log
import shelve

from rtree.index import Rtree

from tsim.model.geometry import BoundingRect, Point, point_in_polygon
from tsim.model.network.entity import Entity
from tsim.model.network.path import PathMap


class EntityIndex:
    """Index of spatial entities.

    When an entity is added to the index, it gets an unique id and is kept in
    a way than can be queried by id or by spatial coordinates.
    """

    __slots__ = ('name', 'id_count', 'entities', 'bounding_rects', 'rtree',
                 'path_map', 'register_updates', '_updates')

    extension: ClassVar[str] = 'shelf'
    storage_fields: ClassVar[Tuple[str]] = ('id_count', 'entities')

    name: str
    id_count: count
    entities: Dict[int, Entity]
    bounding_rects: Dict[int, BoundingRect]
    rtree: Rtree
    path_map: PathMap
    register_updates: bool
    _updates: Set[int]

    def __init__(self, name: Optional[str] = None):
        self.reset(name)

    @property
    def filename(self) -> str:
        """Name with extension added."""
        if self.name.endswith(f'.{EntityIndex.extension}'):
            return self.name
        return f'{self.name}.{EntityIndex.extension}'

    def reset(self, name: Optional[str] = None):
        """Reset index and set name."""
        self.name = name
        self.id_count = count()
        self.entities = {}
        self.bounding_rects = {}
        self.rtree = Rtree()
        self.path_map = PathMap()
        self.register_updates = False
        self._updates = set()

    def add(self, entity: Entity):
        """Add entity to index."""
        if entity.id is None:
            entity.id = next(self.id_count)
            self.entities[entity.id] = entity
            self.bounding_rects[entity.id] = entity.bounding_rect
            self.rtree.insert(entity.id, entity.bounding_rect)
            self.updated(entity)
            self.rebuild_path_map()
            log.debug('[index] Added %s', entity)

    def delete(self, entity: Entity):
        """Delete entity from index."""
        to_remove = {entity}
        while to_remove:
            entity = to_remove.pop()
            assert self.entities[entity.id] is entity
            self.rtree.delete(entity.id, self.bounding_rects[entity.id])
            del self.entities[entity.id]
            del self.bounding_rects[entity.id]
            delete_result = entity.on_delete()
            to_remove.update(delete_result.cascade)
            for updated in delete_result.updated:
                self.updated(updated)
            self.updated(entity)
            self.rebuild_path_map()
            log.debug('[index] Removed %s', entity)

    def update_bounding_rect(self, entity: Entity,
                             new_rect: Optional[BoundingRect] = None):
        """Change the bounding rectangle of an entity.

        Update the bounding rect to `entity.bounding_rect` or to `new_rect` if
        it's not None.
        """
        assert self.entities[entity.id] is entity

        if new_rect is None:
            new_rect = entity.bounding_rect

        old_rect = self.bounding_rects.get(entity.id, None)
        if old_rect is None or old_rect == new_rect:
            return

        self.rtree.delete(entity.id, old_rect)
        self.bounding_rects[entity.id] = new_rect
        self.rtree.insert(entity.id, new_rect)

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
        self.bounding_rects = {id_: e.bounding_rect
                               for id_, e in self.entities.items()}
        self.rtree = Rtree((id_, self.bounding_rects[id_], None)
                           for id_ in self.entities)

    def load(self, name: Optional[str] = None):
        """Load entities from shelf.

        Load enities using the this index name. If a name is passed as
        argument, will set the index name before loading.
        """
        if name is not None:
            self.name = name
        with shelve.open(self.filename) as data:
            for key in EntityIndex.storage_fields:
                value = data.get(key, None)
                if value:
                    setattr(self, key, value)
        self.generate_rtree_from_entities()
        self.rebuild_path_map()

    def save(self):
        """Save entities to shelf."""
        with shelve.open(self.filename) as data:
            for key in EntityIndex.storage_fields:
                data[key] = getattr(self, key)

    def get_all(self, of_type: Type[Entity] = None,
                where: Callable[[Entity], bool] = None) -> Iterator[Entity]:
        """Get all entities with optional filters."""
        def type_filter(entity):
            return isinstance(entity, of_type)

        filters = []
        if of_type is not None:
            filters.append(type_filter)
        if where is not None:
            filters.append(where)

        yield from filter(lambda e: all(f(e) for f in filters),
                          self.entities.values())

    def get_at(self, point: Point, of_type: Type[Entity] = None,
               where: Callable[[Entity], bool] = None) -> List[Entity]:
        """Get entities at given coordinates.

        Get a list with entities intersecting the given point. If of_type is
        not None, will return only entities of the given type. If where is not
        None, where must be a function that receives an Entity and returns True
        or False, meaning whether the entity will be returned.
        """
        def polygon_filter(entity: Entity) -> bool:
            return point_in_polygon(point, entity.polygon)

        def type_filter(entity: Entity) -> bool:
            return isinstance(entity, of_type)

        filters = [polygon_filter]
        if of_type is not None:
            filters.append(type_filter)
        if where is not None:
            filters.append(where)

        return list(filter(
            lambda e: all(f(e) for f in filters),
            map(self.entities.get,
                self.rtree.intersection(point.bounding_rect))))

    def rebuild_path_map(self):
        """Rebuild the path map, invalidating the old map."""
        self.path_map = PathMap()


INSTANCE = EntityIndex()
