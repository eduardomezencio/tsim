"""Implementation and global instance of EntityIndex."""

from __future__ import annotations

from itertools import count
from typing import ClassVar, Dict, Tuple, TYPE_CHECKING
import shelve

from rtree.index import Rtree

from tsim.model.geometry import Point

if TYPE_CHECKING:
    from tsim.model.entity import Entity


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

    def __init__(self, name: str = None):
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
        to_remove = {entity}
        while to_remove:
            entity = to_remove.pop()
            assert self.entities[entity.id] is entity
            del self.entities[entity.id]
            self.rtree.delete(entity.id, entity.bounding_rect)
            to_remove.update(entity.on_delete() or ())

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

    def get_at(self, point: Point, radius: float = 10.0):
        """Get entities at given coordinates.

        Get a list with all entities within radius from given point, sorted
        from closest to farthest.
        """
        distances = {}

        def get_distance(entity, point):
            result = entity.distance(point, squared=True)
            distances[entity] = result
            return result

        return sorted(filter(lambda e: get_distance(e, point) <= radius,
                             map(self.entities.get,
                                 self.rtree.intersection(
                                     (point.x - radius, point.y - radius,
                                      point.x + radius, point.y + radius)))),
                      key=distances.get)


INSTANCE = EntityIndex()
