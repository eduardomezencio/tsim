"""Entity base class."""

from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from typing import Generic, TypeVar, Union

from cached_property import cached_property
from dataslots import with_slots

from tsim.model.geometry import BoundingRect, Point
from tsim.utils import osm as xurl_provider, pickling
import tsim.model.index as Index


@with_slots(add_dict=True)
@dataclass(eq=False)
class Entity(ABC):
    """Base class for spatial entities."""

    id: int = field(init=False, default_factory=type(None))
    xid: int = field(init=False, default_factory=type(None))

    @cached_property
    def bounding_rect(self) -> BoundingRect:
        """Bounding rectangle cached property."""
        return self.calc_bounding_rect()

    @property
    def xurl(self) -> str:
        """Get external url for the entity."""
        return xurl_provider.get_url(self)

    @abstractmethod
    def calc_bounding_rect(self,
                           accumulated: BoundingRect = None) -> BoundingRect:
        """Calculate the bounding rectangle of the entity."""

    @abstractmethod
    def distance(self, point: Point, squared: bool = False) -> float:
        """Calculate smallest distance from the entity to a point."""

    def on_delete(self):
        """Cleanup when deleting entity.

        Will be called by the index when deleting. Must return an iterable with
        other entities to cascade delete or None.
        """

    __getstate__ = pickling.getstate
    __setstate__ = pickling.setstate


T = TypeVar('T', bound=Entity)  # pylint: disable=invalid-name


class EntityRef(Generic[T]):
    """Reference for an entity.

    This class exists to avoid deep recursion in pickling. It pickles itself
    including only the id and not the reference to the entity.
    """

    __slots__ = ('_id', '_value')

    _id: int
    _value: T

    def __init__(self, entity: Union[Entity, int]):
        if isinstance(entity, Entity):
            self._id = None
            self._value = entity
        else:
            self._id = int(entity)
            self._value = entity.id

    @property
    def id(self):
        """Get entity id."""
        if self._id is None and self._value:
            self._id = self._value.id
        return self._id

    @property
    def value(self):
        """Get entity from reference."""
        if not self._value:
            self._value = Index.INSTANCE.entities[self._id]
        return self._value

    def __getstate__(self):
        if self._id is None:
            self._id = self._value.id
        return self._id

    def __setstate__(self, state):
        self._id = state
        self._value = None

    def __repr__(self):
        return f'{EntityRef.__name__}(id={self.id})'

    def __str__(self):
        return repr(self)
