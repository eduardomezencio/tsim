"""Entity base class."""

from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from typing import Any, Generic, Iterable, Optional, TypeVar, Union
from weakref import ref, ReferenceType

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
    __weakref__: Any = field(init=False)

    @cached_property
    def bounding_rect(self) -> BoundingRect:
        """Get the bounding rectangle for the entity."""
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

    def on_delete(self) -> Iterable[Entity]:
        """Cleanup when deleting entity.

        Will be called by the index when deleting. Must return an iterable with
        other entities to cascade delete or None.
        """

    __getstate__ = pickling.getstate
    __setstate__ = pickling.setstate


T = TypeVar('T', bound=Entity)  # pylint: disable=invalid-name


class EntityRef(Generic[T]):
    """Reference for an entity.

    Avoid deep recursion in pickling. It pickles itself including only the id
    and not the entity. It also uses only the id for __eq__ and __hash__
    functions, so it can be used as dict key or in sets. In runtime, the
    reference to the entity is kept as a weakref.

    The id of an EntityRef can only be None when it's referencing an Entity
    that was just created and was not yet added to the index, so it does not
    have an id. Before the entity is added to the index, objects of this class
    must not be used as dict keys or in sets.
    """

    __slots__ = ('_id', '_value')

    _id: int
    _value: ReferenceType

    def __init__(self, entity: Union[ReferenceType, Entity, int]):
        if isinstance(entity, ReferenceType):
            self._id = entity().id
            self._value = entity
        elif isinstance(entity, Entity):
            self._id = entity.id
            self._value = ref(entity)
        else:
            self._id = entity
            self._value = None
        assert (isinstance(self._value, ReferenceType) or
                (self._value is None and isinstance(self._id, int)))

    @property
    def id(self) -> int:
        """Get entity id."""
        if self._id is None:
            self._id = self._value().id
        return self._id

    @property
    def value(self) -> Optional[T]:
        """Get entity from reference."""
        if self._value is not None:
            return self._value()
        value = Index.INSTANCE.entities[self._id]
        if value is not None:
            self._value = ref(value)
            return value
        return None

    def __getstate__(self):
        return self.id

    def __setstate__(self, state):
        self._id = state
        self._value = None

    def __call__(self):
        """Get the referenced value, like calling a weakref."""
        return self.value

    def __eq__(self, other: EntityRef):
        return isinstance(other, EntityRef) and self.id == other.id

    def __hash__(self):
        """Get hash for this reference.

        When the entity was not yet inserted into the index and does not have
        an id, it is not yet usable as a dict key or in a set, because the id
        will still change. A class must be immutable to be hashable, so
        __hash__ will assert id is not None.
        """
        assert self.id is not None
        return self.id

    def __repr__(self):
        return f'{EntityRef.__name__}[{type(self.value)}](id={self.id})'

    def __str__(self):
        return repr(self)
