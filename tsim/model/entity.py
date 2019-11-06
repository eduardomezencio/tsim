"""Entity base class."""

from __future__ import annotations

from abc import ABC
from dataclasses import dataclass, field
from typing import Generic, Iterable, NamedTuple, Optional, TypeVar, Union
from weakref import ref, ReferenceType

from dataslots import with_slots

from tsim.utils import pickling
import tsim.model.index as Index


@with_slots(add_dict=True, add_weakref=True)
@dataclass(eq=False)
class Entity(ABC):
    """Base class for entities."""

    id: int = field(init=False, default_factory=type(None))

    def __post_init__(self):
        Index.INSTANCE.add(self)

    def delete(self):
        """Delete this entity."""
        Index.INSTANCE.delete(self)

    def on_delete(self) -> DeleteResult:
        """Cleanup when deleting entity.

        Will be called by the index when deleting. Must return a `DeleteResult`
        object with iterables of entities to cascade delete and updated
        entities.
        """
        return DeleteResult.empty()

    __getstate__ = pickling.getstate
    __setstate__ = pickling.setstate


T = TypeVar('T', bound=Entity)


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

    __slots__ = '_id', '_value'

    _id: int
    _value: ReferenceType

    def __init__(self, entity: Union[EntityRef, ReferenceType, Entity, int]):
        # TODO: change to singledispatchmethod methods in Python 3.8
        if isinstance(entity, Entity):
            self._id = entity.id
            self._value = ref(entity)
        elif isinstance(entity, ReferenceType):
            self._id = entity().id
            self._value = entity
        elif isinstance(entity, EntityRef):
            self._id = entity.id
            self._value = entity.value
        else:
            self._id = entity.id
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


class DeleteResult(NamedTuple):
    """Result of a delete operation.

    All other entities to be deleted as a consequence of this deletion must be
    in `cascade` while all entities that suffered changes must be in `updated`.
    """

    cascade: Iterable[Entity]
    updated: Iterable[Entity]

    @staticmethod
    def empty():
        """Create empty `DeleteResult`."""
        return DeleteResult((), ())
