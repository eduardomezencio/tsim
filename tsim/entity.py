"""Entity base class."""

from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from itertools import chain, count
from typing import ClassVar, Dict
import shelve

from cached_property import cached_property
from dataslots import with_slots

from tsim.geometry import BoundingRect


@with_slots(add_dict=True)
@dataclass(frozen=False)
class Entity(ABC):
    """Base class with automatic unique id."""

    id_count: ClassVar[count] = count()
    index: ClassVar[Dict[int, 'Entity']] = {}

    id: int = field(init=False, default_factory=id_count.__next__)

    def __post_init__(self):
        """Dataclass post-init."""
        Entity.index[self.id] = self

    @cached_property
    def bounding_rect(self) -> BoundingRect:
        """Bounding rectangle cached property."""
        return self.calc_bounding_rect()

    @abstractmethod
    def calc_bounding_rect(self,
                           accumulated: BoundingRect = None) -> BoundingRect:
        """Calculate the bounding rectangle of the entity."""
        ...

    @classmethod
    def load(cls, filename):
        """Load entities from file."""
        with shelve.open(filename) as data:
            id_count = data.get('id_count', None)
            index = data.get('index', None)
            if id_count:
                cls.id_count = id_count
            if index:
                cls.index = index

    @classmethod
    def save(cls, filename):
        """Save entities to file."""
        with shelve.open(filename) as data:
            data['id_count'] = cls.id_count
            data['index'] = cls.index

    def __getstate__(self):
        """Remove cached properties for use with pickle/shelve."""
        dict_copy = self.__dict__.copy()
        for key in self.__dict__:
            if hasattr(self.__class__, key):
                if isinstance(getattr(self.__class__, key), cached_property):
                    del dict_copy[key]
        slots = chain.from_iterable(
            getattr(c, '__slots__', ()) for c in self.__class__.__mro__)
        slots_dict = {s: getattr(self, s) for s in slots if s != '__dict__'}
        return dict_copy, slots_dict

    def __setstate__(self, state):
        """Restore state for pickle/shelve."""
        dict_copy, slots = state
        self.__dict__.update(dict_copy)
        for key, value in slots.items():
            setattr(self, key, value)
