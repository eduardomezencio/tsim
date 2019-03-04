"""Entity base class."""

from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from itertools import count
from typing import ClassVar, Dict
import shelve

from dataslots import with_slots

from tsim.geometry import BoundingRect


@with_slots
@dataclass(frozen=False)
class Entity(ABC):
    """Base class with automatic unique id."""

    ID = count()
    DICT: ClassVar[Dict[int, 'Entity']] = {}

    id: int = field(init=False, default_factory=ID.__next__)
    bounding_rect: BoundingRect = field(init=False, default=None)

    def __post_init__(self):
        """Dataclass post-init."""
        Entity.DICT[self.id] = self
        object.__setattr__(self, 'bounding_rect', self.calc_bounding_rect())

    @abstractmethod
    def calc_bounding_rect(self,
                           accumulated: BoundingRect = None) -> BoundingRect:
        """Calculate the bounding rect of the entity."""
        ...

    @classmethod
    def load(cls, filename):
        """Load entities from file."""
        with shelve.open(filename) as data:
            id_ = data.get('id', None)
            dict_ = data.get('dict', None)
            if id_:
                cls.ID = id_
            if dict_:
                cls.DICT = dict_

    @classmethod
    def save(cls, filename):
        """Save entities to file."""
        with shelve.open(filename) as data:
            data['id'] = cls.ID
            data['dict'] = cls.DICT
