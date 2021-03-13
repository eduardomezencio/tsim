"""Entity serialization configuration."""

from tsim.core.entity import Entity, EntityRef
from tsim.utils import pickling


def configure():
    """Configure serialization for Entity related classes."""
    Entity.__getstate__ = pickling.getstate
    Entity.__setstate__ = pickling.setstate
    EntityRef.__getstate__ = _entity_ref_getstate
    EntityRef.__setstate__ = _entity_ref_setstate


def _entity_ref_getstate(self):
    return self.id


def _entity_ref_setstate(self, state):
    self._id = state  # pylint: disable=protected-access
    self._value = None  # pylint: disable=protected-access
