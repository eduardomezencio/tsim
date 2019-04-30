from dataclasses import dataclass
from typing import List

from dataslots import with_slots

from tsim.model.entity import Entity, EntityRef
from tsim.model.network import Way


@with_slots
@dataclass(eq=False)
class Road(Entity):

    name: str
    ways: List[EntityRef[Way]]
