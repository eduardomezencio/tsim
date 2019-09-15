from __future__ import annotations

from dataclasses import dataclass
from typing import List, Tuple

from dataslots import with_slots

from tsim.model.entity import Entity, EntityRef
from tsim.model.network.way import Way

Address = Tuple[Way, float]


@with_slots
@dataclass(eq=False)
class Road(Entity):

    name: str
    ways: List[EntityRef[Way]]
