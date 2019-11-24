"""Agent class."""

from __future__ import annotations

from typing import TYPE_CHECKING, List, Optional

import tsim.model.index as Index
from tsim.model.entity import Entity, EntityRef
from tsim.model.geometry import Point, Vector
from tsim.model.network.intersection import Curve
from tsim.model.network.path import Path
from tsim.model.network.position import OrientedWayPosition
from tsim.model.network.way import Lane
from tsim.model.simulation.schedule import Schedule
from tsim.model.units import Duration, kph_to_mps

if TYPE_CHECKING:
    from tsim.model.network.position import NetworkLocation, NetworkPosition

MAX_SPEED_KPH = 60.0
MAX_SPEED_MPS = kph_to_mps(MAX_SPEED_KPH)


class Agent(Entity):
    """The simulated dynamic entity of the simulator.

    An agent represents a single car or person in the simulation. It has a
    schedule that tells where it needs to be at all times, so its destination
    can be set to the correct location. In each step of the simulation, the
    `update` method needs to called passing the target buffer index as
    argument. This index is used for the attributes that are double buffered,
    `speed`, `network_location` and `network_position`. These are buffered so
    that a part of the previous state of the agent is kept after calculating
    its next state. This is important because agents use information from other
    agents to update themselves and this information must always come from the
    previous state.
    """

    __slots__ = ('position', 'direction', 'direction_changed',
                 'network_segment', 'network_segment_end', 'lead',
                 'current_max_speed', 'speed', 'network_location',
                 'network_position', 'schedule')

    active: bool
    position: Point
    direction: Vector
    direction_changed: bool
    network_segment: int
    network_segment_end: float
    lead: EntityRef[Agent]
    current_max_speed: float
    path: Path
    path_segment: int
    path_end: bool
    speed: List[float]
    network_location: List[NetworkLocation]
    network_position: List[float]
    schedule: Schedule

    def __init__(self, schedule: Schedule = None):
        super().__init__()
        self.active = False
        self.position = None
        self.direction = None
        self.direction_changed = False
        self.network_segment = 0
        self.network_segment_end = 0.0
        self.lead = None
        self.current_max_speed = MAX_SPEED_MPS
        self.path = None
        self.path_segment = 0
        self.path_end = False
        self.speed = [0.0, 0.0]
        self.network_location = [None, None]
        self.network_position = [0.0, 0.0]
        self.schedule = schedule
        Index.INSTANCE.simulation.add(self)

    @property
    def state(self) -> Agent.update:
        """Get or set the agent state.

        The state is the method that will be called when `update` is called.
        """
        return self.update

    @state.setter
    def state(self, value: Agent.update):
        self.update = value

    def oriented_way_position(self, ready: int) -> OrientedWayPosition:
        """Get the agent's current `OrientedWayPosition`."""
        location = self.network_location[ready]
        return location.oriented_way_position(self.network_position[ready])

    def set_active(self, active: bool = True):
        """Activate or deactivate the agent.

        Sets the agent's `active` attribute (default True). If changed, update
        the simulation accordingly.
        """
        active = bool(active)
        if active == self.active:
            return
        self.active = active
        Index.INSTANCE.simulation.update_active_set(self)

    def place_at(self, network_position: NetworkPosition,
                 buffer: Optional[int] = None):
        """Place the agent at given `NetworkPosition`.

        Places the agent at the given position, without a destination and
        deactivated.
        """
        if buffer is None:
            buffer = Index.INSTANCE.simulation.ready_buffer
        position = network_position.world_and_segment_position()
        self.position = position.position
        self.direction = position.direction
        self.direction_changed = True
        self.network_segment = position.segment
        self.network_segment_end = position.segment_end
        self.path = None
        self.path_segment = 0
        self.path_end = False
        self.speed[buffer] = 0.0
        self.network_location[buffer] = network_position.location
        self.network_position[buffer] = network_position.position
        self.state = (self.on_lane
                      if isinstance(network_position.location, Lane)
                      else self.on_curve)
        self.set_active(False)

    def set_destination(self, destination: OrientedWayPosition,
                        buffer: Optional[int] = None):
        """Set the agent's destination.

        Tries to find a path from the agent location to the given destination.
        If a path is found, it's set as the agent's path and the agent is
        activated.
        """
        if buffer is None:
            buffer = Index.INSTANCE.simulation.ready_buffer
        oriented_way = self.oriented_way_position(buffer)
        self.path = Index.INSTANCE.path_map.path(oriented_way, destination)
        self.path_segment = 0
        self.path_end = False
        if self.path is not None:
            lane = self.network_location[buffer]
            if isinstance(lane, Curve):
                lane = lane.dest
            self._calc_segment_end(lane, self.network_segment_end)
            self.set_active()

    def update(self, dt: Duration, ready: int, target: int):
        """Update the agent.

        This method is a placeholder for the agent state method. All state
        method share the same arguments. `dt` is the time duration passed since
        last update. `ready` and `target` are the indexes of the ready buffer
        and the target buffer.
        """

    def on_lane(self, dt: Duration, ready: int, target: int):
        """Update the agent on `on_lane` state."""
        self.follow(dt, ready, target)
        speed = self.speed[target] * dt
        position = self.network_position[ready] + speed
        old_location = self.network_location[ready]

        if position < self.network_segment_end:
            # Still in same segment.
            self.position = self.position + self.direction * speed
            self.network_location[target] = old_location
            self.network_position[target] = position
            return

        # Reached end of segment or path.
        if self.path_end:
            segment = old_location.segments[self.network_segment]
            self.speed[target] = 0.0
            self.path = None
            self.path_segment = 0
            self.path_end = False
            self.network_segment_end = segment.end_distance
            self.set_active(False)
            return

        # Reached end of segment
        offset = position - self.network_segment_end
        self.network_segment += 1
        if self.network_segment < old_location.segment_count:
            # Still in same way/lane, entering a new segment.
            segment = old_location.segments[self.network_segment]
            direction = segment.vector
            self.direction = direction
            self.direction_changed = True
            self.position = segment.start + direction * offset
            self._calc_segment_end(old_location, segment.end_distance)
            self.network_location[target] = old_location
            self.network_position[target] = position
            return

        # End of way/lane, entering an intersection/curve.
        self.path_segment += 1
        target_oriented_way = self.path.ways[self.path_segment]
        curve = old_location.get_curve(target_oriented_way)
        position = curve.evaluate_position(offset)
        self.direction = position - self.position
        self.position = position
        self.direction_changed = True
        self.network_segment = 0
        self.network_segment_end = curve.length
        self.network_location[target] = curve
        self.network_position[target] = offset
        self.state = self.on_curve

    def on_curve(self, dt: Duration, ready: int, target: int):
        """Update the agent on `on_curve` state."""
        self.follow(dt, ready, target)
        speed = self.speed[target] * dt
        position = self.network_position[ready] + speed
        old_location = self.network_location[ready]

        if position < self.network_segment_end:
            # Still in curve.
            new_position = old_location.evaluate_position(position)
            self.direction = new_position - self.position
            self.position = new_position
            self.direction_changed = True
            self.network_location[target] = old_location
            self.network_position[target] = position
            return

        # Reached end of curve
        offset = position - self.network_segment_end
        segment = old_location.dest.segments[0]
        self.position = segment.start + segment.vector * offset
        self.direction = segment.vector
        self.direction_changed = True
        self.network_segment = 0
        self._calc_segment_end(old_location.dest, segment.end_distance)
        self.network_location[target] = old_location.dest
        self.network_position[target] = offset
        self.state = self.on_lane

    def _calc_segment_end(self, lane: Lane, segment_end: float):
        if self.path_segment == len(self.path.ways) - 1:
            endpoint = self.path.ways[self.path_segment].endpoint
            path_end = lane.way_to_lane_position(self.path.offsets[1],
                                                 endpoint)
            if path_end <= segment_end:
                self.path_end = True
                self.network_segment_end = path_end
                return
        self.network_segment_end = segment_end

    def follow(self, dt: Duration, ready: int, target: int):
        """Set the agent speed according to car following logic."""
        # TODO: write real car following here
        acceleration = 1.0 * dt
        self.speed[target] = min(max(self.speed[ready] + acceleration, 0.0),
                                 self.current_max_speed)
