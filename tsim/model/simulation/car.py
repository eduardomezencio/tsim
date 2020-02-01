"""Car class."""

from __future__ import annotations

from collections import deque
from math import inf as INF
from typing import TYPE_CHECKING, Deque, List, Optional, Set, Tuple

import bezier

import tsim.model.index as Index
from tsim.model.entity import Entity
from tsim.model.geometry import Point, Vector
from tsim.model.network.intersection import Curve
from tsim.model.network.lane import LanePosition
from tsim.model.network.orientedway import OrientedWayPosition
from tsim.model.network.path import Path
from tsim.model.network.traffic import (TrafficAgent, TrafficDynamicAgent,
                                        TrafficLock)
from tsim.model.network.way import LANE_WIDTH, Lane
from tsim.model.simulation.schedule import Schedule
from tsim.model.units import Duration, kph_to_mps
from tsim.utils.linkedlist import LinkedListNode

if TYPE_CHECKING:
    from tsim.model.network.location import NetworkLocation, NetworkPosition

MINIMUM_DISTANCE = 5.0
SPEED_DISTANCE_FACTOR = 0.5
SPEED_DIFF_DISTANCE_FACTOR = 0.25
ACCELERATION_BASE = 5.0
BREAK_BASE = 75.0

LANE_CHANGE_SPEED_MPS = 1.0
LANE_CHANGE_MIN_DURATION = LANE_WIDTH / LANE_CHANGE_SPEED_MPS
MAX_SPEED_KPH = 60.0
MAX_SPEED_MPS = kph_to_mps(MAX_SPEED_KPH)


class Car(Entity, TrafficAgent):
    """The simulated dynamic entity of the simulator.

    A car, the main agent of the simulation. It has a schedule that tells where
    it needs to be at all times, so its destination can be set to the correct
    location and path. In each step of the simulation, the `update` method
    needs to called passing the target buffer index as argument. This index is
    used for the attributes that are double buffered. Some of the attributes
    are buffered so that a part of the previous state of the agent is kept
    after calculating its next state. This is important because agents use
    information from other agents to update themselves and this information
    must always come from the previous state.

    Attributes:
        active: Reflects the car's active status in the simulation. When not
            active, the car is not updated.
        position: The car world position coordinates.
        direction: Vector pointing in the direction the car is facing.
        direction_changed: Boolean indicating if car has changed the
            direction in the last update, to be used by the front end.
        target_lane: The index of the lane where the car must be to connect
            to the next location in its path.
        side_vector: Vector perpendicular to direction, pointing to the
            direction the car must move on lane change.
        side_offset: The distance the car is, sideways, in relation to where
            it should be to be in the center of the current lane. Used during
            lane changes for the transition.
        network_segment: The index of the segment the car is in, inside its
            current network location.
        network_segment_end: The network position where the current segment
            ends.
        curve_override: Curve used to get smooth animation in cases where the
            car enters a curve while there's still some side offset because
            of lane transitions. It's the curve corrected to start in the
            position the car was in.
        lead: The agent being followed, used to calculate the behavior of this
            car at each update.
        followers: Set of agents that have this car as lead.
        diatance_to_lock: The remaining distance to travel before locking the
            traffic lock ahead. Is `Null` when not following a lock or after
            locking the lock.
        current_max_speed: The maximum speed this car can reach in the current
            network location, given by both the location's maximum speed and
            the car's maximum speed.
        path: The path the car is currently following.
        path_segment: The index of the segment of the current path that the car
            is in.
        path_last_segment: Indicates whether the car is in the last segment of
            its path.
        path_last_way: Indicates whether the car is in the last way of its
            path.
        next_location: The next location ahead in the car's path. This location
            must be straight ahead, without need to change lanes, so if the car
            is at a lane and the next curve he will take is from another lane,
            this attribute is `None` until the lane change, then it becomes the
            curve.
        speed: The car's current speed (double buffered).
        network_location: The car's current location. The location is an object
            where a single line of traffic can happen, like a lane or a curve
            (double buffered).
        network_position: The position inside of the current
            `network_location`. This position is measured in meters from the
            beginning of the location (double buffered).
        schedule: The agent's schedule.
    """

    __slots__ = ('active', 'position', 'direction', 'direction_changed',
                 'target_lane', 'side_vector', 'side_offset',
                 'network_segment', 'network_segment_end', 'curve_override',
                 'lead', 'followers', 'distance_to_lock',
                 'distance_to_release', 'owns', 'current_max_speed', 'path',
                 'path_segment', 'path_last_segment', 'path_last_way',
                 'next_location', 'speed', 'network_location',
                 'network_position', 'traffic_node', 'schedule')

    active: bool
    position: Point
    direction: Vector
    direction_changed: bool
    target_lane: int
    side_vector: Vector
    side_offset: float
    network_segment: int
    network_segment_end: float
    curve_override: bezier.Curve
    lead: TrafficAgent
    followers: Set[Car]
    distance_to_lock: float
    distance_to_release: float
    owns: Deque[TrafficLock]
    current_max_speed: float
    path: Path
    path_segment: int
    path_last_segment: bool
    path_last_way: bool
    next_location: NetworkLocation
    speed: List[float]
    network_location: List[NetworkLocation]
    network_position: List[float]
    traffic_node: LinkedListNode[Car]
    schedule: Schedule

    def __init__(self, schedule: Schedule = None):
        super().__init__()
        self.active = False
        self.position = None
        self.direction = None
        self.direction_changed = False
        self.target_lane = 0
        self.side_vector = None
        self.side_offset = None
        self.network_segment = 0
        self.network_segment_end = 0.0
        self.curve_override = None
        self.lead = None
        self.followers = set()
        self.distance_to_lock = None
        self.distance_to_release = None
        self.owns = deque()
        self.current_max_speed = MAX_SPEED_MPS
        self.path = None
        self.path_segment = 0
        self.path_last_segment = False
        self.path_last_way = False
        self.next_location = None
        self.speed = [0.0, 0.0]
        self.network_location = [None, None]
        self.network_position = [0.0, 0.0]
        self.traffic_node = None
        self.schedule = schedule
        Index.INSTANCE.simulation.add(self)

    @property
    def owner(self) -> None:
        """Get `None` since this is not a lock."""
        return None

    @property
    def acquiring(self) -> None:
        """Get `None` since this is not a lock."""
        return None

    def get_network_position(self, location: NetworkLocation,
                             buffer: Optional[int] = None) -> float:
        """Get network position.

        The implementation of this method for this class ignores `location`.
        """
        if buffer is None:
            buffer = Index.INSTANCE.simulation.ready_buffer

        return self.network_position[buffer]

    def is_at(self, location: NetworkLocation,
              buffer: Optional[int] = None) -> bool:
        """Get whether car is at given `location`."""
        if buffer is None:
            buffer = Index.INSTANCE.simulation.ready_buffer

        return self.network_location[buffer] is location

    def oriented_way_position(self, ready: int) -> OrientedWayPosition:
        """Get the car's current `OrientedWayPosition`."""
        location = self.network_location[ready]
        return location.oriented_way_position(self.network_position[ready])

    def set_active(self, active: bool = True):
        """Activate or deactivate the car.

        Sets the car's `active` attribute (default True). If changed, update
        the simulation accordingly.
        """
        active = bool(active)
        if active == self.active:
            return
        self.active = active
        Index.INSTANCE.simulation.update_active_set(self)

    def place_at(self, network_position: NetworkPosition,
                 buffer: Optional[int] = None):
        """Place the car at given `NetworkPosition`.

        Places the car at the given position, without a destination and
        deactivated.
        """
        if buffer is None:
            buffer = Index.INSTANCE.simulation.ready_buffer

        position = network_position.world_and_segment_position()
        self.position = position.position
        self.direction = position.direction
        self.direction_changed = True
        self.target_lane = 0
        self.side_vector = None
        self.side_offset = None
        self.network_segment = position.segment
        self.network_segment_end = position.segment_end
        self.path = None
        self.path_segment = 0
        self.path_last_segment = False
        self.path_last_way = False
        self.speed[buffer] = 0.0
        self.network_location[buffer] = network_position.location
        self.network_position[buffer] = network_position.position

        lane = network_position.location
        if isinstance(lane, Lane):
            self.update = self._on_lane
        else:
            lane = lane.dest
            self.update = self._on_curve

        self.current_max_speed = min(MAX_SPEED_MPS, lane.way.max_speed)

        self.traffic_node = network_position.location.insert_agent(
            self, buffer)
        self.update_lead()
        if self.lead is None:
            self._update_previous()

        self.set_active(False)

    def set_destination(self, destination: OrientedWayPosition,
                        buffer: Optional[int] = None):
        """Set the car's destination.

        Tries to find a path from the car location to the given destination. If
        a path is found, it's set as the car's path and the car is activated.
        """
        if buffer is None:
            buffer = Index.INSTANCE.simulation.ready_buffer

        oriented_way = self.oriented_way_position(buffer)
        self.path = Index.INSTANCE.path_map.path(oriented_way, destination)
        self.path_segment = 0
        self.path_last_segment = False

        if self.path is None:
            return

        lane = self.network_location[buffer]
        if isinstance(lane, Curve):
            lane = lane.dest
        self.path_last_way = self.path_segment == len(self.path.ways) - 1
        self._calc_segment_end(lane, self.network_segment_end)
        self._calc_next_location(lane)
        if not self.path_last_way:
            self.update_lead()
            self._calc_target_lane(lane)
            self._start_lane_change(buffer, buffer)
        self.set_active()

    def find_lead(self) -> Tuple[TrafficAgent, Set[TrafficLock]]:
        """Find the first agent ahead of this one.

        If there is some agent ahead on the same network location, this method
        must always return this agent. Otherwise, it will depend on the current
        path. If in the last way of the path, no other location is checked and
        `None` is returned. If not in the last way of the path, the next
        location on the path is checked and the first agent found is returned,
        or `None` if there are no agents.

        All locks that are skipped for being owned by the car are returned as
        the second value on the tuple.
        """
        owned = set()
        for agent in self.traffic_node.next.iterate_forward():
            if agent.owner is self:
                owned.add(agent)
            else:
                return agent, owned

        if self.path_last_way:
            return None, owned

        if self.next_location:
            for agent in self.next_location.traffic:
                if agent is self:
                    break
                if agent.owner is self:
                    owned.add(agent)
                else:
                    return agent, owned

        return None, owned

    def update_lead(self):
        """Find new lead for this car and update accordingly."""
        new_lead, owned = self.find_lead()
        if new_lead is self.lead:
            return

        if self.lead and (self.lead not in owned):
            self.lead.remove_follower(self)
        else:
            self.distance_to_lock = None

        self.lead = new_lead
        if new_lead:
            new_lead.add_follower(self)

    def distance_to_lead(self, buffer: Optional[int] = None) -> float:
        """Get the distance to the lead or infinite if there's no lead."""
        if self.lead is None:
            return INF

        if buffer is None:
            buffer = Index.INSTANCE.simulation.ready_buffer

        location = self.network_location[buffer]
        position = self.network_position[buffer]

        if self.lead.is_at(location):
            lead_position = self.lead.get_network_position(location, buffer)
            return lead_position - position

        # This logic assumes that if the lead is not in the same network
        # location, it is in a location that follows the car location
        # immediately. Being two locations ahead would break this.
        if self.next_location:
            lead_position = self.lead.get_network_position(self.next_location,
                                                           buffer)
        else:
            lead_position = None

        if lead_position is not None:
            return location.length - position + lead_position

        return INF

    def distance_to(self, other: TrafficAgent,
                    buffer: Optional[int] = None) -> float:
        """Get distance to another agent.

        Checks in current location and next location ahead. Returns 0.0 if
        `other` is not found in this interval.
        """
        if buffer is None:
            buffer = Index.INSTANCE.simulation.ready_buffer

        location = self.network_location[buffer]
        position = self.network_position[buffer]

        if other.is_at(location):
            other_position = other.get_network_position(location, buffer)
            return other_position - position

        if self.next_location:
            other_position = other.get_network_position(self.next_location,
                                                        buffer)
        else:
            other_position = None

        if other_position is not None:
            return location.length - position + other_position

        return 0.0

    def notify(self):
        """Notify this car of lead events."""
        self.update_lead()
        if not self.active:
            self.set_active()

    def acquire(self, lock: TrafficLock, buffer: Optional[int] = None):
        """Register acquisition of `lock` by car."""
        self.owns.append(lock)
        if self.distance_to_release is None:
            self.distance_to_release = (self.distance_to(lock, buffer)
                                        + MINIMUM_DISTANCE)

    def add_follower(self, agent: TrafficDynamicAgent,
                     buffer: Optional[int] = None):
        """Register agent as follower."""
        for follower in list(self.followers):
            follower.update_lead()
        self.followers.add(agent)

    def remove_follower(self, agent: TrafficDynamicAgent):
        """Unregister agent as follower."""
        self.followers.discard(agent)
        self._update_previous()

    def update(self,  # pylint: disable=method-hidden
               dt: Duration, ready: int, target: int):
        """Update the car.

        This method is a placeholder for the car update method. All update
        methods share the same signature. `dt` is the time duration passed
        since last update. `ready` and `target` are the indexes of the ready
        buffer and the target buffer.
        """

    def _on_lane(self, dt: Duration, ready: int, target: int):
        """Update the car on `on_lane` state."""
        self._follow(dt, ready, target)
        speed = self.speed[target] * dt
        position = self.network_position[ready] + speed

        if (self.side_offset is not None
                and self._lane_movement(dt, ready, target)):
            location = self.network_location[target]
        else:
            location = self.network_location[ready]

        self._update_distance_to_lock(speed, ready)

        if position < self.network_segment_end:
            # Still in same segment.
            self.position = self.position + self.direction * speed
            self.network_location[target] = location
            self.network_position[target] = position
            return

        if self.path_last_segment:
            # Reached end of path.
            segment = location.segments[self.network_segment]
            self.speed[target] = 0.0
            self.path = None
            self.network_segment_end = segment.end_distance
            self.set_active(False)
            return

        # Reached end of segment
        offset = position - self.network_segment_end
        self.network_segment += 1
        if self.network_segment < location.segment_count:
            # Still in same way/lane, entering a new segment.
            segment = location.segments[self.network_segment]
            direction = segment.vector
            self.direction = direction
            self.direction_changed = True
            self.position = segment.start + direction * offset
            if self.side_offset is not None:
                self.position -= self.side_vector * self.side_offset
            self._calc_segment_end(location, segment.end_distance)
            self.network_location[target] = location
            self.network_position[target] = position
            return

        # End of way/lane, entering an intersection/curve.
        self.path_segment += 1
        self.path_last_way = self.path_segment == len(self.path.ways) - 1
        target_oriented_way = self.path.ways[self.path_segment]
        curve = location.get_curve(target_oriented_way)
        self._calc_next_location(curve)
        self._calc_curve_override(curve)
        self.position = curve.evaluate_position(offset, self.curve_override)
        self.network_segment = 0
        self.network_segment_end = curve.length
        self.network_location[target] = curve
        self.network_position[target] = offset
        self.current_max_speed = min(MAX_SPEED_MPS,
                                     target_oriented_way.way.max_speed)
        self.traffic_node.remove()
        self.traffic_node = curve.insert_agent(self, target)
        for agent in list(self.followers):
            agent.notify()
        self.update = self._on_curve

    def _on_curve(self, dt: Duration, ready: int, target: int):
        """Update the car on `on_curve` state."""
        self._follow(dt, ready, target)
        speed = self.speed[target] * dt
        position = self.network_position[ready] + speed
        location = self.network_location[ready]

        self._update_distance_to_lock(speed, ready)

        if position < self.network_segment_end:
            # Still in curve.
            new_position = location.evaluate_position(position,
                                                      self.curve_override)
            direction = new_position - self.position
            if abs(direction.x) + abs(direction.y) > 0.001:
                self.direction = direction
                self.direction_changed = True
            self.position = new_position
            self.network_location[target] = location
            self.network_position[target] = position
            return

        # Reached end of curve
        offset = position - self.network_segment_end
        location = location.dest
        segment = location.segments[0]
        self.position = segment.start + segment.vector * offset
        self.direction = segment.vector
        self.direction_changed = True
        self.network_segment = 0
        self.curve_override = None
        self._calc_segment_end(location, segment.end_distance)
        self.network_location[target] = location
        self.network_position[target] = offset
        self._calc_next_location(location)
        self.traffic_node.remove()
        self.traffic_node = location.insert_agent(self, target)
        self.update_lead()
        if not self.path_last_way:
            self._calc_target_lane(location)
            self._start_lane_change(target, target)
        self.update = self._on_lane

    def _follow(self, dt: Duration, ready: int, target: int):
        """Set the car speed according to car following logic."""
        speed = self.speed[ready]
        speed_diff = (self.lead.speed[ready] - speed) if self.lead else 0.0
        target_dist = max((MINIMUM_DISTANCE
                           + SPEED_DISTANCE_FACTOR * speed
                           - SPEED_DIFF_DISTANCE_FACTOR * speed_diff),
                          MINIMUM_DISTANCE)
        distance = self.distance_to_lead(ready)
        if distance > 0.0:
            acceleration = -(target_dist / distance - 1.0) * dt
            acceleration *= (ACCELERATION_BASE
                             if acceleration > 0.0 else
                             BREAK_BASE)
            self.speed[target] = min(max(self.speed[ready] + acceleration,
                                         0.0),
                                     self.current_max_speed)
        else:
            self.speed[target] = self.speed[ready]

    def _calc_target_lane(self, lane: Lane):
        """Set the value of `target_lane` according to next curve connections.

        The value is set to the closest lane with a connection to the next
        oriented way on the car's path.
        """
        target_oriented_way = self.path.ways[self.path_segment + 1]
        curve = lane.get_curve(target_oriented_way)
        self.target_lane = curve.source.lane_index

    def _calc_segment_end(self, lane: Lane, segment_end: float):
        """Set the value of `network_segment_end` depending on path segment.

        The value is set to the given `segment_end` if the car is not in the
        last segment of its path, meaning that the segment ends on the actual
        segment end. If on the last path segment, the end is set to the point
        where the path ends, so the car stops there.
        """
        if self.path_last_way:
            endpoint = self.path.ways[self.path_segment].endpoint
            path_end = lane.way_to_lane_position(self.path.offsets[1],
                                                 endpoint)
            if path_end <= segment_end:
                self.path_last_segment = True
                self.network_segment_end = path_end
                return
        self.network_segment_end = segment_end

    def _set_lane_change_max_speed(self, position: LanePosition,
                                   lane_index: int):
        """Set `current_max_speed` for lane change."""
        time = abs(self.target_lane - lane_index) * LANE_CHANGE_MIN_DURATION
        if time > 0.0:
            space = position.remaining
            self.current_max_speed = max(LANE_CHANGE_SPEED_MPS,
                                         min(space / time,
                                             self.current_max_speed))

    def _start_lane_change(self, ready: int, target: int) -> bool:
        """Start lane change if needed.

        The `target_lane` must be set before calling this method, so the lane
        change will start if not already in target lane. The return value is
        whether the location has changed on the target buffer.
        """
        lane = self.network_location[ready]
        lane_index = lane.index
        if lane_index == self.target_lane:
            # No lane change needed.
            self.side_offset = None
            return False

        # Start lane change.
        position = LanePosition(lane, self.network_position[ready])
        self._set_lane_change_max_speed(position, lane_index)

        self.side_offset = LANE_WIDTH
        if self.target_lane > lane_index:
            new_position = position.right_neighbor
            self.side_vector = self.direction.rotated_right()
        else:
            new_position = position.left_neighbor
            self.side_vector = self.direction.rotated_left()
        new_lane = new_position.lane
        self.network_location[target] = new_lane
        self.network_position[target] = new_position.position
        self._calc_next_location(new_lane)
        self.traffic_node.remove()
        self.traffic_node = new_lane.insert_agent(self, target)
        self.update_lead()
        if self.lead is None:
            self._update_previous()
        return True

    def _lane_movement(self, dt: Duration, ready: int, target: int) -> bool:
        """Perform sideways movement between lanes.

        Assumes `side_offset` is not None and `_start_lane_change` was called
        before this method. The return value is whether the location has
        changed on the target buffer.
        """
        side_movement = min(LANE_CHANGE_SPEED_MPS, self.speed[ready]) * dt
        self.side_offset -= side_movement
        self.position += self.side_vector * side_movement
        if self.side_offset <= 0:
            self.side_offset = None
            self.side_vector = None
            return self._start_lane_change(ready, target)
        return False

    def _calc_curve_override(self, curve: Curve):
        if self.side_offset is None:
            self.curve_override = None
        else:
            nodes = curve.curve.nodes
            nodes[:, 0] = list(self.position)
            self.curve_override = bezier.Curve(nodes, degree=2)
            self.side_offset = None

    def _calc_next_location(self, location: NetworkLocation):
        if self.path_last_way:
            self.next_location = None
            return

        if isinstance(location, Curve):
            self.next_location = location.dest
            return

        # Assume `location` is `Lane`.
        target_oriented_way = self.path.ways[self.path_segment + 1]
        self.next_location = location.get_curve(target_oriented_way, False)

    def _update_distance_to_lock(self, speed: float, buffer):
        if self.distance_to_lock is not None:
            self.distance_to_lock -= speed
            if self.distance_to_lock <= 0.0:
                self.distance_to_lock = None
                try:
                    self.lead.lock(self)
                except AttributeError:
                    print('ops')

        if self.distance_to_release is not None:
            self.distance_to_release -= speed
            if self.distance_to_release < 0.0:
                lock = self.owns.popleft()
                lock.release(self)
                if self.owns:
                    lock = self.owns[0]
                    self.distance_to_release = (self.distance_to(lock, buffer)
                                                + MINIMUM_DISTANCE)
                else:
                    self.distance_to_release = None

    def _update_previous(self):
        """Update lead for immediate follower on same location.

        To be used when the car is inserted in a location and the agent right
        behind is not yet registered as follower and needs to be updated.
        """
        if self.traffic_node.has_previous:
            prev = self.traffic_node.previous.data
            if isinstance(prev, Car):
                while prev.owns:
                    lock = prev.owns.popleft()
                    lock.release(prev)
                prev.distance_to_release = None
                prev.update_lead()

    def __repr__(self):
        return f'{Car.__name__}(id={self.id})'
