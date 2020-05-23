"""Car class."""

from __future__ import annotations

import logging as log
from collections import defaultdict, deque
from enum import Enum
from itertools import repeat
from math import inf as INF
from typing import (TYPE_CHECKING, Collection, DefaultDict, Deque, List,
                    Optional, Set, Tuple)

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
from tsim.model.units import Duration, kph_to_mps, mps_to_kph, time_string
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
MAX_SPEED_KPH = 100.0
MAX_SPEED_MPS = kph_to_mps(MAX_SPEED_KPH)

LOCK_ALL = True


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

        lead_location: TODO: ???

        followers: Set of agents that have this car as lead.

        diatance_to_lock: The remaining distance to travel before locking the
            traffic lock ahead. Is `Null` when not following a lock or after
            locking the lock.

        distance_to_release: TODO: ???

        lock_queue: TODO: ???

        lock_count: TODO: ???

        waiting: TODO: ???

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
            where a single line of traffic can happen, like a lane or a curve.

        network_position: The position inside of the current
            `network_location`. This position is measured in meters from the
            beginning of the location (double buffered).

        traffic_node: TODO: ???

        shadow_location: TODO: ???

        shadow_node: TODO: ???

        schedule: The agent's schedule.
    """

    __slots__ = ('active', 'deactivation_timer', 'position', 'direction',
                 'direction_changed', 'target_lane', 'side_vector',
                 'side_offset', 'network_segment', 'network_segment_end',
                 'curve_override', 'lead', 'lead_location', 'followers',
                 'distance_to_lock', 'distance_to_release', 'lock_queue',
                 'lock_count', 'waiting', 'current_max_speed', 'path',
                 'path_segment', 'path_last_segment', 'path_last_way',
                 'next_location', 'speed', 'network_location',
                 'network_position', 'traffic_node', 'shadow_location',
                 'shadow_node', 'schedule')

    MINIMUM_DISTANCE = MINIMUM_DISTANCE

    active: bool
    deactivation_timer: Duration
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
    lead_location: NetworkLocation
    followers: Set[Car]
    distance_to_lock: float
    distance_to_release: float
    lock_queue: Deque[TrafficLock]
    lock_count: DefaultDict[TrafficLock, int]
    waiting: list
    current_max_speed: float
    path: Path
    path_segment: int
    path_last_segment: bool
    path_last_way: bool
    next_location: NetworkLocation
    speed: List[float]
    network_location: NetworkLocation
    network_position: List[float]
    traffic_node: LinkedListNode[Car]
    shadow_location: NetworkLocation
    shadow_node: LinkedListNode[Car]
    schedule: Schedule

    def __init__(self, schedule: Schedule = None):
        self.active = False
        self.deactivation_timer = 0.0
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
        self.lead_location = None
        self.followers = set()
        self.distance_to_lock = None
        self.distance_to_release = None
        self.lock_queue = deque()
        self.lock_count = defaultdict(int)
        self.waiting = [0, None]
        self.current_max_speed = MAX_SPEED_MPS
        self.path = None
        self.path_segment = 0
        self.path_last_segment = False
        self.path_last_way = False
        self.next_location = None
        self.speed = [0.0, 0.0]
        self.network_location = None
        self.network_position = [0.0, 0.0]
        self.traffic_node = None
        self.shadow_location = None
        self.shadow_node = None
        self.schedule = schedule
        self._state = None
        super().__init__()
        Index.INSTANCE.simulation.add(self)

    @property
    def owner(self) -> None:
        """Get `None` since this is not a lock."""
        return None

    @property
    def state(self) -> State:
        """Get or set the car state."""
        return self._state

    @state.setter
    def state(self, value: State):
        self._state = value
        self.update = value.__get__(self, self.__class__)

    def get_network_position(self, location: NetworkLocation,
                             buffer: int) -> float:
        """Get network position.

        The implementation of this method for this class ignores `location`.
        """
        return self.network_position[buffer]

    def is_at(self, location: NetworkLocation) -> bool:
        """Get whether car is at given `location`."""
        return (self.network_location is location or
                self.shadow_location is location)

    def oriented_way_position(self, ready: int) -> OrientedWayPosition:
        """Get the car's current `OrientedWayPosition`."""
        return self.network_location.oriented_way_position(
            self.network_position[ready])

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

    # TODO: Here, first remove agent if already placed on a location.
    def place_at(self, network_position: NetworkPosition, buffer: int):
        """Place the car at given `NetworkPosition`.

        Places the car at the given position, without a destination and
        deactivated.
        """
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
        self.speed[:] = 0.0, 0.0
        self.network_location = network_position.location
        self.network_position[:] = repeat(network_position.position, 2)

        lane = network_position.location
        if isinstance(lane, Lane):
            self.state = State.ON_LANE
            Index.INSTANCE.simulation.raise_event(
                'entered_way', self, Index.INSTANCE.simulation.time,
                lane.oriented_way_position(network_position.position))
        else:
            lane = lane.dest
            self.state = State.ON_CURVE
        self.current_max_speed = min(MAX_SPEED_MPS, lane.way.max_speed)

        self.traffic_node = network_position.location.insert_agent(
            self, buffer)
        self.update_lead(buffer)
        self._update_previous(buffer)

        self.set_active(False)

    def set_destination(self, destination: OrientedWayPosition,
                        ready: int, target: int):
        """Set the car's destination.

        Tries to find a path from the car location to the given destination. If
        a path is found, it's set as the car's path and the car is activated.
        """
        oriented_way = self.oriented_way_position(ready)
        self.path = Index.INSTANCE.path_map.path(oriented_way, destination)
        self.path_segment = 0
        self.path_last_segment = False

        if self.path is None:
            return

        lane = self.network_location
        if isinstance(lane, Curve):
            lane = lane.dest
        self.path_last_way = self.path_segment == len(self.path.ways) - 1
        self._calc_segment_end(lane, self.network_segment_end)
        self._calc_next_location(lane)
        if not self.path_last_way:
            self.update_lead(ready)
            self._calc_target_lane(lane)
            self._start_lane_change(lane, ready, target, False)
        self.set_active()

    def remove(self, ready: int, target: int, enqueue: bool = True):
        """Remove car from simulation."""
        def _remove_node():
            followers = list(self.followers)
            self.release_all_locks(target)
            if self.traffic_node:
                self.traffic_node.remove()
            if self.shadow_node:
                self.shadow_node.remove()
            if self.lead is not None:
                self.lead.remove_follower(self, target)
            for follower in followers:
                follower.notify(target)
            Index.INSTANCE.simulation.remove(self)

        self.speed[:] = 0.0, 0.0
        self.path = None
        self.set_active(False)
        self.state = State.REMOVED
        if enqueue:
            Index.INSTANCE.simulation.enqueue(_remove_node)
        else:
            _remove_node()

        if isinstance(self.network_location, Lane):
            lane = self.network_location
            Index.INSTANCE.simulation.raise_event(
                'left_way', self, Index.INSTANCE.simulation.time,
                lane.oriented_way_position(self.network_position[ready]))

    def release_all_locks(self, buffer):
        """Release all locks acquired by the car.

        This is not the natural way of releasing locks, but a way to force all
        locks to be released as terminal locks to be used, for example, before
        removing the car from the simulation, so that no lock stays with this
        car forever.
        """
        index, lock_order = self.waiting
        if lock_order is not None:
            lock_order[index].drop(self)

        for lock, count_ in list(self.lock_count.items()):
            for _ in range(count_):
                lock.release(self, buffer, True)
        self.lock_queue.clear()
        self.distance_to_release = None

    def find_lead(self) -> Tuple[TrafficAgent, NetworkLocation, bool]:
        """Find the first agent ahead of this one.

        If there is some agent ahead on the same network location, this method
        must always return this agent. Otherwise, it will depend on the current
        path. If in the last way of the path, no other location is checked and
        `None` is returned. If not in the last way of the path, the next
        location on the path is checked and the first agent found is returned,
        or `None` if there are no agents. The lead location is returned as the
        second value of the tuple.

        Locks that are owned by the agent and marked as terminal (meaning that
        all other locks required by this one were acquired) are skipped.
        Whether the `lead` was skipped is the third element of the returned
        tuple.
        """
        skipped_lead = False
        for agent in self.traffic_node.next.iterate_forward():
            if agent.owner is self and not agent.terminal:
                skipped_lead |= agent is self.lead
            else:
                return agent, self.network_location, skipped_lead

        if self.path_last_way:
            return None, None, skipped_lead

        if self.next_location:
            for agent in self.next_location.traffic:
                if agent is self:
                    break
                if agent.owner is self and not agent.terminal:
                    skipped_lead |= agent is self.lead
                else:
                    return agent, self.next_location, skipped_lead

        return None, None, skipped_lead

    def update_lead(self, buffer: int):
        """Find new lead for this car and update accordingly."""
        new_lead, new_lead_location, skipped_lead = self.find_lead()
        if new_lead is self.lead:
            return

        if self.lead and not skipped_lead:
            self.lead.remove_follower(self, buffer)
        else:
            self.distance_to_lock = None

        self.lead, self.lead_location = new_lead, new_lead_location
        if new_lead:
            new_lead.add_follower(self, buffer)

    def distance_to_lead(self, buffer: int) -> float:
        """Get the distance to the lead or infinite if there's no lead."""
        if self.lead is None:
            return INF

        location = self.network_location
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

    def distance_to(self, other: TrafficAgent, buffer: int) -> float:
        """Get distance to another agent.

        Checks in current location and next location ahead. Returns 0.0 if
        `other` is not found in this interval.
        """
        location = self.network_location
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

    def notify(self, buffer: int):
        """Notify this car of lead events."""
        self.update_lead(buffer)
        if not self.active and self.state is not State.REMOVED:
            self.set_active()
            self.notify_followers(buffer)

    def notify_followers(self, buffer: int):
        """Notify followers of events."""
        for agent in list(self.followers):
            agent.notify(buffer)

    def start_locking_lead(self, buffer: int, lock_all: bool = LOCK_ALL):
        """Start procedure to acquire the lock right ahead of the car."""
        self.distance_to_lock = None
        location = self.lead_location if lock_all else None
        lock_order = self.lead.lock_order[location]
        self.waiting[:] = 0, lock_order
        lock_order[0].lock(self, buffer, True, location)

    def acquire(self, lock: TrafficLock, buffer: int, terminal: bool,
                location: Optional[NetworkLocation] = None):
        """Register acquisition of `lock` by car."""
        try:
            if terminal:
                self.lock_count[lock] += 1
                next_index = self.waiting[0] + 1
                if next_index >= len(self.waiting[1]):
                    self.waiting[1] = None
                    self.lead.lock(self, buffer, False, location)
                else:
                    self.waiting[0] = next_index
                    self.waiting[1][next_index].lock(self, buffer,
                                                     True, location)
            else:
                self.lock_queue.append(lock)
                if self.distance_to_release is None:
                    self.distance_to_release = (self.distance_to(lock, buffer)
                                                + MINIMUM_DISTANCE / 2)
                self.notify(buffer)
        except AttributeError as error:
            Index.INSTANCE.simulation.raise_event('focus', self)
            log.exception(error)

    def add_follower(self, agent: TrafficDynamicAgent, buffer: int):
        """Register agent as follower."""
        self.followers.add(agent)

    def update_followers(self, buffer: int):
        """Update lead of registered followers."""
        for follower in list(self.followers):
            follower.update_lead(buffer)

    def remove_follower(self, agent: TrafficDynamicAgent, buffer: int):
        """Unregister agent as follower."""
        if agent in self.followers:
            self.followers.remove(agent)
            # Ignoring agent here in `_update_previous` is important because
            # when cars are swapping lanes the "previous" can be a shadow node
            # of the same agent, causing problems.
            self._update_previous(buffer, (agent,))

    def enqueue_network_location_change(self, location: NetworkLocation,
                                        target: int, to_curve: bool):
        """Enqueue location change for the end of the simulation frame."""
        def _network_location_change():
            self._calc_next_location(location)
            if to_curve:
                self._end_lane_change(target)
            self.network_location = location
            self.traffic_node.remove()
            self.traffic_node = location.insert_agent(self, target)
            if to_curve:
                self.notify_followers(target)
            self.update_lead(target)
        Index.INSTANCE.simulation.enqueue(_network_location_change, ())

    def update(self, dt: Duration,  # pylint: disable=method-hidden
               ready: int, target: int):
        """Update the car.

        This method is a placeholder for the car update method. All update
        methods share the same signature. `dt` is the time duration passed
        since last update. `ready` and `target` are the indexes of the ready
        buffer and the target buffer.
        """

    def update_on_lane(self, dt: Duration, ready: int, target: int):
        """Update the car on `on_lane` state."""
        if self.direction_changed:
            self.direction_changed = False

        self._follow(dt, ready, target)

        speed = self.speed[target] * dt
        self._update_distance_to_lock(speed, ready)

        if self.side_offset is not None \
                and self._lane_movement(dt, ready, target):
            position_read_buffer = target
        else:
            position_read_buffer = ready

        position = self.network_position[position_read_buffer] + speed
        location = self.network_location

        if position < self.network_segment_end:
            # Still in same segment.
            self.position = self.position + self.direction * speed
            self.network_position[target] = position
            return

        if self.path_last_segment:
            # Reached end of path.
            self.remove(ready, target)
            log.debug('[%s] %s reached destination.',
                      time_string(Index.INSTANCE.simulation.time), str(self))
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
            self.network_position[target] = position
            return

        # End of way/lane, entering an intersection/curve.
        self.path_segment += 1
        self.path_last_way = self.path_segment == len(self.path.ways) - 1
        target_oriented_way = self.path.ways[self.path_segment]
        curve = location.get_curve(target_oriented_way)
        self.current_max_speed = min(MAX_SPEED_MPS,
                                     target_oriented_way.way.max_speed)
        self._calc_curve_override(curve)
        self.position = curve.evaluate_position(offset, self.curve_override)
        self.network_segment = 0
        self.network_segment_end = curve.length
        self.network_position[target] = offset
        self.enqueue_network_location_change(curve, target, True)
        self.state = State.ON_CURVE

        Index.INSTANCE.simulation.raise_event(
            'left_way', self, Index.INSTANCE.simulation.time,
            location.oriented_way_position(self.network_position[ready]))

    def update_on_curve(self, dt: Duration, ready: int, target: int):
        """Update the car on `on_curve` state."""
        self._follow(dt, ready, target)

        speed = self.speed[target] * dt
        self._update_distance_to_lock(speed, ready)

        position = self.network_position[ready] + speed
        location = self.network_location

        if position < self.network_segment_end:
            # Still in curve.
            new_position = location.evaluate_position(position,
                                                      self.curve_override)
            direction = new_position - self.position
            if abs(direction.x) + abs(direction.y) > 0.005:
                self.direction = direction
                self.direction_changed = True
            else:
                self.direction_changed = False
            self.position = new_position
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
        self.network_position[target] = offset
        self.enqueue_network_location_change(location, target, False)
        if not self.path_last_way:
            self._calc_target_lane(location)
            self._start_lane_change(location, target, target)
        self.state = State.ON_LANE

        Index.INSTANCE.simulation.raise_event(
            'entered_way', self, Index.INSTANCE.simulation.time,
            location.oriented_way_position(offset))

    def update_removed(self, _dt: Duration, _ready: int, _target: int):
        """Update the car on `removed` state."""
        self.set_active(False)

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
            new_speed = min(max(self.speed[ready] + acceleration, 0.0),
                            self.current_max_speed)
            self.speed[target] = new_speed
            if new_speed < 0.001:
                self.deactivation_timer += dt
                if self.deactivation_timer > 0.5:
                    if self.lead is not None and not self.lead.active:
                        self.set_active(False)
                    else:
                        self.deactivation_timer = 0.0
            else:
                self.deactivation_timer = 0.0
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
                                                 endpoint, 0.0)
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

    def _start_lane_change(self, lane: Lane, ready: int, target: int,
                           enqueue: bool = True) -> bool:
        """Start lane change if needed.

        The `target_lane` must be set before calling this method, so the lane
        change will start if not already in target lane. Returns whether the
        network position was changed in the target buffer.
        """
        lane_index = lane.index
        if lane_index == self.target_lane:
            # No lane change needed.
            self.side_offset = None
            return False

        # Start lane change.
        position = LanePosition(lane, self.network_position[ready])
        self.side_offset = LANE_WIDTH
        if self.target_lane > lane_index:
            new_position = position.right_neighbor
            self.side_vector = self.direction.rotated_right()
        else:
            new_position = position.left_neighbor
            self.side_vector = self.direction.rotated_left()
        self._set_lane_change_max_speed(new_position, lane_index)

        new_lane = new_position.lane
        segment = new_lane.segments[self.network_segment]
        self.network_position[target] = new_position.position
        self.network_segment_end = segment.end_distance

        def _lane_change():
            old_location = self.network_location
            self.network_location = new_lane
            self._calc_next_location(new_lane)
            self.shadow_location = old_location
            self.shadow_node = self.traffic_node
            self.traffic_node = new_lane.insert_agent(self, target)
            self.update_lead(target)
            if self.lead is not None:
                self.lead.update_followers(target)
            self._update_previous(target)

        if enqueue:
            Index.INSTANCE.simulation.enqueue(_lane_change, ())
        else:
            _lane_change()

        return True

    def _lane_movement(self, dt: Duration, ready: int, target: int) -> bool:
        """Perform sideways movement between lanes.

        Assumes `side_offset` is not None and `_start_lane_change` was called
        before this method. Returns whether the network position was changed in
        the target buffer.
        """
        side_movement = min(LANE_CHANGE_SPEED_MPS, self.speed[ready]) * dt
        self.side_offset -= side_movement
        self.position += self.side_vector * side_movement

        # This block of code solves a problem that happens when `self` passes a
        # car that is in the source lane while going to the target lane and,
        # before ending the lane transition, the follower of `self` passes this
        # same car. The lead of this car would be updated to the passed car, so
        # the lead would be behind him and this would cause crazy behavior.
        shadow = self.shadow_node
        if (shadow is not None and shadow.has_next and
                shadow.next.data.network_position[ready]
                < self.network_position[ready]):
            shadow.remove()
            self.shadow_node = None
            self.notify_followers(target)

        if self.side_offset <= 0:
            self._end_lane_change(target, True)
            return self._start_lane_change(self.network_location,
                                           ready, target)
        return False

    def _end_lane_change(self, target: int, enqueue: bool = False):
        def _update_target():
            shadow_previous = (self.shadow_node.previous.data
                               if self.shadow_node.has_previous else None)
            self.shadow_location = None
            self.shadow_node.remove()
            self.shadow_node = None
            if shadow_previous is not None:
                shadow_previous.notify(target)

        self.side_offset = None
        self.side_vector = None
        if self.shadow_node is not None:
            if enqueue:
                Index.INSTANCE.simulation.enqueue(_update_target, ())
            else:
                _update_target()

    def _calc_curve_override(self, curve: Curve):
        if self.side_offset is None:
            self.curve_override = None
        else:
            nodes = curve.curve.nodes
            nodes[:, 0] = list(self.position)
            self.curve_override = bezier.Curve(nodes, degree=2)

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

    def _update_distance_to_lock(self, speed: float, buffer: int):
        if self.distance_to_lock is not None:
            self.distance_to_lock -= speed
            if self.distance_to_lock <= 0.0:
                self.start_locking_lead(buffer)

        if self.distance_to_release is not None:
            self.distance_to_release -= speed
            if self.distance_to_release < 0.0:
                lock = self.lock_queue.popleft()
                lock.release(self, buffer)
                if self.lock_queue:
                    self.distance_to_release = (
                        self.distance_to(self.lock_queue[0], buffer)
                        + MINIMUM_DISTANCE / 2)
                else:
                    self.distance_to_release = None

    def _update_previous(
            self, buffer: int,
            ignore: Optional[Collection[TrafficDynamicAgent]] = None):
        """Update lead for immediate follower on same location.

        To be used when the car is inserted in a location and the agent right
        behind is not yet registered as follower and needs to be updated.
        Ignores agents in `ignore` optional collection.
        """
        if self.traffic_node.has_previous:
            prev_node = self.traffic_node.previous
            prev = prev_node.data
            if ignore is not None and prev in ignore:
                return
            if isinstance(prev, Car):
                # There may be a better way to do this, but the locks here are
                # released because if the car behind onws a lock, this car
                # (self) probably got in the way between the car behind and the
                # lock and now will try to acquire the lock in its place.
                prev.release_all_locks(buffer)
                prev.update_lead(buffer)
                if not prev_node.is_shadow():
                    self.followers.add(prev)

    def debug_str(self):
        """Get detailed debug string for the car."""
        buffer = Index.INSTANCE.simulation.ready_buffer
        position = self.network_position[buffer]
        position = f'{position:.2f}' if position is not None else ''
        speed = mps_to_kph(self.speed[buffer])
        max_speed = mps_to_kph(self.current_max_speed)
        lock_count = ', '.join(f'{k.id}: {v}'
                               for k, v in self.lock_count.items()
                               if v != 0)
        to_lock = (f'{self.distance_to_lock:.2f}    '
                   if self.distance_to_lock is not None else 'None')
        to_release = (f'{self.distance_to_release:.2f}'
                      if self.distance_to_release is not None else 'None')

        index, lock_order = self.waiting
        if lock_order is not None:
            lock = lock_order[index]
            waiting = f'{lock}, owned by {lock.owner}'
        else:
            waiting = 'None'

        return (
            f'{repr(self)} @ {self.network_location} : {position}\n'
            f'speed: {speed:.1f} / {max_speed:.1f} kph\n'
            f'lead: {repr(self.lead)}    followers: {self.followers}\n'
            f'lock_queue: {self.lock_queue}\n'
            f'lock_count: {lock_count}\n'
            f'distance to lock: {to_lock}    '
            f'distance to release: {to_release}\n'
            f'waiting: {waiting}')

    def __repr__(self):
        return f'{Car.__name__}(id={self.id})'


class State(Enum):
    """States for the `Car` state machine."""

    ON_LANE = Car.update_on_lane
    ON_CURVE = Car.update_on_curve
    REMOVED = Car.update_removed


def is_shadow(node: LinkedListNode) -> bool:
    """Check if a traffic node is the shadow node of its agent."""
    try:
        return node.data.shadow_node is node
    except AttributeError:
        return False


LinkedListNode.is_shadow = is_shadow
