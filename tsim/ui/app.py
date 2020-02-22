"""App class implementation, the graphic UI main class."""

from __future__ import annotations

import logging as log
from collections import deque
from functools import partial
from itertools import chain, cycle, islice
from math import floor
from random import random, seed, shuffle
from typing import Dict, Tuple

from direct.task import Task
from panda3d.core import (AntialiasAttrib, ConfigVariableBool, NodePath,
                          RigidBodyCombiner, TextNode)

import tsim.ui.input as Input
import tsim.ui.panda3d as p3d
from tsim.model.index import INSTANCE as INDEX
from tsim.model.geometry import Point, bounding_rect_center
from tsim.model.network.node import Node
from tsim.model.network.lane import LanePosition
from tsim.model.network.orientedway import OrientedWayPosition
from tsim.model.network.way import Way
from tsim.model.simulation.car import Car
from tsim.model.units import HOUR, normalized_hours, time_string
from tsim.ui.camera import Camera
from tsim.ui.cursor import Cursor
from tsim.ui.grid import Grid
from tsim.ui.objects import factory as Factory, world as World
from tsim.ui.sky import Sky
from tsim.utils.iterators import window_iter

EVENTS = ('add_car', 'focus', 'follow', 'network_entities_changed',
          'removed_car')
FRAME_DURATION = 1 / 60
SPEED_STEPS = 4


class App:
    """Graphic UI application, using Panda3D."""

    simulation_speed_text: TextNode
    time_text: TextNode
    network_entities: Dict[int, NodePath]
    agents: Dict[Car, NodePath]
    roads: Dict[Tuple[int, int], NodePath]

    def __init__(self, index_name: str):
        self.network_entities = {}
        self.agents = {}
        self.roads = {}
        self._event_queue = deque()
        self._frame_count = 0
        self._simulation_speed = 0
        self._number_input_buffer = None

        log_config()
        panda3d_config()
        init_index(index_name)
        Input.init()

        self.scene = NodePath('scene')
        self.agents_parent = NodePath('agents')
        self.agents_parent.reparent_to(self.scene)
        self.camera = Camera()
        self.world = World.create(self.scene, 10000, 16)
        self.sky = Sky(self.scene, self.camera)
        self.cursor = Cursor(self.world)
        self.grid = Grid(50.0, 1000.0, self.world, self.cursor.actor)

        self._init_objects()
        self._init_event_handlers()
        self._build_on_screen_text()

        seed(2)
        self.generate_random_cars(250)

        # TODO: Change to set simulation time when loading INDEX from file.
        INDEX.simulation.time = random() * 24.0 * HOUR
        self.sky.set_time(normalized_hours(INDEX.simulation.time))

        self.scene.reparent_to(p3d.RENDER)

        # p3d.BASE.movie('movie/movie', 31536000, 60, 'jpg')

    def generate_random_cars(self, limit=500):
        """Generate `limit` random cars."""
        lanes = list(chain.from_iterable(w.lanes for w in
                                         INDEX.get_all(of_type=Way)))
        shuffle(lanes)
        for lane1, lane2 in window_iter(islice(cycle(lanes), limit)):
            distance1 = random() * lane1.length
            distance2 = random() * lane2.length
            path = INDEX.path_map.path(lane1.oriented_way_position(distance1),
                                       lane2.oriented_way_position(distance2))
            if path is None:
                continue
            self.on_add_car(Car(), LanePosition(lane1, distance1),
                            lane2.oriented_way_position(distance2))

    @property
    def simulation_speed(self) -> int:
        """Get simulation speed in `1 / SPEED_STEPS` increments."""
        return self._simulation_speed

    def change_simulation_speed(self, value: int, relative: bool = True,
                                max_: int = SPEED_STEPS * 4):
        """Add value to the simulation speed."""
        if relative:
            value = self._simulation_speed + value
        self._simulation_speed = floor(max(0, min(value, max_)))
        self._update_simulation_speed_text()

    def run(self):
        """Start the main loop."""
        p3d.TASK_MGR.add(self.update)
        p3d.BASE.run()

    def update(self, _task: Task):
        """Update task, to run every frame."""
        self._frame_count += self._simulation_speed

        while self._frame_count >= SPEED_STEPS:
            self._frame_count -= SPEED_STEPS
            INDEX.simulation.update(FRAME_DURATION)
            self.update_agents()

        while self._event_queue:
            name, args = self._event_queue.popleft()
            self.event_handlers[name](*args)

        self.camera.update()
        self.sky.set_time(normalized_hours(INDEX.simulation.time))
        self.sky.update()
        self.cursor.update()
        self.grid.update()
        Input.clear()

        self._update_time_text()

        return Task.cont

    def update_network_entities(self):
        """Update graphics for changed network entities."""
        for id_ in INDEX.consume_updates():
            self._update_network_entity(id_)

    def _update_network_entity(self, id_):
        node_path = self.network_entities.get(id_, None)
        if node_path is not None and not node_path.is_empty():
            node_path.remove_node()
        entity = INDEX.entities.get(id_, None)
        if entity is not None:
            point = bounding_rect_center(entity.bounding_rect)
            parent = self.get_roads_parent(point)
            self.network_entities[id_] = Factory.create(parent, entity)

    def simulation_event_listener(self, name: str, *args):
        """Handle simulation events."""
        p3d.MESSENGER.send(name, list(args))

    def get_roads_parent(self, point: Point) -> NodePath:
        """Get the correct parent for roads on the given point.

        The world is divided into sections, each section with a node path
        holding a part of the network nodes. This is to make the combiner
        `collect` method faster when changing the network, while keeping the
        number of nodes small.
        """
        key = (point.x // 1024, point.y // 1024)
        parent = self.roads.get(key, None)
        if parent is None:
            combiner = RigidBodyCombiner(f'roads{key}')
            parent = self.world.attach_new_node(combiner)
            self.roads[key] = parent
        return parent

    def _init_objects(self):
        """Create all objects on the index."""
        for node in filter(lambda e: isinstance(e, Node),
                           INDEX.entities.values()):
            parent = self.get_roads_parent(node.position)
            self.network_entities[node.id] = Factory.create_node(parent, node)

        for way in filter(lambda e: isinstance(e, Way),
                          INDEX.entities.values()):
            center = bounding_rect_center(way.bounding_rect)
            parent = self.get_roads_parent(center)
            self.network_entities[way.id] = Factory.create_way(parent, way)

        # TODO: load agents

        for node_path in self.roads.values():
            node_path.node().collect()

    def _init_event_handlers(self):
        """Initialize all event listeners/handlers."""
        self.event_handlers = {e: getattr(self, f'on_{e}', lambda: None)
                               for e in EVENTS}
        for event in EVENTS:
            p3d.BASE.accept(event, partial(self.enqueue_event, event))

        INDEX.simulation.register_listener(self.simulation_event_listener)

        def _init_buffer():
            self._number_input_buffer = []

        def _flush_buffer():
            if self._number_input_buffer:
                id_ = int(''.join(self._number_input_buffer))
                car = INDEX.entities.get(id_, None)
                if isinstance(car, Car):
                    self.on_follow(car)
            self._number_input_buffer = None

        def _number_input(key):
            if self._number_input_buffer is not None:
                self._number_input_buffer.append(key)

        p3d.BASE.accept('control', _init_buffer)
        p3d.BASE.accept('control-up', _flush_buffer)

        for i in range(10):
            key = str(i)
            p3d.BASE.accept(key, partial(self.change_simulation_speed,
                                         int(2 ** (i - 1)), False, 256))
            p3d.BASE.accept(f'control-{key}', partial(_number_input, key))
        for key, value in (('wheel_up', 1), ('wheel_down', -1)):
            p3d.BASE.accept(key, partial(self.change_simulation_speed, value))

    def enqueue_event(self, name: str, *args):
        """Enqueue event from panda3d messenger."""
        self._event_queue.append((name, args))

    def on_add_car(self, car: Car, position: LanePosition,
                   destination: OrientedWayPosition):
        """Add car with given position and destination."""
        ready = INDEX.simulation.ready_buffer
        target = INDEX.simulation.target_buffer
        car.place_at(position, ready)
        car.set_destination(destination, ready, target)
        self.agents[car] = Factory.create_car(self.agents_parent, car)

    def on_focus(self, car: Car):
        """Focus on given agent and pause simulation."""
        self.camera.unfollow()
        self.camera.focus.set_x(car.position.x)
        self.camera.focus.set_y(car.position.y)
        self._simulation_speed = 0

    def on_follow(self, car: Car):
        """Follow agent with camera."""
        car_np = self.agents.get(car, None)
        if car_np is not None:
            self.camera.follow(car_np)

    def on_network_entities_changed(self):
        """Update network entities."""
        self.update_network_entities()
        for node_path in self.roads.values():
            node_path.node().collect()

    def on_removed_car(self, car: Car):
        """Remove the car actor when car is removed from simulation."""
        node_path = self.agents.pop(car, None)
        if node_path is not None:
            self.camera.unfollow(node_path)
            node_path.remove_node()

    def update_agents(self):
        """Update agent actors."""
        for agent in INDEX.simulation.active:
            position = agent.position
            node_path = self.agents[agent]
            node_path.set_pos(position.x, position.y, 0.0)
            if agent.direction_changed:
                node_path.look_at(*(position + agent.direction), 0.0)

    def _build_on_screen_text(self):
        aspect = 1.0 / p3d.ASPECT2D.get_scale()[0]

        simspeed_text = TextNode('simulation_speed_text')
        simspeed_text.text = ''
        simspeed_text.slant = 0.3
        simspeed_text.text_scale = 0.05
        simspeed_text.shadow = -0.005, 0.0025
        simspeed_text.shadow_color = 0, 0, 0, 1
        simspeed_text_np = p3d.ASPECT2D.attach_new_node(simspeed_text)
        simspeed_text_np.set_pos(-0.95 * aspect, -0.0, -0.91)
        self.simulation_speed_text = simspeed_text

        time_text = TextNode('time_text')
        time_text.text = ''
        time_text.text_scale = 0.12
        time_text.shadow = -0.005, 0.005
        time_text.shadow_color = 0, 0, 0, 1
        time_text_np = p3d.ASPECT2D.attach_new_node(time_text)
        time_text_np.set_pos(-0.95 * aspect, -0.0, -0.85)
        self.time_text = time_text

        self._update_simulation_speed_text()

    def _update_time_text(self):
        time_text = time_string(INDEX.simulation.time)
        if self.time_text.text != time_text:
            self.time_text.text = time_text

    def _update_simulation_speed_text(self):
        simspeed = self.simulation_speed
        if simspeed == 0:
            simspeed_text = 'paused'
        elif simspeed == SPEED_STEPS:
            simspeed_text = ''
        else:
            simspeed_text = f'x{simspeed / SPEED_STEPS}'.strip('.0')
        if self.simulation_speed_text.text != simspeed_text:
            self.simulation_speed_text.text = simspeed_text


def log_config():
    """Initialize log configuration."""
    log.basicConfig(format='%(levelname)s: %(message)s',
                    level=log.DEBUG if __debug__ else log.INFO)


def panda3d_config():
    """Initialize Panda3D global configurations."""
    p3d.TASK_MGR.remove('audioLoop')
    p3d.TASK_MGR.remove('collisionLoop')
    # print(task_mgr)  # to print all tasks

    p3d.RENDER.set_antialias(AntialiasAttrib.M_auto)
    if ConfigVariableBool('use-shaders', False):
        p3d.RENDER.set_shader_auto()


def init_index(name: str):
    """Initialize index from given index name."""
    INDEX.load(name)
    INDEX.register_updates = True
