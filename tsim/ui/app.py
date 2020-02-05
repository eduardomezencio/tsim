"""App class implementation, the graphic UI main class."""

from __future__ import annotations

import logging as log
from collections import deque
from functools import partial
from itertools import chain
from math import floor
from random import random, seed, shuffle
from typing import Dict, Tuple

from direct.task import Task
from panda3d.core import (AntialiasAttrib, ConfigVariableBool, NodePath,
                          RigidBodyCombiner, TextNode)

import tsim.ui.input as INPUT
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

FRAME_DURATION = 1 / 60
SPEED_STEPS = 4


class App:
    """Graphic UI application, using Panda3D."""

    time_text: TextNode
    network_entities: Dict[int, NodePath]
    agents: Dict[Car, NodePath]
    roads: Dict[Tuple[int, int], NodePath]

    def __init__(self, index_name: str):
        self._event_queue = deque()

        log_config()
        panda3d_config()

        init_index(index_name)

        INPUT.init()

        self.scene = NodePath('scene')
        self.agents_parent = NodePath('agents')
        self.agents_parent.reparent_to(self.scene)

        self.camera = Camera()
        self.world = World.create(self.scene, 10000, 16)
        self.sky = Sky(self.scene, self.camera)
        self.cursor = Cursor(self.world)
        self.grid = Grid(50.0, 1000.0, self.world, self.cursor.actor)

        self._build_on_screen_text()

        # self.roads = self.world.attach_new_node(PandaNode('roads'))
        # self.roads = self.world.attach_new_node(RigidBodyCombiner('roads'))
        self.network_entities = {}
        self.agents = {}
        self.roads = {}
        self.init_objects()
        for node_path in self.roads.values():
            node_path.node().collect()

        # TODO: Change to set simulation time when loading INDEX from file.
        INDEX.simulation.time = 6.5 * HOUR
        self.sky.set_time(8.0)

        self._frame_count = 0
        self._simulation_speed = 0

        p3d.BASE.accept('wheel_up',
                        partial(self.change_simulation_speed, 1))
        p3d.BASE.accept('wheel_down',
                        partial(self.change_simulation_speed, -1))

        p3d.BASE.accept('entities_changed', self.on_network_entities_changed)
        p3d.BASE.accept('new_agent', self.enqueue_event)
        p3d.BASE.accept('focus', self.focus)

        self.scene.reparent_to(p3d.RENDER)

        seed(1)
        self.generate_random_cars()

    def focus(self, *args):
        self.camera.focus.set_x(args[0].position.x)
        self.camera.focus.set_y(args[0].position.y)
        self._simulation_speed = 0

    def generate_random_cars(self, limit=500):
        lanes = list(chain.from_iterable(w.lanes for w in
                                         INDEX.get_all(of_type=Way)))[:limit]
        shuffle(lanes)
        for lane1, lane2 in window_iter(lanes):
            distance1 = random() * lane1.length
            distance2 = random() * lane2.length
            path = INDEX.path_map.path(lane1.oriented_way_position(distance1),
                                       lane2.oriented_way_position(distance2))
            if path is None:
                continue
            self.add_car(Car(), LanePosition(lane1, distance1),
                         lane2.oriented_way_position(distance2))

    @property
    def simulation_speed(self) -> int:
        """Get simulation speed from 0 to `SPEED_STEPS`."""
        return self._simulation_speed

    def change_simulation_speed(self, value: int):
        """Add value to the simulation speed."""
        value = self._simulation_speed + value
        self._simulation_speed = floor(max(0, min(value, SPEED_STEPS)))

    def run(self):
        """Start the main loop."""
        p3d.TASK_MGR.add(self.update)
        p3d.BASE.run()

    def update(self, _task: Task):
        """Update task, to run every frame."""
        self._frame_count += self._simulation_speed

        while self._frame_count >= SPEED_STEPS:
            self._frame_count -= SPEED_STEPS
            INDEX.simulation.update(FRAME_DURATION * self.simulation_speed
                                    / SPEED_STEPS)
            self.update_agents()

            while self._event_queue:
                event = self._event_queue.popleft()
                App.event_handlers[type(event[0])](self, *event)

        self.camera.update()
        self.sky.set_time(normalized_hours(INDEX.simulation.time))
        self.sky.update()
        self.cursor.update()
        self.grid.update()
        INPUT.clear()

        self._update_on_screen_text()

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

    def on_network_entities_changed(self):
        """Update network entities."""
        self.update_network_entities()
        for node_path in self.roads.values():
            node_path.node().collect()

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

    def init_objects(self):
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

    def enqueue_event(self, *args):
        """Enqueue event from panda3d messenger."""
        self._event_queue.append(args)

    def add_car(self, car: Car, position: LanePosition,
                destination: OrientedWayPosition):
        """Add car with given position and destination."""
        buffer = INDEX.simulation.ready_buffer
        car.place_at(position, buffer)
        car.set_destination(destination, buffer)
        self.agents[car] = Factory.create_car(self.agents_parent, car)

    def update_agents(self):
        """Update agent actors."""
        for agent in INDEX.simulation.active:
            position = agent.position
            node_path = self.agents[agent]
            node_path.set_pos(position.x, position.y, 0.0)
            if agent.direction_changed:
                node_path.look_at(*(position + agent.direction), 0.0)

    event_handlers = {Car: add_car}

    def _build_on_screen_text(self):
        time_text = TextNode('time_text')
        time_text.text = ''
        time_text.text_scale = 0.15
        time_text.shadow = -0.01, 0.01
        time_text.shadow_color = 0, 0, 0, 1
        time_text_np = p3d.ASPECT2D.attach_new_node(time_text)
        aspect = 1.0 / p3d.ASPECT2D.get_scale()[0]
        time_text_np.set_pos(-0.9 * aspect, -0.0, -0.8)
        self.time_text = time_text

    def _update_on_screen_text(self):
        time_text = time_string(INDEX.simulation.time)
        if self.time_text.text != time_text:
            self.time_text.text = time_text


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
