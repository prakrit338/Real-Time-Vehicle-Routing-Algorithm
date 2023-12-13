from future import print_function
from six.moves import xrange
from ortools.constraint_solver import pywrapcp
from ortools.constraint_solver import routing_enums_pb2
from timeit import default_timer as timer
import random

time_taken = None

class Vehicle():
    def __init__(self):
        self._capacity = 15
        self._speed = 5 * 60 / 3.6

    @property
    def capacity(self):
        return self._capacity

    @property
    def speed(self):
        return self._speed

class CityBlock():
    @property
    def width(self):
        return 228 / 2

    @property
    def height(self):
        return 80

class DataProblem():
    def __init__(self):
        self._vehicle = Vehicle()
        self._num_vehicles = 4
        locations = [
            (4, 4),  # depot
            (2, 0), (8, 0),
            (0, 1), (1, 1),
            (5, 2), (7, 2),
            (3, 3), (6, 3),
            (5, 5), (8, 5),
            (1, 6), (2, 6),
            (3, 7), (6, 7),
            (0, 8), (7, 8)
        ]
        city_block = CityBlock()
        self._locations = [(loc[0] * city_block.width, loc[1] * city_block.height) for loc in locations]
        self._depot = 0
        self._demands = [0,  # depot
                         1, 1,
                         2, 4,
                         2, 4,
                         8, 8,
                         1, 2,
                         1, 2,
                         4, 4,
                         8, 8]
        self._time_windows = [(0, 0),
                              (75, 85), (75, 85),
                              (60, 70), (45, 55),
                              (0, 8), (50, 60),
                              (0, 10), (10, 20),
                              (0, 10), (75, 85),
                              (85, 95), (5, 15),
                              (15, 25), (10, 20),
                              (45, 55), (30, 40)]

    @property
    def vehicle(self):
        return self._vehicle

    @property
    def num_vehicles(self):
        return self._num_vehicles

    @property
    def locations(self):
        return self._locations

    @property
    def num_locations(self):
        return len(self.locations)

    @property
    def depot(self):
        return self._depot

    @property
    def demands(self):
        return self._demands

    @property
    def time_per_demand_unit(self):
        return 5

    @property
    def time_windows(self):
        return self._time_windows

def manhattan_distance(position_1, position_2):
    return abs(position_1[0] - position_2[0]) + abs(position_1[1] - position_2[1])

class CreateDistanceEvaluator(object):
    def __init__(self, data):
        self._distances = {}
        for from_node in xrange(data.num_locations):
            self._distances[from_node] = {}
            for to_node in xrange(data.num_locations):
                if from_node == to_node:
                    self._distances[from_node][to_node] = 0
                else:
                    self._distances[from_node][to_node] = manhattan_distance(data.locations[from_node],
                                                                             data.locations[to_node])

    def distance_evaluator(self, from_node, to_node):
        return self._distances[from_node][to_node]

class CreateDemandEvaluator(object):
    def __init__(self, data):
        self._demands = data.demands

    def demand_evaluator(self, from_node, to_node):
        del to_node
        return self._demands[from_node]

def add_capacity_constraints(routing, data, demand_evaluator):
    capacity = "Capacity"
    routing.AddDimension(demand_evaluator, 0, data.vehicle.capacity, True, capacity)

class CreateTimeEvaluator(object):
    @staticmethod
    def service_time(data, node):
        return data.demands[node] * data.time_per_demand_unit

    @staticmethod
    def travel_time(data, from_node, to_node):
        if from_node == to_node:
            travel_time = 0
        else:
            travel_time = manhattan_distance(data.locations[from_node],
                                             data.locations[to_node]) / data.vehicle.speed
        return travel_time

    def __init__(self, data):
        self._total_time = {}
        for from_node in xrange(data.num_locations):
            self._total_time[from_node] = {}
            for to_node in xrange(data.num_locations):
                if from_node == to_node:
                    self._total_time[from_node][to_node] = 0
                else:
                    self._total_time[from_node][to_node] = int(self.service_time(data, from_node) +
                                                               self.travel_time(data, from_node, to_node))

    def time_evaluator(self, from_node, to_node):
        return self._total_time[from_node][to_node]

def add_time_window_constraints(routing, data, time_evaluator):
    time = "Time"
    horizon = 120
    routing.AddDimension(time_evaluator, horizon, False, time)
    time_dimension = routing.GetDimensionOrDie(time)
    for location_idx, time_window in enumerate(data.time_windows):
        if location_idx == 0:
            continue
        index = routing.NodeToIndex(location_idx)
        time_dimension.CumulVar(index).SetRange(time_window[0], time_window[1])
        routing.AddToAssignment(time_dimension.SlackVar(index))

    for vehicle_id in xrange(data.num_vehicles):
        index = routing.Start(vehicle_id)
        time_dimension.CumulVar(index).SetRange(data.time_windows[0][0], data.time_windows[0][1])
        routing.AddToAssignment(time_dimension.SlackVar(index))

class ConsolePrinter():
    def __init__(self, data, routing, assignment):
        self._data = data
        self._routing = routing
        self._assignment = assignment

    @property
    def data(self):
        return self._data

    @property
    def routing(self):
        return self._routing

    @property
    def assignment(self):
        return self._assignment

    def print(self):
        capacity_dimension = self.routing.GetDimensionOrDie('Capacity')
        time_dimension = self.routing.GetDimensionOrDie('Time')
        total_dist = 0
        total_time = 0
        for vehicle_id in xrange(self.data.num_vehicles):
            index = self.routing.Start(vehicle_id)
            plan_output = 'Route for vehicle {0}:\n'.format(vehicle_id)
            route_dist = 0
            while not self.routing.IsEnd(index):
                node_index = self.routing
