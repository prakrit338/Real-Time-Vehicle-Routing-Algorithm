from future import print_function
from collections import namedtuple 
from six.moves import xrange
from ortools.constraint_solver import pywrapcp
from ortools.constraint_solver import routing_enums_pb2
from timeit import default_timer as timer

Vehicle = namedtuple('Vehicle', ['capacity'])
CityBlock = namedtuple('CityBlock', ['width', 'height'])

class DataProblem():
    def __init__(self): 
        locations = [
            (4, 4), 
            (2, 0), (8, 0),  # order location
            (0, 1), (1, 1),
            (5, 2), (7, 2),
            (3, 3), (6, 3),
            (5, 5), (8, 5),
            (1, 6), (2, 6),
            (3, 7), (6, 7),
            (0, 8), (7, 8)
        ]
        city_block = CityBlock(width=228 / 2, height=80) 
        self._locations = [(loc[0] * city_block.width, loc[1] * city_block.height) for loc in locations] 
        self._demands = [0,  # depot
                         1, 1,
                         2, 4,
                         2, 4,
                         8, 8,
                         1, 2,
                         1, 2,
                         4, 4,
                         8, 8]

    @property
    def vehicle(self):
        return Vehicle(capacity=15)

    @property
    def num_vehicles(self):
        return 4

    @property
    def locations(self):
        return self._locations

    @property
    def num_locations(self):
        return len(self.locations)

    @property
    def depot(self):
        return 0

    @property
    def demands(self):
        return self._demands

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
                    self._distances[from_node][to_node] = manhattan_distance(data.locations[from_node], data.locations[to_node])

    def distance_evaluator(self, from_node, to_node):
        return self._distances[from_node][to_node]

class CreateDemandEvaluator(object): 
    def __init__(self, data):
        self._demands = data.demands

    def demand_evaluator(self, from_node, to_node): 
        del to_node
        return self._demands[from_node]

def add_capacity_constraints(routing, data, demand_evaluator): 
    capacity = 'Capacity'
    routing.AddDimension(demand_evaluator, 0, data.vehicle.capacity, True, capacity)

def print_solution(data, routing, assignment):
    print('Objective: {}'.format(assignment.ObjectiveValue()))
    total_distance = 0
    total_load = 0
    capacity_dimension = routing.GetDimensionOrDie('Capacity')
    for vehicle_id in xrange(data.num_vehicles):
        index = routing.Start(vehicle_id)
        plan_output = 'Route for vehicle {}:\n'.format(vehicle_id)
        distance = 0
        while not routing.IsEnd(index):
            load_var = capacity_dimension.CumulVar(index)
            plan_output += ' {} Load({}) -> '.format(routing.IndexToNode(index), assignment.Value(load_var))
            previous_index = index
            index = assignment.Value(routing.NextVar(index))
            distance += routing.GetArcCostForVehicle(previous_index, index, vehicle_id)
            load_var = capacity_dimension.CumulVar(index)
            plan_output += ' {} Load({})\n'.format(routing.IndexToNode(index), assignment.Value(load_var))
        plan_output += 'Distance of the route: {}m\n'.format(distance)
        plan_output += 'Load of the route: {}\n'.format(assignment.Value(load_var))
        print(plan_output)
        total_distance += distance
        total_load += assignment.Value(load_var)
    print('Total Distance of all routes: {}m'.format(total_distance))
    print('Total Load of all routes: {}'.format(total_load))

def main():
    start = timer()
    data = DataProblem()
    routing = pywrapcp.RoutingModel(data.num_locations, data.num_vehicles, data.depot)
    distance_evaluator = CreateDistanceEvaluator(data).distance_evaluator
    routing.SetArcCostEvaluatorOfAllVehicles(distance_evaluator)
    demand_evaluator = CreateDemandEvaluator(data).demand_evaluator
    add_capacity_constraints(routing, data, demand_evaluator)
    search_parameters = pywrapcp.RoutingModel.DefaultSearchParameters()
    search_parameters.first_solution_strategy = (routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC)
    # pylint: disable=no-member
    assignment = routing.SolveWithParameters(search_parameters)
    print_solution(data, routing, assignment)
    end = timer()
    print("Time taken by the program:", end - start)

if __name__ == '__main__':
    main()
