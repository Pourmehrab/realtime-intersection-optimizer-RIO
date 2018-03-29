####################################
# File name: mcfopt.py             #
# Author: Mahmoud Pourmehrab       #
# Email: mpourmehrab@ufl.edu       #
# Last Modified: Feb/16/2018       #
####################################


'''
This class is meant to slove the min cost flow problem that is set up for phase selection

Code is written in Python 3
Install the list of packages in the Pipfile using PyEnv

solver by Google: https://goo.gl/jFncvj
'''

from ortools.graph import pywrapgraph
import numpy as np


class SigMinCostNet:
    '''
    NODES:
    Head of phase-selection arcs: from 0 to |p|-1
    Head of sink arcs: from |p| to 2|p|-1
    Head of lane-assignment arcs: from 2|p|+1 to 2|p|+|L|

    ARCS:
    Phase-selection arcs are from 0 to |p|-1
        cost 1 unit / cap of M
    Phase-activator arcs are from |p| to 2|p|-1
        cost 1 unit / cap of 1
    Sink arcs are from 2|p| to 3|p|-1
        cost 0 unit / cap of M
    Lane-assignment arcs are from 3|p| to len(A)
        cost 0 unit / cap of M

    (Note M is the large constant implemented as self.M)

    '''
    M = 999
    CMIN = 1

    def __init__(self, num_lanes, pli):
        num_ph = len(pli)
        self.num_lanes = num_lanes

        # to find out relevant nodes

        # add phase selection arcs
        self.start_nodes = [p for p in range(num_ph)]
        self.end_nodes = [num_ph + p for p in range(num_ph)]
        # find length od biggest phase
        max_ph_size = max([len(pli[p]) for p in range(num_lanes)])
        self.unit_costs = [int(self.CMIN + max_ph_size - sum(pli[p]) + 1) for p in range(num_ph)]
        self.capacities = [self.M for p in range(num_ph)]

        # add phase activator arcs
        self.start_nodes += [p for p in range(num_ph)]
        self.end_nodes += [num_ph + p for p in range(num_ph)]
        self.unit_costs += [self.CMIN for p in range(num_ph)]
        self.capacities += [1 for p in range(num_ph)]

        # add sink arcs
        self.start_nodes += [num_ph + p for p in range(num_ph)]
        self.end_nodes += [2 * num_ph for p in range(num_ph)]
        self.unit_costs += [0 for p in range(num_ph)]
        self.demand_nodes = [num_ph + p for p in range(num_ph)]
        self.capacities += [self.M for p in range(num_ph)]

        # add lane to phase arcs
        index = len(self.start_nodes) - 1
        for p, lanes in pli.items():
            for l in lanes:
                index += 1
                self.start_nodes.append(2 * num_ph + 1 + l)
                self.end_nodes.append(p)
                self.unit_costs.append(0)
                self.capacities.append(self.M)

        self.supplies = [0 for n in range(2 * num_ph + self.num_lanes + 1)]

    def set_dem(self, lanes_demand):
        total_dem = 0
        for l, d in enumerate(lanes_demand):
            total_dem -= d
            self.supplies[l - self.num_lanes] = d

        self.supplies[-1 * self.num_lanes - 1] = total_dem

    def solve(self):
        min_cost_flow = pywrapgraph.SimpleMinCostFlow()

        # Add each arc.
        for i in range(0, len(self.start_nodes)):
            min_cost_flow.AddArcWithCapacityAndUnitCost(self.start_nodes[i], self.end_nodes[i],
                                                        self.capacities[i], self.unit_costs[i])

        # Add node supplies.
        for i in range(0, len(self.supplies)):
            min_cost_flow.SetNodeSupply(i, self.supplies[i])

        # Find the minimum cost flows
        # if min_cost_flow.Solve() == min_cost_flow.OPTIMAL:
        #     print('Minimum cost:', min_cost_flow.OptimalCost())
        #     print('')
        #     print('  Arc    Flow / Capacity  Cost')
        #     # for i in range(min_cost_flow.NumArcs()):
        #     for i in range(16):
        #         if min_cost_flow.Flow(i) > 0:
        #             cost = min_cost_flow.Flow(i) * min_cost_flow.UnitCost(i)
        #             print('%1s -> %1s   %3s  / %3s       %3s' % (
        #                 min_cost_flow.Tail(i),
        #                 min_cost_flow.Head(i),
        #                 min_cost_flow.Flow(i),
        #                 min_cost_flow.Capacity(i),
        #                 cost))
        # else:
        #     raise Exception('There was an issue with the min cost flow input.')

    def do_spat_decision(self, lanes, num_lanes, allowable_phases):
        '''
        Gives all phases equal chance but picks the one with highest throughput
        Similar to GA functionality
        :return:
        '''
        # the goal is to choose the sequence of SPaT which gives more throughput in less time
        self.flush_upcoming_SPaT()

        # keeps index of last vehicle to be served by progressing SPaT
        served_vehicle_indx = np.array([0 if bool(lanes.vehlist[lane]) else -1 for lane in range(num_lanes)],
                                       dtype=np.int)

        # keeps index of last vehicle to be served by progressing SPaT
        last_vehicle_indx = np.array([len(lanes.vehlist[lane]) - 1 for lane in range(num_lanes)], dtype=np.int)

        def all_not_served(a, b):
            for i in range(len(a)):
                if b[i] > -1 and a[i] <= b[i]:
                    return True
            return False

        while all_not_served(served_vehicle_indx, last_vehicle_indx):  # checks if SPaT did not serve all

            best_phase, best_phase_score, best_phase_green_dur = 0, 0, 0
            best_throughput = np.zeros(num_lanes, dtype=np.int)

            for phase in allowable_phases:  # gives all phases a chance

                temp_phase_score = 0
                temp_throughput = np.zeros(num_lanes, dtype=np.int)

                # check the length of phase not to exceed the max green
                start_time = self.SPaT_end[-1] if self.SPaT_sequence[-1] != phase else self.SPaT_start[-1]
                time_phase_ends = start_time + self.min_green if self.SPaT_sequence[-1] != phase else self.SPaT_end[-1]

                for lane in self._pli[phase]:

                    veh_indx = served_vehicle_indx[lane]
                    # check if the lane is not empty and there are vehicles
                    if last_vehicle_indx[lane] > -1 and veh_indx <= last_vehicle_indx[lane]:

                        while veh_indx <= last_vehicle_indx[lane] and lanes.vehlist[lane][
                            veh_indx].earlst - start_time <= self.max_green:
                            # count and time processing new vehicles
                            temp_throughput[lane] += 1
                            if lanes.vehlist[lane][veh_indx].earlst > time_phase_ends:
                                time_phase_ends = max(lanes.vehlist[lane][veh_indx].earlst, time_phase_ends + self.SAT)

                            veh_indx += 1

                # make it permanent if it's better than previous temporary SPaT
                temp_phase_score = 0 if time_phase_ends <= start_time else np.sum(temp_throughput) / (
                        time_phase_ends - start_time)

                if temp_phase_score > best_phase_score:
                    best_phase_score, best_phase = temp_phase_score, phase
                    best_phase_green_dur = time_phase_ends - start_time
                    best_throughput = temp_throughput

            if best_phase_score <= 0:  # vehicles are far away, assign a random phase the max green

                remaining_phases = set(allowable_phases) - {self.SPaT_sequence[-1]}
                self.enqueue(remaining_phases.pop(), self.max_green)  # pop gives a random phase which is new

            else:  # progress SPaT
                served_vehicle_indx += best_throughput
                self.enqueue(best_phase, max(self.min_green, best_phase_green_dur))
