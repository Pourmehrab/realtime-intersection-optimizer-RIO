####################################
# File name: signal.py             #
# Author: Mahmoud Pourmehrab       #
# Email: mpourmehrab@ufl.edu       #
# Last Modified: Apr/06/2018       #
####################################

import numpy as np

np.random.seed(2018)

from src.input.data import get_conflict_dict, get_phases, get_signal_params
from src.optional.enum_phases import phenum


class Signal:
    SAT = 1  # saturation headway (also in trj.py)

    def __init__(self, inter_name, allowable_phases, num_lanes):
        '''
        - sequence keeps the sequence of phases to be executed from 0
        - green_dur keeps the amount of green allocated to each phase
        - yellow and all-red is a fix amount at the end of all phases (look at class variables)
        - start keeps the absolute time (in sec) when each phase starts

        * SPaT starts executing from 0 to end of each list

        - schedule keeps the earliest departures at the stop bars of each lane and gets updated when a signal decision
         goes permanent. It is made by a dictionary of arrays (key is lane, value is sorted earliest departures).

        '''

        self.name = inter_name

        self._y, self._ar, self.min_green, self.max_green = get_signal_params(inter_name)
        self._ts_min, self._ts_max = self.min_green + self._y + self._ar, self.max_green + self._y + self._ar
        self._ts_diff = self.max_green - self.min_green

        self._set_lane_lane_incidence(num_lanes)
        self._set_phase_lane_incidence(num_lanes)
        self._allowable_phases = allowable_phases
        self.SPaT_sequence, self.SPaT_green_dur, self.SPaT_start, self.SPaT_end = [set(allowable_phases).pop()], [0], \
                                                                                  [0], [self._y + self._ar]

        self._critical_volumes = np.zeros(len(allowable_phases), dtype=float)

    def _set_lane_lane_incidence(self, num_lanes):
        '''
            This converts a dictionary of form:
            key is a lane : vals are lanes that are in conflict with key (note numbering starts from 1 not 0)
            to lli which includes the conflict matrix |L|x|L| where element ij is 1 if i and j are conflicting movements

        :return:
        '''
        # gets the conflict dictionary for this intersection
        conf_dict = get_conflict_dict(self.name)

        # lane-lane incidence dictionary (lane: set of lanes in conflict with lane)
        self._lane_lane_incidence = {l: set([]) for l in range(num_lanes)}

        # the whole following loop makes lanes zero-based
        for l, conf in conf_dict.items():
            for j in conf:
                self._lane_lane_incidence[l - 1].add(j - 1)  # these are conflicting lanes

    def _set_phase_lane_incidence(self, num_lanes):
        phase_lane_incidence_one_based = get_phases(self.name)
        if phase_lane_incidence_one_based is None:
            phase_lane_incidence_one_based = phenum(num_lanes, self._lane_lane_incidence, self.name)

        self._pli = {p: [] for p in range(len(phase_lane_incidence_one_based))}

        # the whole following loop makes lanes and phases zero-based
        for l, conf in phase_lane_incidence_one_based.items():
            for j in conf:
                self._pli[l - 1].append(j - 1)  # these are lanes that belong to this phase

    def lane_in_phase(self, lane, phase_indx):
        if lane in self._pli[phase_indx]:
            return True
        else:
            return False

    def get_next_phase_indx(self, phase_indx, lane):
        new_phase_index = phase_indx + 1
        while new_phase_index < len(self.SPaT_sequence):
            if self.lane_in_phase(lane, new_phase_index):
                return new_phase_index
            else:
                new_phase_index += 1
        return -1  # this means no next phase serves this lane

    def enqueue(self, phase, actual_green):
        '''
        makes a signal decision permanent (append a phase and its green to the end of signal array)
        # SPaT decision is the sequence and green duration of phases

        :param phase: phase to be added
        :param actual_green: green duration of that phase
        :return:
        '''
        if self.SPaT_sequence[-1] == phase:  # extend this phase
            self.SPaT_end[-1] = self.SPaT_start[-1] + actual_green + self._y + self._ar
            print('*** Phase {:d} Extended (ends @ {:2.2f} sec)'.format(self.SPaT_sequence[-1], self.SPaT_end[-1]))
        else:  # append a new phase
            self.SPaT_sequence += [phase]
            self.SPaT_green_dur += [actual_green]
            self.SPaT_start += [self.SPaT_end[-1]]
            self.SPaT_end += [self.SPaT_start[-1] + actual_green + self._y + self._ar]
            print('*** Phase {:d} Appended (ends @ {:2.2f} sec)'.format(phase, self.SPaT_end[-1]))

    def set_critical_volumes(self, volumes):
        '''
        not used in GA since the phasing configuration is unknown prior to cycle length formula
        that is derived from time budget concept
        '''
        self._critical_volumes = np.array([max(volumes[self._pli[phase]]) for phase in self._allowable_phases])

    def flush_upcoming_SPaT(self):
        if len(self.SPaT_sequence) > 1:
            self.SPaT_sequence = [self.SPaT_sequence[0]]
            self.SPaT_green_dur = [self.SPaT_green_dur[0]]
            self.SPaT_start = [self.SPaT_start[0]]
            self.SPaT_end = [self.SPaT_end[0]]

        print('*** SPaT Flushed')

    def update_STaT(self, t):
        while len(self.SPaT_end) > 1 and t >= self.SPaT_end[0]:
            print('*** Phase {:d} Purged'.format(self.SPaT_sequence[0]))
            del self.SPaT_sequence[0]
            del self.SPaT_green_dur[0]
            del self.SPaT_start[0]
            del self.SPaT_end[0]

    def reset(self):
        '''
        Reset signal for the next scenario
        Note it assumes next scenario is still on the same test intersection under same yellow and all-red
        Therefore it only resets the SPaT variables
        '''
        self.SPaT_sequence, self.SPaT_green_dur, self.SPaT_start, self.SPaT_end = [set(self._allowable_phases).pop()], [
            0], [0], [self._y + self._ar]


# -------------------------------------------------------
# Genetic Algorithms
# -------------------------------------------------------
from sortedcontainers import SortedDict


class GA_SPaT(Signal):
    '''
    Based on the paper submitted to XXXXX (April 2018)

    Assumptions:
    - The sequence and duration is decided optimally by a Genetic Algorithms
    - The trajectories are computed using:
        - Gipps car following model for conventional vehicles
        - Polynomial degree k area under curve minimization for Lead/Follower AVs

    :param allowable_phases: subset of all possible phases is used (no limitation but it should cover all movements)
    '''

    # do not include more than this in a phase (is exclusive of last: 1,2, ..., MAX_PHASE_LENGTH-1)
    MAX_PHASE_LENGTH = 5

    POPULATION_SIZE = 20
    MAX_ITERATION_PER_PHASE = 10
    CROSSOVER_SIZE = 10

    def __init__(self, inter_name, allowable_phases, num_lanes):
        super().__init__(inter_name, allowable_phases, num_lanes)
        # this dictionary hashes the bad sequences to avoid reevaluating them
        self._bad_sequences = set([])

    def solve(self, lanes, critical_volume_ratio):
        '''
        This runs GA
        the key to the sorted dict is integer part of the travel time times 100
        todo define vars
        '''

        best_SPaT = {'phase_seq': tuple(), 'time_split': tuple(), 'badness_measure': 9999}

        # correct max phase length in case goes above the range
        max_phase_length = min(len(self._allowable_phases), self.MAX_PHASE_LENGTH)

        for phase_length in range(1, max_phase_length):

            population = SortedDict({})

            cycle_length = self.get_optimal_cycle_length(critical_volume_ratio, phase_length)

            # population keeps individuals in each row, and:
            #   - the sequence in the first columns up to the phase size (first block)
            #   - the timings in the second block of columns
            #   - the metrics in the third block of columns up to the number of metrics

            half_max_indx = int(phase_length / 2)  # for crossover

            # POPULATE THE FIRST GENERATION
            for individual in range(self.POPULATION_SIZE):
                # set the individuals
                phase_seq = self.mutate_seq(phase_length)
                time_split = self.mutate_timing(cycle_length, phase_length)
                # evaluate the fitness function
                badness = int(np.random.rand() * 100)  # todo connect the traj optimizer

                population[badness] = {'phase_seq': phase_seq, 'time_split': time_split}

            # does GA operations in-place
            if phase_length > 1:  # need at least two phases
                for i in range(self.MAX_ITERATION_PER_PHASE):
                    for j in range(self.CROSSOVER_SIZE):
                        # ELITE SELECTION
                        # in the next round, the top ones will automatically be removed when crossover
                        sorted_list = list(population.keys())

                        # CROSSOVER
                        del population[sorted_list[-1]]  # delete the worst member and then add one
                        parents_indx = np.random.choice(sorted_list[:-1], 2, replace=False)
                        phase_seq, time_split = self.cross_over(population[parents_indx[0]],
                                                                population[parents_indx[1]],
                                                                phase_length, half_max_indx)
                        # evaluate the fitness function
                        badness = int(np.random.rand() * 100)  # todo connect the traj optimizer
                        population[badness] = {'phase_seq': phase_seq, 'time_split': time_split}

            sorted_list = list(population.keys())
            best_temp_indiv = sorted_list[0]
            if best_SPaT['badness_measure'] > best_temp_indiv:  # this phase length has the fittest so far
                best_SPaT['badness_measure'] = int(best_temp_indiv)
                best_SPaT['phase_seq'] = tuple(population[best_temp_indiv]['phase_seq'])
                best_SPaT['time_split'] = tuple(population[best_temp_indiv]['time_split'])

        for indx, phase in enumerate(best_SPaT['phase_seq']):
            self.enqueue(int(phase), best_SPaT['time_split'][indx] - self._y - self._ar)

    def get_optimal_cycle_length(self, critical_volume_ratio, phase_length):
        '''
        hardcoded max/min cycle lengths here
        refer to HCM 2010 for values
        '''
        cycle_length = (phase_length * self._ar) / (1 - critical_volume_ratio)
        if cycle_length < 60:
            return 60
        elif cycle_length > 150:
            return 150
        else:
            return cycle_length

    def mutate_seq(self, phase_length):
        seq = tuple(np.random.choice(self._allowable_phases, phase_length, replace=False))
        return seq
        # todo: if two same phases follow each other, re-sample carefully with replacement

    def mutate_timing(self, cycle_length, phase_length):
        '''
        Creates the random phase split
        Valid timing should respect the min/max green requirement unless it conflicts with the
        cycle length which in that case we should adjust the maximum green to avoid the slack
        in time
        '''
        if phase_length == 1:
            return np.array([cycle_length])
        else:
            # under worst case all except one phase get minimum green
            #  check if in that case still we can specify the last phase
            min_G_max = cycle_length - (phase_length - 1) * self._ts_min
            if min_G_max >= self._ts_max:
                diff = self._ts_diff
            else:
                diff = min_G_max - self._ts_min

            # create vector of random numbers drawn from uniform distribution
            rand_vec = np.random.rand(phase_length)
            # scale it in a way all sum to the cycle length
            rand_vec *= ((cycle_length - phase_length * self._ts_min) / diff) / rand_vec.sum()

            time_split = np.array([self._ts_min + rand_vec[i] * diff for i in range(phase_length)],
                                  dtype=float)
            return tuple(time_split)

    def cross_over(self, left_parent, right_parent, phase_length, half_max_indx):
        phase_seq = tuple([left_parent['phase_seq'][i] if i < half_max_indx else right_parent['phase_seq'][i]
                           for i in range(phase_length)])
        time_split = tuple([0.5 * (left_parent['time_split'][i] + right_parent['time_split'][i])
                            for i in range(phase_length)])
        # todo: if two same phases follow each other, re-sample carefully with replacement
        return phase_seq, time_split


# -------------------------------------------------------
# min cost flow method
# -------------------------------------------------------

# from ortools.graph import pywrapgraph
#
# todo: replace ortools with CPLEX solver
#  Example at: https://www.ibm.com/support/knowledgecenter/SSSA5P_12.7.1/ilog.odms.cplex.help/CPLEX/UsrMan/topics/cont_optim/network/05_eg_Interactive_prob_descrip.html#User_manual.uss_solveNet.653116__User_manual.uss_solveNet.657919
# https://ibmdecisionoptimization.github.io/tutorials/html/Beyond_Linear_Programming.html

class MinCostFlow_SPaT(Signal):
    '''
    This class is meant to slove the min cost flow problem that is set up for phase selection

    Code is written in Python 3
    Install the list of packages in the Pipfile using PyEnv

    solver by Google: https://goo.gl/jFncvj

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

    def __init__(self, inter_name, allowable_phases, num_lanes, pli):

        super().__init__(inter_name, allowable_phases, num_lanes)

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


# -------------------------------------------------------
# enumeration
# -------------------------------------------------------

class Enumerate_SpaT(Signal):
    '''
    Gives all phases equal chance but picks the one with highest throughput
    Similar to GA functionality
    :return:
    '''

    def __init__(self, inter_name, allowable_phases, num_lanes):

        super().__init__(inter_name, allowable_phases, num_lanes)

    def solve(self, lanes, num_lanes, allowable_phases):
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
