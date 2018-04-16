####################################
# File name: signal.py             #
# Author: Mahmoud Pourmehrab       #
# Email: mpourmehrab@ufl.edu       #
# Last Modified: Apr/15/2018       #
####################################

import numpy as np

np.random.seed(2018)

import src.input.data as data_importer
from src.optional.enum_phases import phenum


class Signal:
    LAG = 1  # lag time from start of green when a vehicle can depart (also used in traj.py)

    def __init__(self, inter_name, num_lanes, min_headway, print_signal_detail):
        '''
        - sequence keeps the sequence of phases to be executed from 0
        - green_dur keeps the amount of green allocated to each phase
        - yellow and all-red is a fix amount at the end of all phases (look at class variables)
        - start keeps the absolute time (in sec) when each phase starts

        * SPaT starts executing from 0 to end of each list

        - schedule keeps the earliest departures at the stop bars of each lane and gets updated when a signal decision
         goes permanent. It is made by a dictionary of arrays (key is lane, value is sorted earliest departures).

        '''

        self._inter_name = inter_name
        self._num_lanes = num_lanes

        self._min_headway = min_headway

        self._set_lane_lane_incidence(num_lanes)
        self._set_phase_lane_incidence(num_lanes)

        self._print_signal_detail = print_signal_detail

    def _set_lane_lane_incidence(self, num_lanes):
        '''
            This converts a dictionary of form:
            key is a lane : vals are lanes that are in conflict with key (note numbering starts from 1 not 0)
            to lli which includes the conflict matrix |L|x|L| where element ij is 1 if i and j are conflicting movements

        :return:
        '''
        # gets the conflict dictionary for this intersection
        conf_dict = data_importer.get_conflict_dict(self._inter_name)

        # lane-lane incidence dictionary (lane: set of lanes in conflict with lane)
        self._lane_lane_incidence = {l: set([]) for l in range(num_lanes)}

        # the whole following loop makes lanes zero-based
        for l, conf in conf_dict.items():
            for j in conf:
                self._lane_lane_incidence[l - 1].add(j - 1)  # these are conflicting lanes

    def _set_phase_lane_incidence(self, num_lanes):
        phase_lane_incidence_one_based = data_importer.get_phases(self._inter_name)
        if phase_lane_incidence_one_based is None:  # todo add this to the readme
            phase_lane_incidence_one_based = phenum(num_lanes, self._lane_lane_incidence, self._inter_name)

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
            if self._print_signal_detail:
                print('*** Phase {:d} Extended (ends @ {:2.2f} sec)'.format(self.SPaT_sequence[-1], self.SPaT_end[-1]))
        else:  # append a new phase
            self.SPaT_sequence += [phase]
            self.SPaT_green_dur += [actual_green]
            self.SPaT_start += [self.SPaT_end[-1]]
            self.SPaT_end += [self.SPaT_start[-1] + actual_green + self._y + self._ar]
            if self._print_signal_detail:
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

        if self._print_signal_detail:
            print('** SPaT Flushed')

    def update_SPaT(self, t):
        while len(self.SPaT_end) > 1 and t >= self.SPaT_end[0]:
            if self._print_signal_detail:
                print('** Phase {:d} Purged.'.format(self.SPaT_sequence[0]))
            del self.SPaT_sequence[0]
            del self.SPaT_green_dur[0]
            del self.SPaT_start[0]
            del self.SPaT_end[0]

    def evaluate_badness(self, phase_seq, time_split, lanes, num_lanes):
        '''
        :return: 100 X average travel time by this SPaT
        '''
        mean_travel_time, throughput = 0.0, 0
        temporary_scheduled_arrivals = {lane: np.zeros(len(lanes.vehlist[lane]), dtype=float) for lane in
                                        range(num_lanes)}

        # since we flushed SPaT, no more/less than one phase is there
        phase = self.SPaT_sequence[0]
        start_green, end_yellow = self.SPaT_start[0] + self.LAG, self.SPaT_end[0] - self._ar

        # keeps index of last vehicle to be served by progressing SPaT
        first_unsrvd_indx = np.array([0 for lane in range(num_lanes)], dtype=int)
        served_vehicle_time = np.array([0.0 for lane in range(num_lanes)], dtype=float)
        # keeps index of last vehicle to be served by progressing SPaT
        last_vehicle_indx = np.array([len(lanes.vehlist[lane]) - 1 for lane in range(num_lanes)], dtype=int)

        any_unserved_vehicle = [bool(lanes.vehlist[lane]) for lane in range(num_lanes)]
        phase_indx, max_phase_indx = -1, len(phase_seq) - 1
        while any(any_unserved_vehicle):  # serve all with the current phasing
            for lane in self._pli[phase]:
                if any_unserved_vehicle[lane]:
                    while True:
                        veh_indx = first_unsrvd_indx[lane]
                        veh = lanes.vehlist[lane][veh_indx]

                        t_earliest = veh.earliest_arrival
                        # depending on if we are keeping the prev trajectory or not, schedule or reschedule departure
                        t_scheduled = max(t_earliest, start_green, served_vehicle_time[
                            lane] + self._min_headway) if veh.redo_trj() else veh.scheduled_arrival

                        if t_scheduled <= end_yellow:
                            travel_time = t_scheduled - veh.init_time
                            mean_travel_time = ((mean_travel_time * throughput) + travel_time) / (throughput + 1)
                            throughput += 1

                            served_vehicle_time[lane] = t_scheduled
                            temporary_scheduled_arrivals[lane][veh_indx] = t_scheduled
                            if first_unsrvd_indx[lane] < last_vehicle_indx[lane]:
                                first_unsrvd_indx[lane] += 1
                            else:
                                any_unserved_vehicle[lane] = False
                                break
                        else:
                            break
            if phase_indx < max_phase_indx:
                phase_indx += 1
                phase = phase_seq[phase_indx]
                start_green = end_yellow + self._ar
                end_yellow = start_green + time_split[phase_indx] - self._ar
            else:
                break
        return int(mean_travel_time * 100), throughput, any_unserved_vehicle, temporary_scheduled_arrivals

    def complete_unserved_vehicles(self, lanes, num_lanes, scheduled_arrivals, any_unserved_vehicle):
        '''
        Since we don't consider phases here, this only serves the rest of vehicles one at a time
        '''
        max_arrival_time = 0
        while any(any_unserved_vehicle):  # serve all with the current phasing
            for lane in range(num_lanes):
                if any_unserved_vehicle[lane]:
                    veh_indx, max_veh_indx = 0, len(lanes.vehlist[lane])
                    veh = lanes.vehlist[lane][veh_indx]
                    lead_arrival_time = veh.get_scheduled_arrival()

                    for veh_indx in range(1, max_veh_indx):

                        veh = lanes.vehlist[lane][veh_indx]
                        arrival_time = veh.get_scheduled_arrival()

                        if arrival_time < lead_arrival_time:  # if not set through GA, arrival_time is 0.0
                            arrival_time = max(lead_arrival_time, max_arrival_time) + self._min_headway
                            max_arrival_time = max(arrival_time, max_arrival_time)

                            # veh.set_scheduled_arrival(arrival_time)
                            scheduled_arrivals[lane][veh_indx] = arrival_time
                any_unserved_vehicle[lane] = False

        return scheduled_arrivals

    def reset(self):
        '''
        Reset signal for the next scenario
        Note it assumes next scenario is still on the same test intersection under same yellow and all-red
        Therefore it only resets the SPaT variables
        Not important if the phase index even does not exists because the zero green duration removes this
        instantly in the next iteration.
        '''
        self.SPaT_sequence, self.SPaT_green_dur, self.SPaT_start, self.SPaT_end = [0], [
            0], [0], [self._y + self._ar]


# -------------------------------------------------------
# Pre-timed Signal Control
# -------------------------------------------------------
class Pretimed(Signal):
    NUM_CYCLES = 4  # keeps this many cycles in queue (at least 2)
    '''
    Assumptions:
    - The sequence and duration is pre-determined
    '''

    def __init__(self, inter_name, num_lanes, min_headway, print_signal_detail):
        super().__init__(inter_name, num_lanes, min_headway, print_signal_detail)

        pretimed_signal_plan = data_importer.get_pretimed_parameters(inter_name)
        self._phase_seq = pretimed_signal_plan['phase_seq']
        self._green_dur = pretimed_signal_plan['green_dur']
        self._y, self._ar = pretimed_signal_plan['yellow'], pretimed_signal_plan['all-red']

        # add a dummy phase to initiate (note this is the last phase in the sequence to make the order right)
        self.SPaT_sequence, self.SPaT_green_dur, self.SPaT_start, self.SPaT_end = [self._phase_seq[-1]], [0], \
                                                                                  [0], [self._y + self._ar]

        for cycle in range(self.NUM_CYCLES):
            for indx, phase in enumerate(self._phase_seq):
                self.enqueue(int(phase), self._green_dur[indx])

    def solve(self, lanes, num_lanes):
        '''
        The phases sequence is exactly as the allowable phases
        this simply adds a cycle once it drops by one
        Next it provides the departure schedule to be used for trajectory optimization
        '''
        if len(self.SPaT_sequence) // len(self._phase_seq) < self.NUM_CYCLES - 1:
            for indx, phase in enumerate(self._phase_seq):
                self.enqueue(int(phase), self._green_dur[indx])

        # regardless of change in future SPaT, the following gets the arrival schedule
        # scheduled_arrivals = {lane: np.zeros(len(lanes.vehlist[lane]), dtype=float) for lane in
        #                       range(num_lanes)}

        badness, throughput, any_unserved_vehicle, scheduled_arrivals = self.evaluate_badness(self.SPaT_sequence,
                                                                                              self.SPaT_green_dur,
                                                                                              lanes, num_lanes)

        if any(any_unserved_vehicle):
            scheduled_arrivals = self.complete_unserved_vehicles(lanes, num_lanes, scheduled_arrivals,
                                                                 any_unserved_vehicle)
        lanes.set_all_scheduled_arrival(scheduled_arrivals)

    def reset(self):
        # add a dummy phase to initiate (note this is the last phase in the sequence to make the order right)
        self.SPaT_sequence, self.SPaT_green_dur, self.SPaT_start, self.SPaT_end = [self._phase_seq[-1]], [0], \
                                                                                  [0], [self._y + self._ar]
        for cycle in range(self.NUM_CYCLES):
            for indx, phase in enumerate(self._phase_seq):
                self.enqueue(int(phase), self._green_dur[indx])


# -------------------------------------------------------
# Genetic Algorithms
# -------------------------------------------------------
from sortedcontainers import SortedDict
from copy import deepcopy


class GA_SPaT(Signal):
    '''
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

    ACCURACY_OF_BADNESS_MEASURE = 100  # this is 10**(number of decimals we want to keep)

    def __init__(self, inter_name, allowable_phases, num_lanes, min_headway, print_signal_detail):
        super().__init__(inter_name, num_lanes, min_headway, print_signal_detail)

        self._allowable_phases = allowable_phases

        self._y, self._ar, self.min_green, self.max_green = data_importer.get_signal_params(inter_name)
        self._ts_min, self._ts_max = self.min_green + self._y + self._ar, self.max_green + self._y + self._ar
        self._ts_diff = self.max_green - self.min_green

        # add a dummy phase to initiate
        self.SPaT_sequence, self.SPaT_green_dur, self.SPaT_start, self.SPaT_end = [set(allowable_phases).pop()], [0], \
                                                                                  [0], [self._y + self._ar]

    def solve(self, lanes, critical_volume_ratio, num_lanes):
        '''
        This runs GA
        the key to the sorted dict is integer part of the travel time times 10
        todo define vars
        '''

        self.flush_upcoming_SPaT()  # todo make sure of the effect

        best_SPaT = {'phase_seq': tuple(), 'time_split': tuple(), 'badness_measure': 9999}
        best_all_served = bool()

        # correct max phase length in case goes above the range
        max_phase_length = min(len(self._allowable_phases), self.MAX_PHASE_LENGTH)

        for phase_length in range(1, max_phase_length):

            population = SortedDict({})

            cycle_length = self.get_optimal_cycle_length(critical_volume_ratio, phase_length)

            # population keeps individuals in each row, and:
            #   - the sequence in the first columns up to the phase size (first block)
            #   - the timings in the second block of columns
            #   - the metrics in the third block of columns up to the number of metrics

            half_max_indx = phase_length // 2  # for crossover

            # POPULATE THE FIRST GENERATION
            for individual in range(self.POPULATION_SIZE):
                # set the individuals
                phase_seq = self.mutate_seq(phase_length)
                time_split = self.mutate_timing(cycle_length, phase_length)
                # evaluate the fitness function
                badness, throughput, any_unserved_vehicle, temporary_scheduled_arrivals = self.evaluate_badness(
                    phase_seq, time_split, lanes,
                    num_lanes)

                population[badness] = {'phase_seq': phase_seq, 'time_split': time_split}

            # does GA operations in-place
            if phase_length > 1:
                for i in range(self.MAX_ITERATION_PER_PHASE):
                    for j in range(self.CROSSOVER_SIZE):
                        # ELITE SELECTION
                        # in the next round, the top ones will automatically be removed when crossover
                        sorted_list = list(population.keys())

                        if len(population) > 3:
                            # CROSSOVER
                            del population[sorted_list[-1]]  # delete the worst member and then add one
                            parents_indx = np.random.choice(sorted_list[:-1], 2, replace=False)
                            phase_seq, time_split = self.cross_over(population[parents_indx[0]],
                                                                    population[parents_indx[1]],
                                                                    phase_length, half_max_indx)
                            # evaluate the fitness function
                            badness, throughput, any_unserved_vehicle, temporary_scheduled_arrivals = self.evaluate_badness(
                                phase_seq, time_split,
                                lanes,
                                num_lanes)
                            population[badness] = {'phase_seq': phase_seq, 'time_split': time_split}

            sorted_list = list(population.keys())
            best_temp_indiv = sorted_list[0]
            if best_SPaT['badness_measure'] > best_temp_indiv:  # this phase length has the fittest so far
                best_SPaT['badness_measure'] = int(best_temp_indiv)
                best_SPaT['phase_seq'] = tuple(population[best_temp_indiv]['phase_seq'])
                best_SPaT['time_split'] = tuple(population[best_temp_indiv]['time_split'])

                best_scheduled_arrivals = deepcopy(temporary_scheduled_arrivals)
                best_all_served = any_unserved_vehicle

        for indx, phase in enumerate(best_SPaT['phase_seq']):
            self.enqueue(int(phase), best_SPaT['time_split'][indx] - self._y - self._ar)

        if any(any_unserved_vehicle):
            best_scheduled_arrivals = self.complete_unserved_vehicles(lanes, num_lanes, best_scheduled_arrivals,
                                                                      any_unserved_vehicle)
        lanes.set_all_scheduled_arrival(best_scheduled_arrivals)

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
        note each timing is between g_min+y+ar and g_max+y+ar
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
# ACTUATED SIGNAL CONTROL
# -------------------------------------------------------
class ActuatedControl(Signal):

    def __init__(self, inter_name, allowable_phases, num_lanes):
        super().__init__(inter_name, allowable_phases, num_lanes)


# -------------------------------------------------------
# MIN COST FLOW MODEL
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
                                time_phase_ends = max(lanes.vehlist[lane][veh_indx].earlst,
                                                      time_phase_ends + self._min_headway)

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
