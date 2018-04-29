####################################
# File name: signal.py             #
# Author: Mahmoud Pourmehrab       #
# Email: mpourmehrab@ufl.edu       #
# Last Modified: Apr/24/2018       #
####################################

import numpy as np
from sortedcontainers import SortedDict
from copy import deepcopy
import data.data as data_importer

np.random.seed(2018)


class Signal:
    """
    The class serves the following goals:
        - Keeps the SPaT decision updated
        - Makes SPaT decisions through variety of control methods. For now it supports:
            - Pre-timed control
            - Genetic Algorithm
            - Min Cost Flow model

    Set the class variable ``LAG`` to the time (in seconds) that from start of green is not valid to schedule any
    departurs.

.. note::
    - ``LAG`` also is used in ``Trajectory()`` class. Set them consistent.
    - ``LARGE_NUM`` is a large number to initialize badness of alternatives in GA.


:Author:
    Mahmoud Pourmehrab <pourmehrab@gmail.com>
:Date:
    April-2018
    """
    LAG = 1
    LARGE_NUM = 999999

    def __init__(self, inter_name, num_lanes, min_headway, print_signal_detail):
        """
        Elements:
            - Sequence keeps the sequence of phases to be executed from 0
            - ``green_dur`` keeps the amount of green allocated to each phase
            - ``yellow`` and ``all-red`` is a fix amount at the end of all phases (look at class variables)
            - start keeps the absolute time (in seconds) when each phase starts

        Note: SPaT starts executing from index 0 to end of each list
        """
        self._inter_name = inter_name
        self._num_lanes = num_lanes

        self._min_headway = min_headway

        self._set_lane_lane_incidence(num_lanes)
        self._set_phase_lane_incidence(num_lanes)

        self._print_signal_detail = print_signal_detail

        self.first_unsrvd_indx = np.zeros(num_lanes, dtype=int)

    def _set_lane_lane_incidence(self, num_lanes):
        """
        This converts a dictionary of the form:
        key is a lane and value is *set* of lanes that are in conflict with key (note numbering starts from 1 not 0)
        to ``lane_lane_incidence`` which includes the conflict matrix :math:`|L|*|L|` where element :math:`ij`
        is 1 if :math:`i` and :math:`j` are
        conflicting movements

        :param num_lanes:
        """
        # gets the conflict dictionary for this intersection
        conf_dict = data_importer.get_conflict_dict(self._inter_name)

        # lane-lane incidence dictionary (lane: set of lanes in conflict with lane)
        self._lane_lane_incidence = {l: set([]) for l in range(num_lanes)}

        # the whole following loop makes lanes zero-based
        for l, conf in conf_dict.items():
            for j in conf:
                self._lane_lane_incidence[l - 1].add(j - 1)  # these are conflicting lanes

    def _set_phase_lane_incidence(self, num_lanes):
        """
        Sets the phase-phase incidence matrix of the intersection
        # todo automate phase enumerator
        :param num_lanes:
        :return:
        """
        phase_lane_incidence_one_based = data_importer.get_phases(self._inter_name)
        if phase_lane_incidence_one_based is None:  # todo add this to the readme
            phase_lane_incidence_one_based = phase_enumerator(num_lanes, self._lane_lane_incidence, self._inter_name)

        self._pli = {p: [] for p in range(len(phase_lane_incidence_one_based))}

        # the whole following loop makes lanes and phases zero-based
        for l, conf in phase_lane_incidence_one_based.items():
            for j in conf:
                self._pli[l - 1].append(j - 1)  # these are lanes that belong to this phase

    def append_extend_phase(self, phase, actual_green):
        """
        Append a phase to the SPaT (append a phase and its green to the end of signal array)
        Note SPaT decision is the sequence and green duration of phases

        :param phase: phase to be added
        :param actual_green: green duration of that phase

        """
        if self.SPaT_sequence[-1] == phase:  # extend this phase
            self.SPaT_end[-1] = self.SPaT_start[-1] + actual_green + self._y + self._ar
            if self._print_signal_detail:
                print('>-> Phase {:d} Extended (ends @ {:>2.1f} sec) >->'.format(self.SPaT_sequence[-1],
                                                                                 self.SPaT_end[-1]))
        else:  # append a new phase
            self.SPaT_sequence += [phase]
            self.SPaT_green_dur += [actual_green]
            self.SPaT_start += [self.SPaT_end[-1]]
            self.SPaT_end += [self.SPaT_start[-1] + actual_green + self._y + self._ar]
            if self._print_signal_detail:
                print('>>> Phase {:d} appended (ends @ {:>2.1f} sec)'.format(phase, self.SPaT_end[-1]))

    def set_critical_phase_volumes(self, volumes):
        """
        Not used in GA since the phasing configuration is unknown prior to cycle length formula
        that is derived from time budget concept

        .. warning::
            Do not call this on a signal method that does not take ``allowable_phases`` as input

        :param volumes:
        :return:
        """
        self._critical_phase_volumes = np.array([max(volumes[self._pli[phase]]) for phase in self._allowable_phases])

    def update_SPaT(self, time_threshold):
        """
        Performs two tasks to update SPaT based on the given clock:
            - Removes terminated phase (happens when the all-red is passed)
            - Checks for  SPaT to not get empty after being updated

        .. attention::
            - If all phases are getting purged, either make longer SPaT decisions or reduce the simulation steps.

        :param time_threshold: Normally the current clock of simulation or real-time in :math:`s`
        """
        if self.SPaT_end[-1] < time_threshold:
            raise Exception('If all phases get purged, SPaT becomes empty!')

        phase_indx, any_to_be_purged = 0, False
        while time_threshold >= self.SPaT_end[phase_indx]:
            any_to_be_purged = True
            phase_indx += 1

        if self._print_signal_detail and any_to_be_purged:
            print('<<< Phase(s) ' + ','.join(str(p) for p in self.SPaT_sequence[:phase_indx]) + ' expired')

        del self.SPaT_sequence[:phase_indx]
        del self.SPaT_green_dur[:phase_indx]
        del self.SPaT_start[:phase_indx]
        del self.SPaT_end[:phase_indx]

    def _flush_upcoming_SPaTs(self):
        """
        Just leaves the first SPaT and flushes the rest. One more severe variant to this is to even reduce the the green
        time of first phase.

        """
        if len(self.SPaT_sequence) > 1:
            self.SPaT_sequence = [self.SPaT_sequence[0]]
            self.SPaT_green_dur = [self.SPaT_green_dur[0]]
            self.SPaT_start = [self.SPaT_start[0]]
            self.SPaT_end = [self.SPaT_end[0]]

            if self._print_signal_detail:
                print('<<< Phase(s) ' + ','.join(str(p) for p in self.SPaT_sequence[1:]) + ' flushed.')

    def base_badness(self, lanes, num_lanes, max_speed):
        """
        This method aims to serve as many vehicles as possible given the available SPaT. Depending on the signal method, the set of current SPaT could be different. For example:

            - If called by ``Pretimed()`` solver, the current SPaT may include multiple phases as Pretimed SPaT never gets flushed.
            - If called by ``GA_SPaT()`` solver, since the SPaT gets flushed before calling. The goal is to serve as many vehicles with only the single current phase in SPaT.

        The condition to be served is to meet the following criteria:
            - Respect the minimum headway to the lead vehicle (if present)
            - Respect the initiation of green plus a lag time specified by LAG as a class variable
            - Respect the earliest available time at the stop bar controlled by the speed limit  acc/dec rates
            - Vehicle is allowed to acquire a new trajectory (``veh.redo_trj_allowed`` holds True)

        The method does not compute or return the badness metric since the it does not aim to change current phase and timing.

        It may only gets called once per each Signal solve call prior to computation of the new SPaTs.

        The schedule keeps the earliest departures at the stop bars of each lane and gets updated when a signal decision goes permanent. It is made by a dictionary of arrays (key is lane, value is sorted earliest departures).

        ``self.first_unsrvd_indx`` and setting the schedule of any possible served vehicles make the main result of this method. The ``self.first_unsrvd_indx`` will be used after this to avoid reserving and double-counting those already served with base SPaT. This also returns ``any_unserved_vehicle`` array that has True if any lane has vehicles that could not be unserved with base SPaT.

        .. note::
            - Since base SPaT never gets changed (for safety and practical reasons), any vehicle served by it has to get ``redo_trj_allowed`` value set to ``False``.
            - It is feasible that if fusion algorithm updates the info on this vehicle and wants an update on trajectory, it rolls back the ``redo_trj_allowed`` to be ``True``. However, this should be decided outside this method.
            - The reason that this does not return schedule of departures is because they are already set inside this method. Late, the set method skips these.
            - If a vehicle gets a schedule and has more than one trajectory point, the last index should reset to the first index so when the trajectory is set there would be two points.
            - all-red from the end and ``LAG`` time from the beginning of a phase are note utilizes by any vehicle.

        :param lanes:
        :param num_lanes:
        :param max_speed:
        :return: The ``self.first_unsrvd_indx`` array that keeps index off the first unserved vehicle in each lane, is initialized to zero before calling this method and gets updated by the end of this call. It also returns ``served_vehicle_time`` that shows the schedule
        """

        served_vehicle_time = np.zeros(num_lanes, dtype=np.float)
        any_unserved_vehicle = [0 <= lanes.last_vehicle_indx[lane] for lane in range(num_lanes)]

        for phase_indx, phase in enumerate(self.SPaT_sequence):
            start_green, end_yellow = self.SPaT_start[phase_indx] + self.LAG, self.SPaT_end[phase_indx] - self._ar
            if any(any_unserved_vehicle):  # serve all with the current phasing
                for lane in self._pli[phase]:
                    if any_unserved_vehicle[lane]:
                        for veh_indx in range(self.first_unsrvd_indx[lane], lanes.last_vehicle_indx[lane] + 1):
                            veh = lanes.vehlist[lane][veh_indx]

                            if veh.redo_trj_allowed:
                                t_earliest = veh.earliest_arrival
                                t_scheduled = max(t_earliest, start_green, served_vehicle_time[
                                    lane] + self._min_headway)
                                if t_scheduled < end_yellow:
                                    self.first_unsrvd_indx[lane] += 1
                                    veh.last_trj_point_indx = veh.first_trj_point_indx
                                    veh.set_scheduled_arrival(t_scheduled, 0, max_speed, lane, veh_indx,
                                                              self._print_signal_detail)  # since this is final
                                    served_vehicle_time[lane] = t_scheduled
                                    veh.redo_trj_allowed = False

                                else:
                                    break  # no more room in this phase (no point to continue)

                            else:  # next vehicle may want trajectory
                                served_vehicle_time[lane] = veh.scheduled_arrival
                                self.first_unsrvd_indx[lane] += 1

                        if self.first_unsrvd_indx[lane] > lanes.last_vehicle_indx[lane]:
                            any_unserved_vehicle[lane] = False

            else:
                break
        return any_unserved_vehicle

    def _schedule_unserved_vehicles(self, lanes, num_lanes, served_vehicle_time, any_unserved_vehicle):
        """
        Sometimes the base SPaT prior to running a ``solve()`` method does not serve all vehicles. However, vehicles
        require trajectory to be provided. One way to address this is to assign them the best temporal trajectory which
        only has some of general qualities necessary for continuation of program. In this method we do the followings
        to compute the ``departure times`` of such trajectories:

            - Without use of phases, schedule vehicles one after the other at minimum headway restricted by the
                saturation headway. This gives an overestimate of teh departure time since one vehicle gets served by
                intersection at a time, while having allowing to depart in phases let multiple simultaneous departures.
            - This may be called after a signal ``solve()`` method decided to complete those that did not get served.
            - Also this assumes min headway after green starts instead of ``LAG`` time which is a simplification.
            - If a vehicle gets a schedule and has more than one trajectory point, the last index should reset to the
                first index so when the trajectory is set there would be two points.


        .. warning:: Since the departure times are definitely temporal, DO NOT set ``redo_trj_allowed`` to ``False``.

        :param lanes:
        :param num_lanes:
        :param served_vehicle_time: includes schedule of departures for those served by base SPaT
        :param any_unserved_vehicle: `Has `False`` for the lane that has all vehicles scheduled through base SPaT and the ``solve()``, ``True`` otherwise.
        :return: ``served_vehicle_time`` that now includes the schedules of all vehicle except those served through base SPaT
        """
        max_arrival_time = 0.0
        for lane in range(num_lanes):
            if any_unserved_vehicle[lane]:
                lead_arrival_time = self.SPaT_end[-1]

                for veh_indx in range(self.first_unsrvd_indx[lane], lanes.last_vehicle_indx[lane] + 1):
                    veh = lanes.vehlist[lane][veh_indx]

                    arrival_time = max(lead_arrival_time, max_arrival_time) + self._min_headway
                    max_arrival_time = arrival_time

                    served_vehicle_time[lane][veh_indx] = arrival_time
                    veh.last_trj_point_indx = veh.first_trj_point_indx
                    veh.redo_trj_allowed = True  # just to make sure it's not False

                any_unserved_vehicle[lane] = False

        return served_vehicle_time

    def _set_non_base_scheduled_arrival(self, lanes, scheduled_arrivals, num_lanes, max_speed):
        """
        Sets the scheduled departure in the trajectory of the vehicle.

        .. note::
            - Departure schedule of those which were served by base SPaT is set in ``base_badness()`` and not here.

        :param lanes:
        :param scheduled_arrivals:
        :param num_lanes:
        :param max_speed: by default the departure speed is maximum allowable speed in :math:`m/s`
        """

        for lane in range(num_lanes):
            for veh_indx in range(self.first_unsrvd_indx[lane], lanes.last_vehicle_indx[lane] + 1):
                veh = lanes.vehlist[lane][veh_indx]
                if veh.redo_trj_allowed:
                    veh.set_scheduled_arrival(scheduled_arrivals[lane][veh_indx], 0, max_speed, lane, veh_indx,
                                              self._print_signal_detail)
                else:
                    continue


# -------------------------------------------------------
# Pre-timed Signal Control
# -------------------------------------------------------
class Pretimed(Signal):
    """
    .. note::
        Assumptions:
            - The sequence and duration are pre-determined
            - Cycle length is computed using the time budget concept in traffic flow theory
                * min and max of 60 and 120 seconds bound the *cycle length*
    .. warning::
        Must choose ``NUM_CYCLES`` at least 2.


    :Author:
        Mahmoud Pourmehrab <pourmehrab@gmail.com>
    :Date:
        April-2018
    """

    NUM_CYCLES = 2

    def __init__(self, inter_name, num_lanes, min_headway, print_signal_detail):
        """ Initialize the pretimed SPaT """

        super().__init__(inter_name, num_lanes, min_headway, print_signal_detail)

        pretimed_signal_plan = data_importer.get_pretimed_parameters(inter_name)
        self._phase_seq = pretimed_signal_plan['phase_seq']
        self._green_dur = pretimed_signal_plan['green_dur']
        self._y, self._ar = pretimed_signal_plan['yellow'], pretimed_signal_plan['all-red']

        # add a dummy phase to initiate (note this is the last phase in the sequence to make the order right)
        self.SPaT_sequence, self.SPaT_green_dur, self.SPaT_start, self.SPaT_end = [self._phase_seq[-1]], [0], \
                                                                                  [0], [self._y + self._ar]
        if self._print_signal_detail:
            print('>>> Phase {:d} appended (ends @ {:2.1f} sec)'.format(self._phase_seq[-1], self.SPaT_end[-1]))

        for cycle in range(self.NUM_CYCLES):
            for indx, phase in enumerate(self._phase_seq):
                self.append_extend_phase(int(phase), self._green_dur[indx])

    def solve(self, lanes, num_lanes, max_speed):
        """
        The phases sequence is exactly as the provided in ``data.py``. The flow is:
            #. First serves using the available SPaT
            #. This simply adds a cycle to SPaT if a cycle is terminated
            #. Serves unserved vehicles, if any present
            #. Next it provides the departure schedule

        .. note:: The ``scheduled_departures`` is made only to call ``complete_unserved_vehicles()``. It only stores
                    departures for those vehicles nit served bt base SPaT.

        :param lanes:
        :param num_lanes:
        :param max_speed:
        """

        self.first_unsrvd_indx = np.zeros(num_lanes, dtype=np.int)
        any_unserved_vehicle = self.base_badness(lanes, num_lanes, max_speed)

        if len(self.SPaT_sequence) // len(self._phase_seq) < self.NUM_CYCLES - 1:
            for indx, phase in enumerate(self._phase_seq):
                self.append_extend_phase(int(phase), self._green_dur[indx])
        if any(any_unserved_vehicle):
            scheduled_departures = {lane: np.zeros(len(lanes.vehlist[lane]), dtype=float) for lane in
                                    range(num_lanes)}
            scheduled_departures = self._schedule_unserved_vehicles(lanes, num_lanes,
                                                                    scheduled_departures,
                                                                    any_unserved_vehicle)
            self._set_non_base_scheduled_arrival(lanes, scheduled_departures, num_lanes, max_speed)


# -------------------------------------------------------
# Genetic Algorithms
# -------------------------------------------------------


class GA_SPaT(Signal):
    """
    The sequence and duration is decided optimally by a Genetic Algorithms


    :param allowable_phases: subset of all possible phases to be used.

    .. warning::
        - ``allowable_phases`` should cover all lanes or some would not get green.
        - ``allowable_phases`` should be zero-based unlike what is provided in ``data.py``

    :Author:
        Mahmoud Pourmehrab <pourmehrab@gmail.com>
    :Date:
        April-2018
    """
    # do not include more than this in a phase (is exclusive of last: 1,2, ..., MAX_PHASE_LENGTH-1)
    MAX_PHASE_LENGTH = 4

    POPULATION_SIZE = 20
    MAX_ITERATION_PER_PHASE = 10
    CROSSOVER_SIZE = 10

    ACCURACY_OF_BADNESS_MEASURE = 100  # this is 10**(number of decimals we want to keep)

    def __init__(self, inter_name, allowable_phases, num_lanes, min_headway, print_signal_detail):
        """

        :param inter_name:
        :param allowable_phases: zero-based subset of phases
        :type allowable_phases: tuple
        :param num_lanes:
        :param min_headway:
        :param print_signal_detail:
        """

        super().__init__(inter_name, num_lanes, min_headway, print_signal_detail)

        self._allowable_phases = allowable_phases

        self._y, self._ar, self.min_green, self.max_green = data_importer.get_signal_params(inter_name)
        self._ts_min, self._ts_max = self.min_green + self._y + self._ar, self.max_green + self._y + self._ar
        self._ts_diff = self.max_green - self.min_green

        # add a dummy phase to initiate
        self.SPaT_sequence, self.SPaT_green_dur, self.SPaT_start, self.SPaT_end = [set(allowable_phases).pop()], [0], \
                                                                                  [0], [self._y + self._ar]
        if self._print_signal_detail:
            print('>>> Phase {:d} appended (ends @ {:2.1f} sec)'.format(self.SPaT_sequence[-1], self.SPaT_end[-1]))

    def solve(self, lanes, num_lanes, max_speed, critical_volume_ratio):
        """
        This method implements Genetic Algorithm to determine :term:`SPaT`. The high-level work flow is as the following:
            #. From the available SPaT, only keep the ongoing one due to safety and practical reasons (*Here we do not change the timing of the first phase, however a variant is to reduce the timing to the minimum green time*).
            #. Serve as many as possible with the remaining phase.
            #. If any unserved vehicle is present, do GA.

        .. attention::
            - We define :term:`badness` (the opposite of fitness) as the measure that less of it is preferred for choosing a SPaT. Here we used travel time time ``ACCURACY_OF_BADNESS_MEASURE`` as the ``badness`` measure but any other measure can be simply used.
            - GA has access to only the given subset of phases provided by ``allowable_phases`` from the full set in ``data.py`` file.
            - If an alternative beats the best known SPaT, it takes the ``best_SPaT`` spot inside the ``evaluate_badness()`` call.
            - GA tries cycles with 1 up to the defined number of phases and for each it computes the cycle length using the time budget concept in traffic flow theory.
            - GA keeps the alternative in a sorted dictionary that the key is ``badness`` and the value keeps the corresponding SPaT decision. This helps when we want to replace worse individuals with new ones from crossover.
            - The phase sequence are randomly drawn from the set of phases **without** replacement.
            - The timings are random but respects the minimum and maximum green. They also sum to the cycle length.
            - Note since the dictionary hashes individuals based on their ``badness``, it may overwrite one individual with anther. Hence the population may fall less than what defined initially.
            - The crossover step is in-place, meaning it replaces the individuals with higher badness with crossovered ones. This way elite selection step is implemented at the same time crossover executes.
            - Eventually, the best SPaT may not serve all vehicles. In that case, ``_schedule_unserved_vehicles()`` method gets called to provide temporary schedule for the unserved vehicles.



        :param lanes:
        :param num_lanes:
        :param max_speed:
        :param critical_volume_ratio:
        """
        self._flush_upcoming_SPaTs()

        self.first_unsrvd_indx = np.zeros(num_lanes, dtype=np.int)
        any_unserved_vehicle = self.base_badness(lanes, num_lanes, max_speed)

        if any(any_unserved_vehicle):  # if the base SPaT serves, don't bother doing GA
            self.best_SPaT = {'phase_seq': tuple(), 'time_split': tuple(), 'badness_measure': self.LARGE_NUM}

            # correct max phase length in case goes above the range
            max_phase_length = min(len(self._allowable_phases), self.MAX_PHASE_LENGTH)

            for phase_length in range(1, max_phase_length):

                # this data structure keeps the individuals sorted based on badness level
                population = SortedDict({})

                cycle_length = self.get_optimal_cycle_length(critical_volume_ratio, phase_length)

                half_max_indx = phase_length // 2  # for crossover

                # POPULATE THE FIRST GENERATION
                for individual in range(self.POPULATION_SIZE):
                    # set the individuals
                    phase_seq = self.mutate_seq(phase_length)
                    time_split = self.mutate_timing(cycle_length, phase_length)
                    # evaluate the fitness function
                    badness = self.evaluate_badness(phase_seq, time_split, lanes, num_lanes)

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
                                badness = self.evaluate_badness(phase_seq, time_split, lanes, num_lanes)
                                population[badness] = {'phase_seq': phase_seq, 'time_split': time_split}

            for indx, phase in enumerate(self.best_SPaT['phase_seq']):
                self.append_extend_phase(int(phase), self.best_SPaT['time_split'][indx] - self._y - self._ar)

            if any(any_unserved_vehicle):
                self.best_scheduled_arrivals = self._schedule_unserved_vehicles(lanes, num_lanes,
                                                                                self.best_scheduled_arrivals,
                                                                                self.best_any_unserved_vehicle)
                self.first_unsrvd_indx = self.best_first_unsrvd_indx
                self._set_non_base_scheduled_arrival(lanes, self.best_scheduled_arrivals, num_lanes, max_speed)

    def evaluate_badness(self, phase_seq, time_split, lanes, num_lanes):
        """
        .. attentions::
            - Here we do not account for the vehicles served with base SPaT as they are already served.
            - We create a copy of ``first_unsrvd_indx`` since there is no guarantee this SPaT is the best by the end of GA.
            - The vehicle to be served by this method should have had ``veh.redo_trj_allowed`` set to ``True``.
            - An individual which has ``throughput`` of zero is not qualified for comparison to best known SPaT.

        :param phase_seq:
        :param time_split:
        :param lanes: holds the traffic intended to be served
        :param num_lanes:
        :return: The corresponding ``badness`` for given SPaT defined by ``phase_seq`` and ``time_split`` to be added to the population. It also sets, if qualified, this individual as the best known so far.
        """

        mean_travel_time, throughput = 0.0, 0  # if no vehicle is found return zero throughput
        temporary_scheduled_arrivals = {lane: np.zeros(len(lanes.vehlist[lane]), dtype=float) for lane in
                                        range(num_lanes)}

        # keeps departure time of last vehicle to be served by progressing SPaT
        served_vehicle_time = np.zeros(num_lanes, dtype=float)
        # keeps index of last vehicle to be served by progressing SPaT
        first_unsrvd_indx = np.copy(self.first_unsrvd_indx)

        any_unserved_vehicle = [first_unsrvd_indx[lane] <= lanes.last_vehicle_indx[lane] for lane in range(num_lanes)]

        phase_indx, phase_length = 0, len(phase_seq)
        start_green = self.SPaT_end[-1] + self.LAG
        while any(any_unserved_vehicle) and phase_indx < phase_length:  # serve all with the current phasing
            end_yellow = start_green - self.LAG + time_split[phase_indx] - self._ar
            phase = phase_seq[phase_indx]
            for lane in self._pli[phase]:
                if any_unserved_vehicle[lane]:
                    while True:
                        veh_indx = first_unsrvd_indx[lane]
                        veh = lanes.vehlist[lane][veh_indx]

                        t_earliest = veh.earliest_arrival
                        # depending on if we are keeping the prev trajectory or not, schedule or reschedule departure
                        t_scheduled = max(t_earliest, start_green, served_vehicle_time[
                            lane] + self._min_headway)

                        if t_scheduled <= end_yellow:
                            first_unsrvd_indx[lane] += 1

                            travel_time = t_scheduled - veh.init_time
                            mean_travel_time = ((mean_travel_time * throughput) + travel_time) / (throughput + 1)
                            throughput += 1

                            served_vehicle_time[lane] = t_scheduled
                            temporary_scheduled_arrivals[lane][veh_indx] = t_scheduled
                            if first_unsrvd_indx[lane] > lanes.last_vehicle_indx[lane]:
                                any_unserved_vehicle[lane] = False
                                break
                        else:
                            break
            phase_indx += 1
            start_green = end_yellow + self._ar

        badness = int(mean_travel_time * 100)
        if self.best_SPaT['badness_measure'] > badness and throughput > 0:
            self.best_SPaT['badness_measure'] = badness
            self.best_SPaT['phase_seq'] = tuple(phase_seq)
            self.best_SPaT['time_split'] = tuple(time_split)

            self.best_scheduled_arrivals = deepcopy(temporary_scheduled_arrivals)
            self.best_any_unserved_vehicle = any_unserved_vehicle
            self.best_first_unsrvd_indx = first_unsrvd_indx

        return int(mean_travel_time * 100)

    def get_optimal_cycle_length(self, critical_volume_ratio, phase_length):
        """
        Uses the time budget concept from traffic flow theory to compute the cycle length :math:`C=(n*ar)/(1-V_{cr})`.

        .. seealso::
            Refer to HCM 2010 for more details.

        :param critical_volume_ratio:
        :param phase_length:
        :return:
        """
        cycle_length = (phase_length * self._ar) / (1 - critical_volume_ratio)
        if cycle_length < 60:
            return 60
        elif cycle_length > 150:
            return 150
        else:
            return cycle_length

    def mutate_seq(self, phase_length):
        """
        Generates a randomized sequence from the provided subset of allowable phases.

        .. todo:: If two same phases follow each other within the sequence or at the boundary, re-sample carefully with replacement

        :param phase_length:
        :return: seq
        """
        seq = tuple(np.random.choice(self._allowable_phases, phase_length, replace=False))
        return seq

    def mutate_timing(self, cycle_length, phase_length):
        """
        Creates the random phase split. A valid timing should respect the min/max green requirement unless it conflicts with the cycle length requirement which in that case we should adjust the maximum green to avoid the slack in time.

        .. note:: A phase timing should be between :math:`g_{min}+y+ar` and :math:`g_{max}+y+ar`

        :param cycle_length:
        :param phase_length:
        :return: time_split
        """
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
        """
        Performs the crossover operation in GA.

        .. todo:: If two same phases follow each other, re-sample carefully with replacement or merge them

        :param left_parent:
        :param right_parent:
        :param phase_length:
        :param half_max_indx:
        :return: child with valid SPaT inherited from provided parents.
        """
        phase_seq = tuple([left_parent['phase_seq'][i] if i < half_max_indx else right_parent['phase_seq'][i]
                           for i in range(phase_length)])
        time_split = tuple([0.5 * (left_parent['time_split'][i] + right_parent['time_split'][i])
                            for i in range(phase_length)])

        return phase_seq, time_split


# -------------------------------------------------------
# ACTUATED SIGNAL CONTROL
# -------------------------------------------------------
class ActuatedControl(Signal):
    """
    # todo: main problem is how to schedule the departures

    :Author:
        Mahmoud Pourmehrab <pourmehrab@gmail.com>
    :Date:
        April-2018
    """

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
    (UNDERDEVELOPMENT)
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


    :Author:
        Mahmoud Pourmehrab <pourmehrab@gmail.com>
    :Date:
        April-2018
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
    (UNDERDEVELOPMENT)
    :return:

    :Author:
        Mahmoud Pourmehrab <pourmehrab@gmail.com>
    :Date:
        April-2018
    '''

    def __init__(self, inter_name, allowable_phases, num_lanes):

        super().__init__(inter_name, allowable_phases, num_lanes)

    def solve(self, lanes, num_lanes, allowable_phases):
        # the goal is to choose the sequence of SPaT which gives more throughput in less time
        self._flush_upcoming_SPaTs()

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
                self.append_extend_phase(remaining_phases.pop(),
                                         self.max_green)  # pop gives a random phase which is new

            else:  # progress SPaT
                served_vehicle_indx += best_throughput
                self.append_extend_phase(best_phase, max(self.min_green, best_phase_green_dur))
