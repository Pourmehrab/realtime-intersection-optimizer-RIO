####################################
# File name: signal.py             #
# Author: Mahmoud Pourmehrab       #
# Email: pourmehrab@gmail.com      #
# Last Modified: Jun/07/2018       #
####################################

import csv
import os
from copy import deepcopy

import numpy as np
from sortedcontainers import SortedDict

from data.config import get_GA_parameters

np.random.seed(2018)


class Signal:
    """
    The class serves the following goals:
        - Keeps the SPaT decision updated
        - Makes SPaT decisions through variety of control methods. It supports:
            - Pre-timed control
            - Genetic Algorithm

    Set the class variable ``lag_on_green`` to the time (in seconds) that from start of green is not valid to schedule any departure.

    .. note::
        - The signal status is saved under ``\log\<intersection name>\`` directory.

    Use Case:

        Instantiate like::

            >>> signal = MCF_SPaT(.)

        Perform :term:`SPaT` computation by::

            >>> signal.solve(.)


    :Author:
        Mahmoud Pourmehrab <pourmehrab@gmail.com>
    :Date:
        April-2018
    """

    def __init__(self, intersection, sc, start_time_stamp):
        """
        Elements:
            - Sequence keeps the sequence of phases to be executed from 0
            - ``green_dur`` keeps the amount of green allocated to each phase
            - ``yellow`` and ``all-red`` is a fixed amount at the end of all phases (look at class variables)

        .. note:: :term:`SPaT` starts executing from front (index 0) to the back of each list.

        :Author:
            Mahmoud Pourmehrab <pourmehrab@gmail.com>
        :Date:
            April-2018
       """
        self._inter_name, num_lanes = map(intersection._general_params.get, ["inter_name", "num_lanes"])

        self._set_lane_lane_incidence(intersection)
        self._set_phase_lane_incidence(intersection)

        if intersection._general_params.get("log_csv"):
            filepath_sig = os.path.join(
                "log/" + self._inter_name + "/" + start_time_stamp + "_" + str(sc) + "_sig_level.csv")
            self.__sig_csv_file = open(filepath_sig, "w", newline="")
            writer = csv.writer(self.__sig_csv_file, delimiter=",")
            writer.writerow(["sc", "phase", "start", "end"])
            self.__sig_csv_file.flush()
        else:
            self.__sig_csv_file = None

    def _set_lane_lane_incidence(self, intersection):
        """
        This converts a dictionary of the form:
        key is a lane and value is a *set* of lanes that are in conflict with key (note numbering starts from 1 not 0) to ``lane_lane_incidence`` which includes the conflict matrix :math:`|L|\\times |L|` where element :math:`ij` is 1 if :math:`i` and :math:`j` are conflicting movements

        :param num_lanes: number of incoming lanes

        :Author:
            Mahmoud Pourmehrab <pourmehrab@gmail.com>
        :Date:
            April-2018
        """
        self._lane_lane_incidence, num_lanes = map(intersection._general_params.get, ["lli", "num_lanes"])

        # self._lane_lane_incidence = {lane: set([]) for lane in range(num_lanes)}
        # for lane, conflicting_lane_set in lane_lane_incidence_one_based.items():  # the whole loop makes lanes zero-based
        #     self._lane_lane_incidence[lane - 1] = {conflicting_lane_1 - 1 for conflicting_lane_1 in
        #                                            conflicting_lane_set}

    def _set_phase_lane_incidence(self, intersection):
        """
        Sets the phase-phase incidence matrix of the intersection

        :param num_lanes:

        :Author:
            Mahmoud Pourmehrab <pourmehrab@gmail.com>
        :Date:
            April-2018
        """
        self._phase_lane_incidence, num_lanes = map(intersection._general_params.get, ["pli", "num_lanes"])

        # self._phase_lane_incidence = {phase: set([]) for phase in range(len(self._phase_lane_incidence))}
        # the whole following loop makes lanes and phases zero-based
        # for lane_1, nonconflicting_lane_set in phase_lane_incidence_one_based.items():
        #     self._phase_lane_incidence[lane_1 - 1] = {nonconflicting_lane_1 - 1 for nonconflicting_lane_1 in
        #                                               nonconflicting_lane_set}

        self._lane_phase_incidence = {lane: set([]) for lane in range(num_lanes)}
        for phase_indx, phase in self._phase_lane_incidence.items():
            for lane in phase:
                self._lane_phase_incidence[lane].add(phase_indx)

    def _append_extend_phase(self, phase, actual_green, intersection):
        """
        Appends a phase to the :term:`SPaT` (append/extend a phase and its green to the end of signal array)

        :param phase: phase to be added
        :param actual_green: green duration of that phase
        :Author:
            Mahmoud Pourmehrab <pourmehrab@gmail.com>
        :Date:
            April-2018
        """
        if self.SPaT_sequence[-1] == phase:  # extend the phase
            self.SPaT_end[-1] = self.SPaT_end[-1] - self._y - self._ar + actual_green
            intersection._general_params.get("print_commandline") and print(
                '>-> Phase {:d} extended (ends @ {:>5.1f} sec)'.format(self.SPaT_sequence[-1], self.SPaT_end[-1]))
        else:  # append a new phase
            self.SPaT_sequence += [phase]
            self.SPaT_green_dur += [actual_green]
            self.SPaT_start += [self.SPaT_end[-1]]
            self.SPaT_end += [self.SPaT_start[-1] + actual_green + self._y + self._ar]
            intersection._general_params.get("print_commandline") and print(
                '>>> Phase {:d} appended (ends @ {:>5.1f} sec)'.format(phase, self.SPaT_end[-1]))

    def update_SPaT(self, intersection, time_threshold, sc):
        """
        Performs two tasks to update SPaT based on the given clock:
            - Removes terminated phase (happens when the all-red is passed)
            - Checks for SPaT to not get empty after being updated

        .. attention::
            - If all phases are getting purged, either make longer SPaT decisions or reduce the simulation steps.

        :param time_threshold: Normally the current clock of simulation or real-time in :math:`s`
        :param sc: scenario number to be recorded in CSV

        :Author:
            Mahmoud Pourmehrab <pourmehrab@gmail.com>
        :Date:
            April-2018
        """
        assert self.SPaT_end[-1] >= time_threshold, "If all phases get purged, SPaT becomes empty"

        phase_indx, any_phase_to_purge = 0, False

        if self.__sig_csv_file is None:
            while time_threshold >= self.SPaT_end[phase_indx]:
                any_phase_to_purge = True
                phase_indx += 1
        else:
            writer = csv.writer(self.__sig_csv_file, delimiter=',')
            while time_threshold > self.SPaT_end[phase_indx]:
                any_phase_to_purge = True
                writer.writerows(
                    [[sc, self.SPaT_sequence[phase_indx], self.SPaT_start[phase_indx], self.SPaT_end[phase_indx]]])
                self.__sig_csv_file.flush()
                phase_indx += 1

        if any_phase_to_purge:
            intersection._general_params.get("print_commandline") and print(
                '<<< Phase(s) ' + ','.join(str(p) for p in self.SPaT_sequence[:phase_indx]) + ' expired')
            del self.SPaT_sequence[:phase_indx]
            del self.SPaT_green_dur[:phase_indx]
            del self.SPaT_start[:phase_indx]
            del self.SPaT_end[:phase_indx]

    def close_sig_csv(self):
        """Closes the signal CSV file"""
        self.__sig_csv_file.close()

    def _flush_upcoming_SPaTs(self, intersection):
        """
        Just leaves the first SPaT and removes the rest.

        .. note:: One more severe variant to this is to even reduce the the green time of the first phase.

        :Author:
            Mahmoud Pourmehrab <pourmehrab@gmail.com>
        :Date:
            April-2018
        """
        if len(self.SPaT_sequence) > 1:
            intersection._general_params.get("print_commandline") and print(
                '<<< Phase(s) ' + ','.join(str(p) for p in self.SPaT_sequence[1:]) + ' flushed')

            del self.SPaT_sequence[1:]
            del self.SPaT_green_dur[1:]
            del self.SPaT_start[1:]
            del self.SPaT_end[1:]

            extra_green = self.SPaT_green_dur[0] - self.min_green
            if extra_green > 0:
                self.SPaT_green_dur[0] -= extra_green
                self.SPaT_end[0] -= extra_green

    def _do_base_SPaT(self, lanes, intersection, trajectory_planner, tester):
        """
        This method aims to serve as many vehicles as possible given the available SPaT. Depending on the signal method, the set of current SPaT could be different. For example:

            - If called by :any:`Pretimed()` solver, the current SPaT may include multiple phases as a pretimed SPaT never gets flushed.
            - If called by :any:`GA_SPaT()` solver, since the SPaT gets flushed before calling. The goal is to serve as many vehicles with only the single current phase in SPaT.
            - It plans trajectories if necessary.

        The condition to be served is to meet the following criteria:
            - Respect the minimum headway to the lead vehicle (if present)
            - Respect the initiation of green plus a lag time specified by LAG as a class variable
            - Respect the earliest available time at the stop bar controlled by the speed limit  acc/dec rates
            - Vehicle is allowed to acquire a new trajectory (``veh.reschedule_departure`` holds True)

        The method does not compute or return the badness metric since the it does not aim to change current phase and timing.

        It may only gets called once per each Signal solve call prior to computation of the new SPaTs.

        The schedule keeps the earliest departures at the stop bars of each lane and gets updated when a signal decision goes permanent. It is made by a dictionary of arrays (key is lane, value is sorted earliest departures).

        ``lanes.first_unsrvd_indx`` and setting the schedule of any possible served vehicles make the main result of this method. The ``lanes.first_unsrvd_indx`` will be used after this to avoid reserving and double-counting those already served with base SPaT. This also returns ``any_unserved_vehicle`` array that has True if any lane has vehicles that could not be unserved with base SPaT.

        .. note::
            - Since base SPaT never gets changed (for safety and practical reasons), any vehicle served by it has to get ``reschedule_departure`` value set to ``False``.
            - It is feasible that if fusion algorithm updates the info on this vehicle and wants an update on trajectory, it rolls back the ``reschedule_departure`` to be ``True``. However, this should be decided outside this method.
            - The reason that this does not return schedule of departures is because they are already set inside this method. Late, the set method skips these.
            - If a vehicle gets a schedule and has more than one trajectory point, the last index should reset to the first index so when the trajectory is set there would be two points.
            - all-red from the end and ``LAG`` time from the beginning of a phase are note utilizes by any vehicle.
            - The ``veh.reschedule_departure`` is set to False for vehicles that get schedules here, however if decided a vehcile needs to be rescheduled, make it True wherever that decision is being made.

        :param lanes:
        :type lanes: Lanes
        :param intersection:
        :type intersection: Intersection
        :param trajectory_planner:
        :type trajectory_planner: src.intersection.TrajectoryPlanner
        :return: Increments the ``lanes.first_unsrvd_indx`` array that keeps index off the first unserved vehicle in each lane, is initialized to zero before calling this method and gets updated by the end of this call. It also returns ``served_vehicle_time`` that shows the schedule

        :Author:
            Mahmoud Pourmehrab <pourmehrab@gmail.com>
        :Date:
            April-2018
        """
        num_lanes, max_speed, min_headway, do_traj_computation, lag_on_green = map(intersection._general_params.get,
                                                                                   ['num_lanes', 'max_speed',
                                                                                    'min_headway',
                                                                                    'do_traj_computation',
                                                                                    'lag_on_green']
                                                                                   )
        any_unserved_vehicle = [lanes.first_unsrvd_indx[lane] <= lanes.last_vehicle_indx[lane] for lane in
                                range(num_lanes)]

        for phase_indx, phase in enumerate(self.SPaT_sequence):
            green_starts, yellow_ends = self.SPaT_start[phase_indx] + lag_on_green, self.SPaT_end[phase_indx] - self._ar
            for lane in self._phase_lane_incidence.get(phase):
                if any_unserved_vehicle[lane]:
                    for veh_indx, veh in enumerate(lanes.vehlist.get(lane)[lanes.first_unsrvd_indx[lane]:]):
                        if veh.reschedule_departure:
                            dep_after_lead_veh = lanes.vehlist.get(lane)[
                                                     veh_indx - 1].best_temporary_departure + min_headway if veh_indx > 0 else -10.0
                            t_scheduled = max(veh.earliest_departure, green_starts, dep_after_lead_veh)
                            if t_scheduled <= yellow_ends:
                                lanes.increment_first_unsrvd_indx(lane)
                                # t, d, s = t_scheduled, 0.0, min(max_speed, veh.desired_speed)
                                veh.set_best_temporary_departure(t_scheduled)
                                # veh.set_scheduled_departure(t, d, s, lane, veh_indx, intersection)

                                if do_traj_computation and veh.freshly_scheduled:  # freshly_scheduled is set within set_scheduled_departure method
                                    trajectory_planner.plan_trajectory(lanes, veh, lane, veh_indx, intersection, tester,
                                                                       '*')
                                    veh.reschedule_departure, veh.freshly_scheduled = False, False
                                else:
                                    break  # no more room to serve vehicles in this phase
                        else:  # next vehicle may want trajectory
                            veh.set_best_temporary_departure(veh.scheduled_departure)
                            lanes.increment_first_unsrvd_indx(lane)

                    if lanes.first_unsrvd_indx[lane] > lanes.last_vehicle_indx[lane]:
                        any_unserved_vehicle[lane] = False
        tester is not None and tester.test_departure_of_trj(lanes, intersection, [0] * num_lanes,
                                                            lanes.first_unsrvd_indx)
        return any_unserved_vehicle  # it also sets the lanes.first_unsrvd_indx and veh.set_best_temporary_departure but does not return them

    def _do_non_base_SPaT(self, lanes, first_unsrvd_indx, intersection, tester):
        """
        Most of times the base SPaT prior to running a ``solve()`` method does not serve all vehicles. However, vehicles require trajectory to be provided. One way to address this is to assign them the best temporal trajectory which only has some of general qualities necessary for continuation of program. In this method we do the followings to compute the ``departure times`` of such trajectories:

            - Without use of phases, schedule vehicles one after the other at minimum headway restricted by the saturation headway. This gives an overestimate of teh departure time since one vehicle gets served by intersection at a time, while having allowing to depart in phases let multiple simultaneous departures.
            - This may be called after a signal ``solve()`` method decided to complete those that did not get served.
            - Also this assumes min headway after green starts instead of ``LAG`` time which is a simplification.
            - If a vehicle gets a schedule and has more than one trajectory point, the last index should reset to the first index so when the trajectory is set there would be two points.


        .. warning::
            - Since the departure times are definitely temporal, DO NOT set ``reschedule_departure`` to ``False``.
            - The ``lanes.first_unsrvd_indx`` cannot be used since it does not keep GA newly served vehicles. However, it would work for pretimed since the method is static.

        :param lanes:
        :type lanes: Lanes
        :param first_unsrvd_indx: keeps the index of first unserved vehicle in lanes.
        :param intersection:
        :type intersection: Intersection
        :return: The ``temporary_departure`` field in the vehicle object is set.

        :Author:
            Mahmoud Pourmehrab <pourmehrab@gmail.com>
        :Date:
            April-2018
        """
        num_lanes, max_speed, phase_cover_set, lag_on_green, min_headway, do_traj_computation = map(
            intersection._general_params.get,
            ["num_lanes", "max_speed", "phase_cover_set", "lag_on_green", "min_headway", "do_traj_computation"])
        time_phase_ends = self.SPaT_end[-1] - self._ar
        for phase in phase_cover_set:
            time_phase_ends += self._ar + lag_on_green
            for lane in self._phase_lane_incidence.get(phase):
                start_indx = first_unsrvd_indx[lane]
                for veh_indx, veh in enumerate(lanes.vehlist.get(lane)[start_indx:], start_indx):
                    t_earliest = veh.earliest_departure
                    dep_after_lead_veh = max(lanes.vehlist.get(lane)[veh_indx - 1].scheduled_departure,
                                             lanes.vehlist.get(lane)[veh_indx - 1].best_temporary_departure) \
                        if veh_indx > 0 else -10.0
                    t_scheduled = max(t_earliest, time_phase_ends, dep_after_lead_veh + min_headway)
                    time_phase_ends = t_scheduled
                    veh.set_best_temporary_departure(t_scheduled)
        if tester is not None:
            tester.test_SPaT_alternative(lanes, [0] * num_lanes, lanes.last_vehicle_indx,
                                         lanes.last_vehicle_indx, min_headway)

    def _set_non_base_scheduled_departures(self, lanes, any_unserved_vehicle, trajectory_planner, intersection, tester):
        """
        Sets the scheduled departure in the trajectory of the vehicle and plans trajectory of vehicle.

        .. note::
            - Departure schedule of those which were served by base SPaT is set in ``base_badness()`` and not here.
            - A cover phase set for all lanes is needed here.

        :param lanes:
        :type lanes: Lanes
        :param trajectory_planner:
        :type trajectory_planner: TrajectoryPlanner
        :param tester: the test object
        :type tester: test.unit_tests.SimTest

        :Author:
            Mahmoud Pourmehrab <pourmehrab@gmail.com>
        :Date:
            April-2018
        """
        max_speed, num_lanes, min_headway, do_traj_computation = map(
            intersection._general_params.get, ["max_speed", "num_lanes", "min_headway", "do_traj_computation"])
        for lane in range(num_lanes):
            if any_unserved_vehicle[lane]:
                for veh_indx, veh in enumerate(lanes.vehlist.get(lane)):
                    t, d, s = veh.best_temporary_departure, 0.0, min(max_speed, veh.desired_speed)
                    if veh_indx > 0:
                        assert t + 0.01 >= min_headway + lanes.vehlist.get(lane)[
                            veh_indx - 1].scheduled_departure, "violation in min headway"
                    veh.reschedule_departure and veh.set_scheduled_departure(t, d, s, lane, veh_indx, intersection)
                    if do_traj_computation and veh.freshly_scheduled:
                        trajectory_planner.plan_trajectory(lanes, veh, lane, veh_indx, intersection, tester, '#')
                        veh.freshly_scheduled = False
                    else:
                        continue


# -------------------------------------------------------
# Genetic Algorithms
# -------------------------------------------------------
class GA_SPaT(Signal):
    """
    Under this class, the :term:`SPaT` is decided optimally by a :term:`GA`.


    .. warning::
        - ``allowable_phases`` **must** cover all lanes or some would not get green at all.
        - ``allowable_phases`` **must** be zero-based unlike what is provided in ``config.py``

    :Author:
        Mahmoud Pourmehrab <pourmehrab@gmail.com>
    :Date:
        April-2018
    """

    def __init__(self, first_detection_time, intersection, sc, start_time_stamp):
        """

        :param inter_name:
        :param first_detection_time:
        :param intersection:
        :type intersection: Intersection
        :param sc:
        :param start_time_stamp:
        :Author:
            Mahmoud Pourmehrab <pourmehrab@gmail.com>
        :Date:
            April-2018
        """

        super().__init__(intersection, sc, start_time_stamp)
        inter_name, print_commandline = map(intersection._general_params.get, ["inter_name", "print_commandline"])
        self.__GA_params = get_GA_parameters(inter_name)

        self._max_phase_length = min(len(self.__GA_params.get("allowable_phases")),
                                     self.__GA_params.get("max_phase_length"))

        self._y, self._ar, self.min_green, self.max_green = get_signal_params(inter_name)
        self._ts_min, self._ts_max = self.min_green + self._y + self._ar, self.max_green + self._y + self._ar
        self._ts_diff = self.max_green - self.min_green

        # add a dummy phase to initiate
        self.SPaT_sequence = [self.__GA_params.get("allowable_phases")[0]]
        self.SPaT_green_dur = [first_detection_time - self._y - self._ar]
        self.SPaT_start = [0.0]
        self.SPaT_end = [first_detection_time]

        if print_commandline:
            print(">>> Phase {:d} appended (ends @ {:2.1f} sec)".format(self.SPaT_sequence[-1], self.SPaT_end[-1]))

    def solve(self, lanes, intersection, critical_volume_ratio, trajectory_planner, tester):
        """
        This method implements Genetic Algorithm to determine :term:`SPaT`. The high-level work flow is as the following:
            #. From the available :term:`SPaT`, only keep the ongoing one due to safety and practical reasons (*Here we do not change the timing of the first phase, however a variant is to reduce the timing to the minimum green time*).
            #. Serve as many as possible with the remaining phase.
            #. If any unserved vehicle is present, do :term:`GA`.

        .. attention::
            - We define :term:`badness` (the opposite of fitness) as the measure that less of it is preferred for choosing a SPaT.
            - GA has access to only the given subset of phases provided by ``allowable_phases`` from the full set in ``config.py`` file.
            - If an alternative beats the best known SPaT, it takes the ``__best_SPaT`` spot inside the :any:`_evaluate_badness` call.
            - GA tries cycles with 1 up to the defined number of phases and for each it computes the cycle length using the time budget concept in traffic flow theory.
            - GA keeps the alternative in a sorted dictionary that the key is ``badness`` and the value keeps the corresponding SPaT decision. This helps when we want to replace worse individuals with new ones from crossover.
            - The phase sequence are randomly drawn from the set of phases **without** replacement.
            - The timings are random but respects the minimum and maximum green. They also sum to the cycle length.
            - Note since the dictionary hashes individuals based on their :term:`badness`, it may overwrite one individual with anther. Hence the population may fall less than what defined initially.
            - The crossover step is in-place, meaning it replaces the individuals with higher badness with crossovered ones. This way elite selection step is implemented at the same time crossover executes.
            - Eventually, the best SPaT may not serve all vehicles. In that case, :any:`_schedule_unserved_vehicles` method gets called to provide temporary schedule for the unserved vehicles.

        :param lanes:
        :type lanes: Lanes
        :param intersection:
        :type intersection: Intersection
        :param critical_volume_ratio:
        :param trajectory_planner:
        :type trajectory_planner: TrajectoryPlanner
        :param tester: test object

        :Author:
            Mahmoud Pourmehrab <pourmehrab@gmail.com>
        :Date:
            April-2018
        """
        num_lanes, large_positive_num = map(intersection._general_params.get, ['num_lanes', 'large_positive_num'])
        self._flush_upcoming_SPaTs(intersection)
        any_unserved_vehicle = self._do_base_SPaT(lanes, intersection, trajectory_planner, tester)
        if any(any_unserved_vehicle):
            # if the base SPaT serves, don't bother doing GA # correct max phase length in case goes above the range
            cycle_length = self._get_optimal_cycle_length(critical_volume_ratio, 1)
            self.__best_GA_alt = {
                'SPaT': {'phase_seq': self._mutate_seq(1), 'time_split': self._mutate_timing(cycle_length, 1),
                         'badness_measure': large_positive_num},
                'scheduled_departures': {lane: np.zeros(len(lanes.vehlist.get(lane)), dtype=float) for lane in
                                         range(num_lanes)},
                'any_unserved_vehicle': any_unserved_vehicle,
                'first_unserved_indx': np.copy(lanes.first_unsrvd_indx),
            }

            population = SortedDict({})
            for individual in range(self.__GA_params.get('population_size')):
                phase_seq = self._mutate_seq(1)
                time_split = self._mutate_timing(cycle_length, 1)
                badness = self._evaluate_badness(phase_seq, time_split, lanes, intersection, tester)
                population[badness] = {'phase_seq': phase_seq, 'time_split': time_split}

            for phase_length in range(2, self._max_phase_length + 1):
                population = SortedDict({})  # keeps the individuals
                cycle_length = self._get_optimal_cycle_length(critical_volume_ratio, phase_length)
                half_max_indx = phase_length // 2  # need this for crossover

                for individual in range(self.__GA_params.get('population_size')):
                    phase_seq = self._mutate_seq(phase_length)
                    time_split = self._mutate_timing(cycle_length, phase_length)
                    badness = self._evaluate_badness(phase_seq, time_split, lanes, intersection, tester)
                    population[badness] = {'phase_seq': phase_seq, 'time_split': time_split}

                # perform GA operations in-place
                for i in range(self.__GA_params.get('max_iteration_per_phase')):
                    for j in range(self.__GA_params.get('crossover_size')):
                        # ELITE SELECTION
                        # in the next round, the top ones will automatically be removed when crossover
                        badness_list = list(population.keys())
                        if len(population) > 3:  # CROSSOVER
                            del population[badness_list[-1]]  # delete the worst member and then add one
                            parents_indx = np.random.choice(badness_list[:-1], 2, replace=False)
                            phase_seq, time_split = self._cross_over(population[parents_indx[0]],
                                                                     population[parents_indx[1]],
                                                                     phase_length, half_max_indx)
                            # evaluate badness function
                            badness = self._evaluate_badness(phase_seq, time_split, lanes, intersection, tester)
                            population[badness] = {'phase_seq': phase_seq, 'time_split': time_split}

            # assert self.__best_GA_alt.get('SPaT').get(
            #     'badness_measure') < large_positive_num, "GA failed to find any serving SPaT"

            for indx, phase in enumerate(self.__best_GA_alt.get('SPaT').get('phase_seq')):
                self._append_extend_phase(int(phase), self.__best_GA_alt.get('SPaT').get('time_split')[
                    indx] - self._y - self._ar, intersection)
            self._do_non_base_SPaT(lanes, self.__best_GA_alt.get('first_unserved_indx'), intersection, tester)
            self._set_non_base_scheduled_departures(lanes, any_unserved_vehicle, trajectory_planner, intersection,
                                                    tester)

    def _evaluate_badness(self, phase_seq, time_split, lanes, intersection, tester):
        """
         This method computes the badness (opposite if fitness) of an alternative using the equation :math:`\lambda \\times t-c`, where:
            - :math:`c` is the count of served vehicles in :math:`veh`
            - :math:`\lambda` is weight factor in :math:`veh/s`
            - :math:`t` is the average travel time in :math:`s`, under the given :term:`SPaT`.

         .. attention::
            - A rough approximate for :math:`\lambda` is the inverse of the detection range.
            - Here we do not account for the vehicles served with base :term:`SPaT` as they are already served.
            - We create a copy of ``first_unsrvd_indx`` since there is no guarantee this :term:`SPaT` is the best by the end of :term:`GA`.
            - The vehicle to be served by this method should have had ``veh.reschedule_departure`` set to ``True``.
            - An individual which has ``throughput`` of zero is not qualified for comparison to best known :term:`SPaT`.
            - Please note base on the provided definition :term:`badness` can acquire negative values.
            - Recursively computes the average travel time

        :param phase_seq:
        :param time_split:
        :param lanes: holds the traffic intended to be served
        :type lanes: Lanes
        :param intersection:
        :type intersection: Intersection
        :param tester: the test object
        :type tester: test.unit_tests.SimTest

        :return: The corresponding :term:`badness` for given SPaT defined by ``phase_seq`` and ``time_split`` to be added to the population. It also sets, if qualified, this individual as the best known so far.

        :Author:
            Mahmoud Pourmehrab <pourmehrab@gmail.com>
        :Date:
            April-2018
        """
        num_lanes, large_positive_num, lag_on_green, min_headway = map(intersection._general_params.get,
                                                                       ['num_lanes', 'large_positive_num',
                                                                        'lag_on_green', 'min_headway'])
        mean_travel_time, throughput = 0.0, 0  # if no vehicle is found return zero throughput
        # keeps index of last vehicle to be served by progressing SPaT
        first_unsrvd_indx = np.copy(lanes.first_unsrvd_indx)
        any_unserved_vehicle = [first_unsrvd_indx[lane] <= lanes.last_vehicle_indx[lane] for lane in range(num_lanes)]
        green_starts = self.SPaT_end[-1] + lag_on_green
        for phase_indx, phase in enumerate(phase_seq):
            yellow_ends = green_starts - lag_on_green + time_split[phase_indx] - self._ar
            for lane in self._phase_lane_incidence.get(phase):
                if any_unserved_vehicle[lane]:
                    start_indx = first_unsrvd_indx[lane]
                    for veh_indx, veh in enumerate(
                            lanes.vehlist.get(lane)[start_indx:lanes.last_vehicle_indx[lane] + 1], start_indx):
                        t_earliest = veh.earliest_departure
                        if veh_indx == 0:
                            t_scheduled = max(t_earliest, green_starts)
                        else:
                            lead_veh = lanes.vehlist.get(lane)[veh_indx - 1]
                            lead_veh_dep_time = lead_veh.scheduled_departure if veh_indx < lanes.first_unsrvd_indx[
                                lane] else lead_veh.temporary_departure
                            t_scheduled = max(t_earliest, green_starts, lead_veh_dep_time + min_headway)
                            if lead_veh.temporary_departure > 0:
                                assert t_scheduled > lead_veh.temporary_departure, "min headway violated"

                        if t_scheduled <= yellow_ends:
                            first_unsrvd_indx[lane] += 1
                            travel_time = t_scheduled - veh.init_time
                            mean_travel_time = ((mean_travel_time * throughput) + travel_time) / (throughput + 1)
                            throughput += 1
                            veh.set_temporary_departure(t_scheduled)  # not the best though
                            if first_unsrvd_indx[lane] > lanes.last_vehicle_indx[lane]:
                                any_unserved_vehicle[lane] = False
                                break
                        else:
                            break
            green_starts = yellow_ends + self._ar

        if tester is not None:
            tester.test_SPaT_alternative(lanes, lanes.first_unsrvd_indx, first_unsrvd_indx, lanes.last_vehicle_indx,
                                         min_headway)

        badness = int(
            (self.__GA_params.get('lambda') * mean_travel_time - throughput) * self.__GA_params.get(
                'badness_accuracy')) if throughput > 0 else large_positive_num
        if self.__best_GA_alt.get('SPaT').get('badness_measure') > badness:
            self.__best_GA_alt = \
                {
                    'SPaT': {'phase_seq': tuple(phase_seq), 'time_split': tuple(time_split),
                             'badness_measure': badness},
                    'any_unserved_vehicle': deepcopy(any_unserved_vehicle),
                    'first_unserved_indx': deepcopy(first_unsrvd_indx),
                }
            lanes.set_temp_as_best_departure(lanes.first_unsrvd_indx, first_unsrvd_indx)
        return badness

    def _get_optimal_cycle_length(self, critical_volume_ratio, phase_length):
        """
        Uses the time budget concept from traffic flow theory to compute the cycle length :math:`C=\\frac{n \\times ar}{1-V_{cr}}`.

        .. seealso::
            Refer to HCM 2010 for more details.

        :param critical_volume_ratio:
        :param phase_length:
        :return:
        :Author:
            Mahmoud Pourmehrab <pourmehrab@gmail.com>
        :Date:
            April-2018
        """
        cycle_length = (phase_length * self._ar) / (1 - critical_volume_ratio)
        if cycle_length < 60:
            return 60
        elif cycle_length > 240:
            return 240
        else:
            return cycle_length

    def _mutate_seq(self, phase_length):
        """
        Generates a randomized sequence from the provided subset of allowable phases.

        .. todo:: If two same phases follow each other within the sequence or at the boundary, re-sample carefully with replacement

        :param phase_length:
        :return: seq
        :Author:
            Mahmoud Pourmehrab <pourmehrab@gmail.com>
        :Date:
            April-2018
        """
        seq = tuple(np.random.choice(self.__GA_params.get("allowable_phases"), phase_length, replace=False))
        return seq

    def _mutate_timing(self, cycle_length, phase_length):
        """
        Creates the random phase split. A valid timing should respect the min/max green requirement unless it conflicts with the cycle length requirement which in that case we should adjust the maximum green to avoid the slack in time.

        .. note:: A phase timing should be between :math:`g_{min}+y+ar` and :math:`g_{max}+y+ar`

        :param cycle_length:
        :param phase_length:
        :return: time_split
        :Author:
            Mahmoud Pourmehrab <pourmehrab@gmail.com>
        :Date:
            April-2018
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

    def _cross_over(self, left_parent, right_parent, phase_length, half_max_indx):
        """
        Performs the crossover operation in GA.

        .. todo:: If two same phases follow each other, re-sample carefully with replacement or merge them

        :param left_parent:
        :param right_parent:
        :param phase_length:
        :param half_max_indx:
        :return: child with valid SPaT inherited from provided parents.

        :Author:
            Mahmoud Pourmehrab <pourmehrab@gmail.com>
        :Date:
            April-2018
        """
        phase_seq = tuple([left_parent['phase_seq'][i] if i < half_max_indx else right_parent['phase_seq'][i]
                           for i in range(phase_length)])
        time_split = tuple([0.5 * (left_parent['time_split'][i] + right_parent['time_split'][i])
                            for i in range(phase_length)])

        return phase_seq, time_split


# -------------------------------------------------------
# MIN COST FLOW SPaT
# -------------------------------------------------------
class MCF_SPaT(Signal):
    """
    Under this class, the :term:`SPaT` is decided optimally by a :term:`MCF` model.


    .. warning::
        - 

    :Author:
        Mahmoud Pourmehrab <pourmehrab@gmail.com>
    :Date:
        April-2018
    """

    def __init__(self, first_detection_time, intersection, sc, start_time_stamp):
        import cplex
        """

        :param inter_name:
        :param first_detection_time:
        :param intersection:
        :type intersection: Intersection
        :param sc:
        :param start_time_stamp:

        :Author:
            Mahmoud Pourmehrab <pourmehrab@gmail.com>
        :Date:
            July-2018
        """

        super().__init__(intersection, sc, start_time_stamp)
        inter_name, num_lanes, print_commandline, self._y, self._ar, self.min_green, self.max_green, self._allowable_phases = map(
            intersection._general_params.get,
            ["inter_name", "num_lanes", "print_commandline", "yellow", "allred", "min_green", "max_green",
             "allowable_phases"])

        self._ts_min, self._ts_max = self.min_green + self._y + self._ar, self.max_green + self._y + self._ar
        self._ts_diff = self.max_green - self.min_green

        # add a dummy phase to initiate
        self.SPaT_sequence = [self._allowable_phases[0]]
        self.SPaT_green_dur = [first_detection_time - self._y - self._ar]
        self.SPaT_start = [0.0]
        self.SPaT_end = [first_detection_time]

        if print_commandline:
            print(">>> Phase {:d} appended (ends @ {:2.1f} sec)".format(self.SPaT_sequence[-1], self.SPaT_end[-1]))

        self._mcf_model = cplex.Cplex()
        self._mcf_model.objective.set_sense(self._mcf_model.objective.sense.minimize)
        self._mcf_model.set_problem_name('MCF SPaT optimization')
        self._mcf_model.set_problem_type(cplex.Cplex().problem_type.LP)
        self._mcf_model.parameters.lpmethod.set(self._mcf_model.parameters.lpmethod.values.network)
        self._mcf_model.parameters.read.datacheck.set(2)
        self._mcf_model.set_log_stream(None)
        self._mcf_model.set_error_stream(None)
        self._mcf_model.set_warning_stream(None)
        self._mcf_model.set_results_stream(None)

        max_phase_length = max([len(phase) for phase_indx, phase in self._phase_lane_incidence.items()])

        for lane, phase_set in self._lane_phase_incidence.items():
            var_name_l2p = ["l" + str(lane) + "p" + str(phase_indx) for phase_indx in phase_set]
            n = len(var_name_l2p)
            self._mcf_model.variables.add(obj=[0.0] * n, names=var_name_l2p, lb=[0.0] * n, ub=[cplex.infinity] * n)

            self._mcf_model.linear_constraints.add(
                lin_expr=[[var_name_l2p, [1.0] * n]], senses=["E"], rhs=[0.0], names=["lane_" + str(lane)], )

        for phase_indx, phase in self._phase_lane_incidence.items():
            var_name_p2p = ["p" + str(phase_indx) + "p" + str(phase_indx)]
            var_name_p2s = ["p" + str(phase_indx) + "s"]

            c = max_phase_length - len(phase)
            self._mcf_model.variables.add(obj=[c, 0.0], names=var_name_p2p + var_name_p2s,
                                          lb=[0.0] * 2, ub=[0.0, cplex.infinity])

            var_name_l2p = ["l" + str(lane) + "p" + str(phase_indx) for lane in phase]
            self._mcf_model.linear_constraints.add(
                lin_expr=[[var_name_l2p + var_name_p2p, [1.0] * len(var_name_l2p) + [-1.0]],
                          [var_name_p2p + var_name_p2s, [1.0, -1.0]]], senses=["E"] * 2,
                rhs=[0.0, 0.0], names=["phase_" + str(phase_indx), "sink_" + str(phase_indx)], )
        self._mcf_model.linear_constraints.add(
            lin_expr=[[["p" + str(phase_indx) + "s" for phase_indx in self._phase_lane_incidence.keys()],
                       [1.0] * len(self._phase_lane_incidence)]], senses=["E"], rhs=[0.0], names=["sink"], )

    def solve(self, lanes, intersection, critical_volume_ratio, trajectory_planner, tester):
        """

        :param lanes:
        :param intersection:
        :param critical_volume_ratio:
        :param trajectory_planner:
        :param tester:
        :return:

        :Author:
            Mahmoud Pourmehrab <pourmehrab@gmail.com>
        :Date:
            July-2018
        """
        num_lanes, min_CAV_headway, min_CNV_headway, lag_on_green, min_dist_to_stop_bar = map(
            intersection._general_params.get,
            ["num_lanes", "min_CAV_headway", "min_CNV_headway", "lag_on_green", "min_dist_to_stop_bar"])
        num_phases = len(self._phase_lane_incidence)
        self._flush_upcoming_SPaTs(intersection)

        demand = [float(len(lanes.vehlist.get(lane))) for lane in range(num_lanes)]
        total_demand = sum(demand)
        if total_demand > 0:
            self._mcf_model.linear_constraints.set_rhs(
                [("sink", total_demand), ] + list(zip(["lane_" + str(lane) for lane in range(num_lanes)], demand)))
            self._mcf_model.variables.set_upper_bounds([("p" + str(phase_indx) + "p" + str(phase_indx),
                                                         sum([demand[lane] for lane in phase])) for
                                                        phase_indx, phase in self._phase_lane_incidence.items()])

            try:
                self._mcf_model.solve()
            except cplex.exceptions.CplexSolverError:
                raise Exception("Exception raised during solve")

            if self._mcf_model.solution.get_status() in {1, }:
                pass
            else:
                raise Exception("Check MCF CPLEX model")

            phase_veh_incidence = np.array(self._mcf_model.solution.get_values(
                ["p" + str(phase_indx) + "p" + str(phase_indx) for phase_indx in range(num_phases)]),
                dtype=np.int)

            phase_early_first = SortedDict({})
            for phase_indx, phase in self._phase_lane_incidence.items():
                if phase_veh_incidence[phase_indx] > 0:
                    min_dep_time = min([lanes.vehlist.get(lane)[0].earliest_departure for lane in phase if
                                        bool(lanes.vehlist.get(lane))])
                    key = min_dep_time + 0.01 if min_dep_time in phase_early_first else min_dep_time
                    phase_early_first[key] = phase_indx

            time_ordered, phase_ordered = map(list, [phase_early_first.keys(), phase_early_first.values()])

            if time_ordered[0] > self.SPaT_end[-1]:
                phase = np.random.randint(0, len(self._phase_lane_incidence))
                while phase == phase_ordered[0]:
                    phase = np.random.randint(0, len(self._phase_lane_incidence))
                self._append_extend_phase(phase,
                                          time_ordered[0] - self.SPaT_end[-1] - self._y - self._ar - lag_on_green,
                                          intersection)

            veh_indx_vec = [min(0, len(veh_list) - 1) for lane, veh_list in lanes.vehlist.items()]
            vehicle_counter, num_vehicles = 0, int(total_demand)
            # green_dur = 0.0 if phase_ordered[0] != self.SPaT_sequence[-1] else -self.SPaT_green_dur[-1]
            phase_start_time = self.SPaT_end[-1] if phase_ordered[0] != self.SPaT_sequence[-1] else self.SPaT_start[-1]
            phase_circular_indx, mod = -1, len(phase_ordered)
            while vehicle_counter < num_vehicles:
                phase_green_end_time = phase_start_time + self.min_green
                phase_circular_indx = (phase_circular_indx + 1) % mod
                phase = phase_ordered[phase_circular_indx]
                served_veh_phase_counter = 0
                flag = True
                for lane in self._phase_lane_incidence.get(phase):
                    start_indx = veh_indx_vec[lane]
                    for veh_indx, veh in enumerate(lanes.vehlist.get(lane)[start_indx:], start_indx):
                        t_scheduled = max(veh.earliest_departure, phase_start_time + lag_on_green)
                        if veh_indx > 0:
                            lead_veh = lanes.vehlist.get(lane)[veh_indx - 1]
                            t_scheduled = max(t_scheduled, lead_veh.scheduled_departure + (
                                min_CAV_headway if veh.veh_type == 1 else min_CNV_headway))

                        if served_veh_phase_counter < phase_veh_incidence[phase]: # t_scheduled <= phase_start_time + self.max_green and
                            veh_indx_vec[lane] += 1
                            vehicle_counter += 1
                            served_veh_phase_counter += 1
                            phase_green_end_time = max(t_scheduled, phase_green_end_time)
                            _, det_dist, _ = veh.get_arrival_schedule()
                            flag = False
                            if det_dist >= min_dist_to_stop_bar:
                                veh.set_scheduled_departure(t_scheduled, 0, veh.desired_speed, lane, veh_indx,
                                                            intersection)
                                trajectory_planner.plan_trajectory(lanes, veh, lane, veh_indx, intersection, tester,
                                                                   '#')
                                veh.freshly_scheduled = False

                if flag:
                    print("done")
                self._append_extend_phase(int(phase), phase_green_end_time - phase_start_time, intersection)
                phase_start_time = self.SPaT_end[-1]
        else:
            pass  # check if a dummy phase is needed
