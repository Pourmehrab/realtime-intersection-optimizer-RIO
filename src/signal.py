import csv
import os
import numpy as np
from sortedcontainers import SortedDict


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

    def __init__(self, args, intersection, sc, start_time_stamp):
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
        self._inter_name, num_lanes = map(intersection._inter_config_params.get, ["inter_name", "num_lanes"])

        self.mode = args.mode
        self._set_lane_lane_incidence(intersection)
        self._set_phase_lane_incidence(intersection)

        if args.do_logging:
            filepath_sig = os.path.join(args.log_dir, "sig_level.csv")
            self.__sig_csv_file = open(filepath_sig, "w", newline="")
            writer = csv.writer(self.__sig_csv_file, delimiter=",")
            header = ["sc", "phase", "start", "end"]
            if self.mode == "realtime":
                header += ["timestamp"]
            writer.writerow(header)
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
        self._lane_lane_incidence, num_lanes = map(intersection._inter_config_params.get, ["lli", "num_lanes"])

    def _set_phase_lane_incidence(self, intersection):
        """
        Sets the phase-phase incidence matrix of the intersection

        :param num_lanes:

        :Author:
            Mahmoud Pourmehrab <pourmehrab@gmail.com>
        :Date:
            April-2018
        """
        self._phase_lane_incidence, num_lanes = map(intersection._inter_config_params.get, ["pli", "num_lanes"])

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
            intersection._inter_config_params.get("print_commandline") and print(
                '>-> Phase {:d} extended (ends @ {:>5.1f} sec)'.format(self.SPaT_sequence[-1], self.SPaT_end[-1]))
        else:  # append a new phase
            self.SPaT_sequence += [phase]
            self.SPaT_green_dur += [actual_green]
            self.SPaT_start += [self.SPaT_end[-1]]
            self.SPaT_end += [self.SPaT_start[-1] + actual_green + self._y + self._ar]
            intersection._inter_config_params.get("print_commandline") and print(
                '>>> Phase {:d} appended (ends @ {:>5.1f} sec)'.format(phase, self.SPaT_end[-1]))

    def update_SPaT(self, intersection, absolute_time, sc, time_since_last_arrival, timestamp):
        """
        Performs two tasks to update SPaT based on the given opt_clock:
            - Removes terminated phase (happens when the all-red is passed)
            - Checks for SPaT to not get empty after being updated

        .. attention::
            - If all phases are getting purged, either make longer SPaT decisions or reduce the simulation steps.

        :param absolute_time: Normally the current opt_clock of simulation or real-time in :math:`s`
        :param sc: scenario number to be recorded in CSV
        :param time_since_last_arrival: in realtime, the absolute Datetime timestamp or in sim mode,
        the number of seconds since start from the last vehicle message received
        :param timestamp: the current Datetime timestamp, for recording in the CSV

        :Author:
            Mahmoud Pourmehrab <pourmehrab@gmail.com>
        :Date:
            April-2018
        """
        SPaT_expiration = self.SPaT_end[-1] - absolute_time
        if SPaT_expiration <= 0:  # continue the last phase util 1 sec after absolute time
            self._append_extend_phase(self.SPaT_sequence[-1], 1 - SPaT_expiration + self._y + self._ar, intersection)

        assert self.SPaT_end[-1] >= absolute_time, \
            "If all phases get purged, SPaT becomes empty. SPaT end: {}, elapsed time: {}" \
                .format(self.SPaT_end[-1], absolute_time)

        phase_indx, any_phase_to_purge = 0, False

        if self.__sig_csv_file is None:
            while absolute_time > self.SPaT_end[phase_indx]:
                any_phase_to_purge = True
                phase_indx += 1
        else:
            writer = csv.writer(self.__sig_csv_file, delimiter=',')
            while absolute_time > self.SPaT_end[phase_indx]:
                any_phase_to_purge = True
                row = [sc, self.SPaT_sequence[phase_indx], self.SPaT_start[phase_indx], self.SPaT_end[phase_indx]]
                if self.mode == "realtime":
                    row += [str(timestamp)]
                writer.writerows([row])
                self.__sig_csv_file.flush()
                phase_indx += 1

        if any_phase_to_purge:
            intersection._inter_config_params.get("print_commandline") and print(
                '<<< Phase(s) ' + ','.join(str(p) for p in self.SPaT_sequence[:phase_indx]) + ' expired')
            del self.SPaT_sequence[:phase_indx]
            del self.SPaT_green_dur[:phase_indx]
            del self.SPaT_start[:phase_indx]
            del self.SPaT_end[:phase_indx]

    def close_sig_csv(self):
        """Closes the signal CSV file"""
        self.__sig_csv_file.close()

    def protect_SPaT(self, lanes, intersection):
        """

        - Finds the latest departure time assigned to a vehicle closer than `min_dist_to_stop_bar`,
        - If no vehicle is close, then just reserve the minimum green for the front phase,
        - Otherwise, reserve the latest assigned green for vehicles closer than `min_dist_to_stop_bar`.

        .. note:: Those vehicles that are closer than the min distance should not be counted as demand when solving MCF.

        :returns array of count of close vehicles in lanes to protect (since they are closer than `min_dist_to_stop_bar`)

        :Author:
            Mahmoud Pourmehrab <pourmehrab@gmail.com>
        :Date:
            April-2018
        """

        num_lanes, min_dist_to_stop_bar = map(intersection._inter_config_params.get,
                                              ["num_lanes", "min_dist_to_stop_bar", ])

        latest_departure_to_protect = 0
        last_locked_veh_indx = np.zeros(num_lanes, dtype=np.int) - 1

        for lane in range(num_lanes):
            for veh_indx, veh in enumerate(lanes.vehlist.get(lane)):
                _, curr_dist, _ = veh.get_arr_sched()
                if curr_dist < min_dist_to_stop_bar:
                    last_locked_veh_indx[lane] += 1
                    dep_time = veh.scheduled_departure
                    if dep_time > latest_departure_to_protect:
                        latest_departure_to_protect = dep_time
                else:
                    break

        locked_SPaT_indx, max_SPaT_indx = 0, len(self.SPaT_sequence) - 1

        while locked_SPaT_indx < max_SPaT_indx and latest_departure_to_protect > self.SPaT_end[locked_SPaT_indx]:
            locked_SPaT_indx += 1

        if locked_SPaT_indx < max_SPaT_indx:
            intersection._inter_config_params.get("print_commandline") and print(
                '<<< Phase(s) ' + ','.join(str(p) for p in self.SPaT_sequence[locked_SPaT_indx + 1:]) + ' flushed')

            del self.SPaT_sequence[locked_SPaT_indx + 1:]
            del self.SPaT_green_dur[locked_SPaT_indx + 1:]
            del self.SPaT_start[locked_SPaT_indx + 1:]
            del self.SPaT_end[locked_SPaT_indx + 1:]

        return last_locked_veh_indx
    # -------------------------------------------------------


# MIN COST FLOW SPaT
# -------------------------------------------------------
class MCF_SPaT(Signal):
    """
    Under this class, the :term:`SPaT` is decided optimally by a :term:`MCF` model.

    The first phase in `_allowable_phases` gets initialized with a minimum green in the constructor.



    :Author:
        Mahmoud Pourmehrab <pourmehrab@gmail.com>
    :Date:
        April-2018
    """

    def __init__(self, args, first_detection_time, intersection, sc, start_time_stamp):
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

        super().__init__(args, intersection, sc, start_time_stamp)
        num_lanes, print_commandline, self._y, self._ar, self.min_green, self.max_green, self._allowable_phases = map(
            intersection._inter_config_params.get,
            ["num_lanes", "print_commandline", "yellow", "allred", "min_green", "max_green", "allowable_phases"])

        self._ts_min, self._ts_max = self.min_green + self._y + self._ar, self.max_green + self._y + self._ar
        self._ts_diff = self.max_green - self.min_green

        # add a dummy phase to initiate
        self.SPaT_sequence = [self._allowable_phases[0]]
        self.SPaT_green_dur = [self.min_green]
        self.SPaT_start = [first_detection_time]
        self.SPaT_end = [first_detection_time + self.SPaT_green_dur[0] + self._y + self._ar]

        print_commandline and print(
            ">>> Phase {:d} initializes SPaT (ends @ {:2.1f} sec)".format(self.SPaT_sequence[-1], self.SPaT_end[-1]))

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

    def solve(self, lanes, intersection, trajectory_planner, absolute_time):
        """
        The minimum cost flow problem that yields the optimal departure times and SPaT is coded as a CPLEX model.

        :param lanes:
        :param intersection:
        :param trajectory_planner:
        :param absolute_time:
        :return:

        :Author:
            Mahmoud Pourmehrab <pourmehrab@gmail.com>
        :Date:
            July-2018
        """

        num_lanes, min_CAV_headway, min_CNV_headway, lag_on_green, min_dist_to_stop_bar = map(
            intersection._inter_config_params.get,
            ["num_lanes", "min_CAV_headway", "min_CNV_headway", "lag_on_green", "min_dist_to_stop_bar"])
        num_phases = len(self._phase_lane_incidence)

        last_locked_veh_indx = self.protect_SPaT(lanes, intersection)

        first_veh_indx_to_serve = np.array(
            [min(len(lanes.vehlist.get(lane)) - 1, last_locked_veh_indx[lane] + 1) for lane in
             range(num_lanes)], dtype=int)

        protected_veh_count = sum(last_locked_veh_indx)
        intersection._inter_config_params.get("print_commandline") and protected_veh_count > 0 and print(
            '   Trajectory and SPaT of ' + str(protected_veh_count) + ' vehicles are protected')

        demand = first_veh_indx_to_serve - last_locked_veh_indx
        demand = demand.astype(float, copy=False)
        total_demand = demand.sum()
        if total_demand > 0:
            self._mcf_model.linear_constraints.set_rhs(
                [("sink", total_demand), ] + list(zip(["lane_" + str(lane) for lane in range(num_lanes)], demand)))
            self._mcf_model.variables.set_upper_bounds([("p" + str(phase_indx) + "p" + str(phase_indx),
                                                         sum([demand[lane] for lane in phase])) for
                                                        phase_indx, phase in self._phase_lane_incidence.items()])
            self._mcf_model.write("mcf.lp")

            try:
                self._mcf_model.solve()
            except cplex.exceptions.CplexSolverError:
                raise Exception("Exception raised during solve")

            assert self._mcf_model.solution.get_status() in {
                1, }, "MCF CPLEX model did not end up with optimal solution"

            phase_veh_incidence = np.array(self._mcf_model.solution.get_values(
                ["p" + str(phase_indx) + "p" + str(phase_indx) for phase_indx in range(num_phases)]), dtype=np.int)

            phase_early_first = SortedDict({})
            for phase_indx, phase in self._phase_lane_incidence.items():
                if phase_veh_incidence[phase_indx] > 0:
                    min_dep_time = min(
                        [lanes.vehlist.get(lane)[first_veh_indx_to_serve[lane]].earliest_departure for lane in phase if
                         first_veh_indx_to_serve[lane] != -1])
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

            veh_indx_vec = np.copy(first_veh_indx_to_serve)
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

                    if last_locked_veh_indx[lane] == first_veh_indx_to_serve[lane]:
                        continue

                    start_indx = veh_indx_vec[lane]
                    for veh_indx, veh in enumerate(lanes.vehlist.get(lane)[start_indx:], start_indx):
                        t_scheduled = max(veh.earliest_departure, phase_start_time + lag_on_green)
                        veh.got_trajectory = False
                        if veh_indx > 0:
                            lead_veh = lanes.vehlist.get(lane)[veh_indx - 1]
                            t_scheduled = max(t_scheduled, lead_veh.scheduled_departure + (
                                min_CAV_headway if veh.veh_type == 1 else min_CNV_headway))

                        if served_veh_phase_counter < phase_veh_incidence[
                            phase]:  # t_scheduled <= phase_start_time + self.max_green and
                            veh_indx_vec[lane] += 1
                            vehicle_counter += 1
                            served_veh_phase_counter += 1
                            phase_green_end_time = max(t_scheduled, phase_green_end_time)
                            _, det_dist, _ = veh.get_arr_sched()
                            flag = False
                            if det_dist >= min_dist_to_stop_bar:
                                veh.set_sched_dep(t_scheduled, 0, veh.desired_speed, lane, veh_indx)
                                trajectory_planner.plan_trajectory(lanes, veh, lane, veh_indx, intersection, '#')
                                veh.got_trajectory = True

                if flag:
                    print("done")
                self._append_extend_phase(int(phase), phase_green_end_time - phase_start_time, intersection)
                phase_start_time = self.SPaT_end[-1]
