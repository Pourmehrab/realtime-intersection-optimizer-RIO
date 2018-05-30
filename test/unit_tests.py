####################################
# File name: unit_tests.py         #
# Author: Mahmoud Pourmehrab       #
# Email: pourmehrab@gmail.com      #
# Last Modified: Apr/08/2018       #
####################################

import sys
import unittest
import operator
import numpy as np
import numpy.testing as npt


class SimTest(unittest.TestCase):
    """
    A series of test methods.

    :Author:
        Mahmoud Pourmehrab <pourmehrab@gmail.com>
    :Date:
        April-2018
    """

    def py_version_test(self, req=(3, 6, 0)):
        """

        :param req: python version to be checked against

        :Author:
            Mahmoud Pourmehrab <pourmehrab@gmail.com>
        :Date:
            April-2018
        """
        assert all(sys.version_info[i] >= req[i] for i in range(len(req))), "Please update python interpreter."

    def arguments_check(self, req=({"13th16th", "TERL", "reserv", },
                                   {"GA", "pretimed"},
                                   {"simulation", "realtime"},)):
        """

        :param req: set of available intersections, signal opt methods, and run modes to choose

        :Author:
            Mahmoud Pourmehrab <pourmehrab@gmail.com>
        :Date:
            April-2018
        """
        assert all(sys.argv[i + 1] in req[i] for i in range(len(req))), "Check the input arguments and try again."

    def test_departure_of_trj(self, lanes, intersection, start_indx, end_indx):
        """
        Checks for no early scheduled arrivals

        :param lanes:
        :param intersection:
        :param start_indx:
        :param end_indx:

        :Author:
            Mahmoud Pourmehrab <pourmehrab@gmail.com>
        :Date:
            April-2018
        """
        num_lanes, max_speed, min_headway = map(intersection._general_params.get,
                                                ['num_lanes', 'max_speed', 'min_headway'])
        for lane in range(num_lanes):
            if bool(lanes.vehlist[lane]):
                for veh_indx in range(start_indx[lane], end_indx[lane]):
                    veh = lanes.vehlist.get(lane)[veh_indx]
                    det_time, det_dist, det_speed = veh.get_arrival_schedule()
                    dep_time, dep_dist, dep_speed = veh.get_departure_schedule()
                    departure_time_relative = dep_time - det_time  # traj will be from t in [0, t_rel]
                    self.assertGreater(veh.scheduled_departure, 0.0, msg="No departure is scheduled")
                    if det_dist >= 50:
                        assert departure_time_relative >= 0.0, "Vehicle is scheduled to depart at time less/equal to the first trajectory point time stamp"
                    if departure_time_relative != 0.0:
                        avg_speed = (det_dist - dep_dist) / departure_time_relative  # trajectory average speed
                        self.assertLessEqual(avg_speed, 1.5 * max_speed, msg="avg speed above 1.5 X speed limit")
                        self.assertGreaterEqual(avg_speed, 0.0, msg="negative avg speed")
                    if veh_indx > 0:
                        lead_veh = lanes.vehlist.get(lane)[veh_indx - 1]
                        lead_dep_time = lead_veh.trajectory[0, lead_veh.last_trj_point_indx]
                        self.assertGreaterEqual(dep_time, lead_dep_time + min_headway - 0.1,
                                                msg="the follower cannot depart earlier than the lead.")

    def test_SPaT_alternative(self, scheduled_departures, start_unsrvd_indx, end_vehicle_indx, last_vehicle_indx,
                              min_headway):
        """
        Checks for min headway to be respected in the schedules

        :param scheduled_departures:
        :param start_unsrvd_indx:
        :param end_vehicle_indx:
        :param last_vehicle_indx:
        :param min_headway:

        :Author:
            Mahmoud Pourmehrab <pourmehrab@gmail.com>
        :Date:
            April-2018
        """
        for lane in range(len(scheduled_departures)):
            for veh_indx in range(start_unsrvd_indx[lane], min(end_vehicle_indx[lane], last_vehicle_indx[lane] + 1)):
                if veh_indx > start_unsrvd_indx[lane]:
                    headway = scheduled_departures.get(lane)[veh_indx] - scheduled_departures.get(lane)[veh_indx - 1]
                    self.assertGreaterEqual(headway, min_headway - 0.01, msg="The min headway constraint is violated.")

    def test_trj_points(self, veh):
        """
        Checks all the planned trajectory points.

        :param veh:
        :type veh: Vehicle

        :Author:
            Mahmoud Pourmehrab <pourmehrab@gmail.com>
        :Date:
            April-2018
        """

        trj_point_indx = veh.first_trj_point_indx
        first_trj_point_indx, last_trj_point_indx = veh.first_trj_point_indx, veh.last_trj_point_indx
        trajectory = veh.trajectory
        time_diff = np.diff(trajectory[0, first_trj_point_indx:last_trj_point_indx + 1], n=1)
        npt.assert_array_less([0.0] * len(time_diff), time_diff, err_msg="time is not strictly monotonic")
        dist_diff = np.diff(trajectory[1, first_trj_point_indx:last_trj_point_indx + 1], n=1)
        npt.assert_array_less(dist_diff, [1.0] * len(dist_diff), err_msg="trj distance is not monotonic")
        if last_trj_point_indx - trj_point_indx > 0:  # if there are at least two points, check them
            while trj_point_indx <= last_trj_point_indx:
                time, dist, speed = trajectory[:, trj_point_indx]
                assert all(map(operator.not_, np.isnan([time, dist, speed]))), 'nan found in trajectory'
                assert all(map(operator.not_, np.isinf([time, dist, speed]))), 'infinity found in the schedule'

                self.assertGreater(speed, -3, msg="Negative speed")
                self.assertGreater(dist, -3, msg="Traj point after the stop bar")
                trj_point_indx += 1
