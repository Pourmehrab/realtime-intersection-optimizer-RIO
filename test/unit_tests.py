####################################
# File name: unit_tests.py         #
# Author: Mahmoud Pourmehrab       #
# Email: pourmehrab@gmail.com      #
# Last Modified: May/30/2018       #
####################################

import operator
import sys
import unittest

import numpy as np
import numpy.testing as npt


class SimTest(unittest.TestCase):
    """
    A series of test methods for the optimization and part of the planner that was implemented for the simulation purpose.

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

    def test_departure_of_trj(self, lanes, intersection, start_indx, end_indx):
        """
        Checks scheduled arrivals for:
            - being processed by planner
            - relative departure to be after arrival
            - range of average speed to be feasible
            - min headway with the lead vehicle (if exists)

        :param lanes:
        :param intersection:
        :param start_indx:
        :param end_indx:

        :Author:
            Mahmoud Pourmehrab <pourmehrab@gmail.com>
        :Date:
            April-2018
        """
        num_lanes, max_speed, min_CAV_headway, min_CNV_headway = map(intersection._general_params.get,
                                                                     ['num_lanes', 'max_speed', "min_CAV_headway",
                                                                      "min_CNV_headway"])
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
                        lead_dep_time, _, _ = lead_veh.get_departure_schedule()
                        min_headway = min_CAV_headway if veh.veh_type == 1 else min_CNV_headway
                        self.assertGreaterEqual(dep_time, lead_dep_time + min_headway - 0.01,
                                                msg="the follower cannot depart earlier than the lead.")

    def test_SPaT_alternative(self, lanes, start_unsrvd_indx, end_vehicle_indx, last_vehicle_indx, min_headway):
        """
        Checks for min headway to be respected in the schedule.

        :param start_unsrvd_indx:
        :param end_vehicle_indx:
        :param last_vehicle_indx:
        :param min_headway:

        :Author:
            Mahmoud Pourmehrab <pourmehrab@gmail.com>
        :Date:
            April-2018
        """
        for lane in range(len(start_unsrvd_indx)):
            start_indx = start_unsrvd_indx[lane]
            end_indx = min(end_vehicle_indx[lane], last_vehicle_indx[lane] + 1)
            for veh_indx, veh in enumerate(lanes.vehlist.get(lane)[start_indx:end_indx], start_indx):
                self.assertGreater(veh.best_temporary_departure, 0.0, msg="no departure time is scheduled")
                if veh_indx > start_indx:
                    headway = veh.best_temporary_departure - lanes.vehlist.get(lane)[
                        veh_indx - 1].best_temporary_departure
                    self.assertGreaterEqual(headway, min_headway - 0.01, msg="The min headway constraint is violated.")

    def test_planned_departure(self, veh):
        """
        Tests if actually the departure from trajectory matches the planned departure.

        :param veh:
        :type veh: Vehicle

        :Author:
            Mahmoud Pourmehrab <pourmehrab@gmail.com>
        :Date:
            April-2018
        """
        self.assertAlmostEqual(veh.trajectory[0, veh.last_trj_point_indx], veh.scheduled_departure, places=1,
                               msg="The planned trj does not match the scheduled time")

    def test_trj_points(self, veh):
        """
        Checks all trajectory assigned to a vehicle for:
            - time difference.
            - distance difference.
            - speed & distance non-negativity.
            - any null & nan values.

        :param veh:
        :type veh: Vehicle

        :Author:
            Mahmoud Pourmehrab <pourmehrab@gmail.com>
        :Date:
            April-2018
        """

        first_trj_point_indx, last_trj_point_indx, trajectory = veh.first_trj_point_indx, veh.last_trj_point_indx, veh.trajectory
        time_diff = np.diff(trajectory[0, first_trj_point_indx:last_trj_point_indx + 1], n=1)
        npt.assert_array_less([0.0] * len(time_diff), time_diff, err_msg="time is not strictly monotonic")
        dist_diff = np.diff(trajectory[1, first_trj_point_indx:last_trj_point_indx + 1], n=1)
        npt.assert_array_less(dist_diff, [2.0] * len(dist_diff), err_msg="trj distance is not monotonic")
        trj_point_indx = veh.first_trj_point_indx
        while trj_point_indx <= last_trj_point_indx:
            time, dist, speed = trajectory[:, trj_point_indx]
            assert all(map(operator.not_, np.isnan([time, dist, speed]))), 'nan found in trajectory'
            assert all(map(operator.not_, np.isinf([time, dist, speed]))), 'infinity found in the schedule'

            self.assertGreater(speed, -3, msg="Negative speed")
            self.assertGreater(dist, -3, msg="Traj point after the stop bar")
            trj_point_indx += 1

    def check_order_in_lanes(self, lanes):
        """
        Tests, after updating the trajectories, if the order in each lane is right.

        :param lanes:
        :type lanes: Lanes

        :Author:
            Mahmoud Pourmehrab <pourmehrab@gmail.com>
        :Date:
            May-2018
        """
        for lane in range(len(lanes.vehlist)):
            if len(lanes.vehlist[lane]) > 0:  # at least two vehicles
                for veh_indx in range(1, len(lanes.vehlist[lane])):
                    veh = lanes.vehlist.get(lane)[veh_indx]
                    lead_veh = lanes.vehlist.get(lane)[veh_indx - 1]
                    _, foll_det_dist, _ = veh.get_arrival_schedule()
                    _, lead_det_dist, _ = lead_veh.get_arrival_schedule()
                    self.assertLess(lead_det_dist, foll_det_dist, msg="follower is closer to stop bar than the lead")

    @staticmethod
    def check_for_collision(veh, lead_veh):
        """
        Tests every pair of trajectory points to respect zero gap.

        :param veh: follower
        :type veh: Vehicle
        :param lead_veh: leader
        :type lead_veh: Vehicle
        :return:

        :Author:
            Mahmoud Pourmehrab <pourmehrab@gmail.com>
        :Date:
            May-2018
        """
        lead_d_vec, foll_d_vec = map(
            lambda veh_obj: veh_obj.trajectory[1, veh_obj.first_trj_point_indx:veh_obj.last_trj_point_indx + 1],
            [lead_veh, veh])
        npt.assert_array_less(lead_d_vec, foll_d_vec[0:len(lead_d_vec)], err_msg="collision will happen")
