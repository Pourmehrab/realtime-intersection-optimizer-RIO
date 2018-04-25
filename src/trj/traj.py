####################################
# File name: traj.py               #
# Author: Mahmoud Pourmehrab       #
# Email: mpourmehrab@ufl.edu       #
# Last Modified: Apr/23/2018       #
####################################

import numpy as np


# -------------------------------------------------------
# TRAJECTORY SUPER CLASS
# -------------------------------------------------------

class Trajectory:
    """
    Abstract class for computing the trajectory points. Four subclasses inherited from Trajectory():
        * LeadConventional
        * FollowerConnected
        * LeadConnected
        * FollowerConventional

    Note if want to omit the trajectory planning, there are two options:
        * If a particular vehicle is intended to be skipped, simply invoke veh.set_redo_trj_false() whenever needed
        * If the whole simulation is intended to be run without trajectory planer, set do_traj_computation in main.py
        to False.

    Any solve method under each class shall invoke set_trajectory() method at the end or does the assignment in-place.

    :param LAG: the lag time from start of green when a vehicle can depart
    :param RES: time difference between two consecutive trajectory points in second (be careful not to exceed max size
     of trajectory
    :param EPS: small number that lower than that is approximated by zero
    """

    LAG = 1
    RES = 1
    EPS = 0.01

    def __init__(self, max_speed, min_headway):
        """
        :param max_speed: Trajectories are designed to respect the this speed limit (in m/s).
        :param min_headway: This is the minimum headway that vehicles in a lane can be served (in sec/veh)
        """
        self._max_speed = max_speed
        self._min_headway = min_headway

    def discretize_time_interval(self, start_time, end_time):
        """
        Discretize the given time interval to a numpy array of time stamps
        The resolution is equal to the :param RES: (sec)
        """
        if end_time <= start_time:
            return np.array([])
        elif end_time - start_time % self.RES > self.EPS:
            trj_time_stamps = np.append(np.arange(start_time, end_time, Trajectory.RES, dtype=float), end_time)
        else:
            trj_time_stamps = np.arange(start_time, end_time, Trajectory.RES, dtype=float)

        return trj_time_stamps

    @staticmethod
    def set_trajectory(veh, t, d, s):
        """
        Sets trajectory of the vehicle and updates the first and last trajectory point index.

        :param veh: the vehicle object that is owns the trajectory
        :param t: time stamps (seconds from the reference time)
        :param d: distances at each time stamp (in meters from the stop bar)
        :param s: speed at each time stamp (in m/s)
        """
        n = len(t)
        veh.trajectory[:, 0:n] = [t, d, s]
        veh.set_first_trj_point_indx(0)
        veh.set_last_trj_point_indx(n - 1)


# -------------------------------------------------------
# LEAD CONVENTIONAL TRAJECTORY ESTIMATOR
# -------------------------------------------------------
class LeadConventional(Trajectory):
    """
    Computes the trajectory for a lead conventional vehicle assuming the vehicle tends to maintain its arrival speed.
    """

    def __init__(self, max_speed, min_headway):
        super().__init__(max_speed, min_headway)

    def solve(self, veh):
        """
        Constructs the trajectory of a lead conventional vehicle assuming the driver maintains its speed

        :param veh: the lead conventional vehicle
        """
        trajectory = veh.trajectory
        first_trj_point_indx = veh.first_trj_point_indx
        det_time, det_dist, det_speed = trajectory[:, first_trj_point_indx]
        scheduled_arrival = veh.scheduled_arrival  # the time this vehicle departs the stop bar

        arrival_time = det_time + det_dist / det_speed

        t = self.discretize_time_interval(det_time, arrival_time)
        s = np.array([det_speed for i in range(len(t))])
        d = np.array([det_dist - det_speed *
                      (t[i] - det_time) for i in range(len(t))])

        if arrival_time > scheduled_arrival + self.EPS:
            raise Exception("the computed earliest arrival of a lead conventional is later than the scheduled time.")
        else:

            t_augment = self.discretize_time_interval(t[-1] + self.RES, scheduled_arrival)
            d_augment = [0 for t in t_augment]
            s_augment = [0 for t in t_augment]

            t = np.append(t, t_augment)
            d = np.append(d, d_augment)
            s = np.append(s, s_augment)

            self.set_trajectory(veh, t, d, s)


# -------------------------------------------------------
# FOLLOWER CONVENTIONAL TRAJECTORY ESTIMATOR
# -------------------------------------------------------
class FollowerConventional(Trajectory):
    """
    Computes the trajectory for a follower conventional vehicle assuming a car following model.

    """

    def __init__(self, max_speed, min_headway):
        super().__init__(max_speed, min_headway)

    def solve(self, veh, lead_veh):
        """
        Gipps car following model is assumed here.
        It is written in-place (does not call set_trajectory)

        Refer to:
            Gipps, Peter G. "A behavioural car-following model for computer simulation."
            Transportation Research Part B: Methodological 15.2 (1981): 105-111.
        Note the only trajectory point index that changes is follower's last one
        """
        follower_trajectory = veh.trajectory
        follower_desired_speed = veh.desired_speed
        follower_max_acc, follower_max_dec = veh.max_accel_rate, veh.max_decel_rate
        follower_trj_indx = veh.first_trj_point_indx
        follower_detection_time = follower_trajectory[0, follower_trj_indx]

        lead_trajectory = lead_veh.trajectory
        lead_max_dec, lead_length = lead_veh.max_decel_rate, lead_veh.length
        lead_trj_indx = lead_veh.first_trj_point_indx  # this starts with followers first and goes to leads last point
        lead_last_trj_point_indx = lead_veh.last_trj_point_indx

        while lead_trj_indx <= lead_last_trj_point_indx:
            # this avoids considering the part of lead trajectory before follower showed up
            if follower_detection_time > lead_trajectory[0, lead_trj_indx]:
                lead_trj_indx += 1
            else:
                break
        if lead_trj_indx > lead_last_trj_point_indx:
            # the lead vehicle left before detection of this vehicle
            # check vehicle update process since lead vehicle should have been removed
            raise Exception("The lead vehicle sent to FollowerConventional() should have been removed.")
        else:
            while lead_trj_indx < lead_last_trj_point_indx:  # less than is used since it computes the next trajectory point
                next_trj_indx = follower_trj_indx + 1
                lead_speed = lead_trajectory[2, lead_trj_indx]
                follower_speed = follower_trajectory[2, follower_trj_indx]
                gap = follower_trajectory[1, follower_trj_indx] - lead_trajectory[1, lead_trj_indx]
                dt = lead_trajectory[0, lead_trj_indx + 1] - lead_trajectory[0, lead_trj_indx]
                if lead_speed <= self.EPS:  # Gipps doesn"t work well for near zero speed
                    follower_trajectory[0, next_trj_indx] = lead_trajectory[0, lead_trj_indx + 1]
                    v_get_behind = (gap - lead_length) / dt
                    v = min(v_get_behind, follower_desired_speed) if v_get_behind >= 0 else 0.0
                    follower_trajectory[1, next_trj_indx] = follower_trajectory[1, follower_trj_indx] - v * dt
                    follower_trajectory[2, next_trj_indx] = 0.0

                else:
                    follower_trajectory[0, next_trj_indx] = lead_trajectory[0, lead_trj_indx + 1]

                    s1 = 1 / 40 + follower_speed / follower_desired_speed
                    s2 = (follower_max_dec * (lead_max_dec * (2 * (lead_length - gap) + dt * (
                            follower_max_dec * dt + follower_speed)) + lead_speed ** 2)) / lead_max_dec

                    if s1 >= 0 and s2 >= 0:  # determines followers next speed
                        v1 = follower_speed + 2.5 * follower_max_acc * dt * (
                                1 - follower_speed / follower_desired_speed) * np.sqrt(s1)
                        v2 = follower_max_dec * dt + np.sqrt(s2)  # might get negative
                        follower_trajectory[2, next_trj_indx] = min(v1, v2) if v2 >= 0 else v1
                    elif s1 >= 0:
                        v1 = follower_speed + 2.5 * follower_max_acc * dt * (
                                1 - follower_speed / follower_desired_speed) * np.sqrt(s1)
                        follower_trajectory[2, next_trj_indx] = v1
                    elif s2 >= 0:
                        v2 = follower_max_dec * dt + np.sqrt(s2)  # might get negative
                        follower_trajectory[2, next_trj_indx] = max(0.0, v2)
                    else:  # not supposed to happen
                        follower_trajectory[2, follower_trj_indx + 1] = follower_speed

                    a = (follower_trajectory[2, next_trj_indx] - follower_speed) / dt
                    follower_trajectory[1, next_trj_indx] = max(follower_trajectory[1, follower_trj_indx] - (
                            a * (follower_trajectory[0, next_trj_indx] ** 2 - follower_trajectory[
                        0, follower_trj_indx] ** 2) / 2 + (follower_trajectory[2, follower_trj_indx] -
                                                           a * follower_trajectory[0, follower_trj_indx]) * dt), 0.0)
                    # todo remove the max and see why distance may get zero
                    if np.any(np.isnan(follower_trajectory[1, :])):
                        raise Exception("nan in time stamp of follower conventional")

                follower_trj_indx += 1
                lead_trj_indx += 1

        # This part adds the end part that is out of Gipps Car Following domain
        d_follower_end = follower_trajectory[1, follower_trj_indx]
        t_follower_end = follower_trajectory[0, follower_trj_indx]

        t_augment = self.discretize_time_interval(self.RES, d_follower_end / follower_desired_speed)
        d_augment = [d_follower_end - t * follower_desired_speed for t in t_augment]
        s_augment = [follower_desired_speed for i in range(len(t_augment))]

        last_index = follower_trj_indx + len(t_augment) + 1
        follower_trajectory[0, follower_trj_indx + 1:last_index] = t_augment + t_follower_end
        follower_trajectory[1, follower_trj_indx + 1:last_index] = d_augment
        follower_trajectory[2, follower_trj_indx + 1:last_index] = s_augment
        veh.set_last_trj_point_indx(last_index - 1)


import cplex


# -------------------------------------------------------
# LEAD CONNECTED AND AUTOMATED TRAJECTORY OPTIMIZER
# -------------------------------------------------------

class LeadConnected(Trajectory):
    """
    Trajectory function:
        f(t)   = sum_{n=0}^{k-1} b_n t^n

        Negative of speed profile:
        f'(t)  = sum_{n=1}^{k-1} n b_n t^{n-1}

        Negative of acceleration profile:
        f''(t) = sum_{n=2}^{k-1} n (n-1) b_n t^{n-2}

    Refer to "IBM(R) ILOG CPLEX Python API Reference Manual" for CPLEX usage using Python
    For solver status codes: https://www.ibm.com/support/knowledgecenter/SSSA5P_12.8.0/ilog.odms.cplex.help/refcallablelibrary/macros/Solution_status_codes.html
    To write the model use 'write("model.lp")' on it.

    """
    NUM_DIGS = 3  # the accuracy to keep decimals

    def __init__(self, max_speed, min_headway, k, m):
        """

        :param k: the size of array that keeps the polynomial (k-1 is the degree of polynomial)
        :param m: number of points (exclusive of boundaries) to control speed/acceleration magnitude
        """
        super().__init__(max_speed, min_headway)

        self.k, self.m = k, m

        self._lp_model = cplex.Cplex()
        self._lp_model.set_log_stream(None)
        self._lp_model.set_error_stream(None)
        self._lp_model.set_warning_stream(None)
        self._lp_model.set_results_stream(None)

        # set optimization direction
        self._lp_model.objective.set_sense(self._lp_model.objective.sense.minimize)

        # add variables (coefficients of the polynomial that'll represent the trajectory)
        var_name = ["beta_" + str(n) for n in range(self.k)]
        self._lp_model.variables.add(obj=[1.0 for n in range(self.k)],
                                     names=var_name,
                                     lb=[-cplex.infinity for n in range(self.k)])
        # add two constraints that fixes the detected distance/speed on the polynomial
        self._lp_model.linear_constraints.add(lin_expr=[[["beta_0"], [1.0]], [["beta_1"], [1.0]]],
                                              senses=["E", "E"],
                                              rhs=[0.0, 0.0],
                                              names=["match_det_dist", "match_speed"])

        # add generic form constraints with zero coefficients (will be modified in set_model(.) )
        constraint = [var_name, [0.0 for n in range(self.k)]]
        self._lp_model.linear_constraints.add(
            lin_expr=[constraint, constraint],
            senses=["E", "E"],
            rhs=[1.0, 1.0],
            names=["match_dep_dist", "match_dep_speed"])
        self._lp_model.linear_constraints.add(
            lin_expr=[constraint for j in range(self.m)] +
                     [constraint for j in range(self.m)] +
                     [constraint for j in range(self.m)] +
                     [constraint for j in range(self.m)],
            senses=["G" for j in range(self.m)] +
                   ["L" for j in range(self.m)] +
                   ["G" for j in range(self.m)] +
                   ["L" for j in range(self.m)],
            rhs=[-max_speed for j in range(self.m)] +
                [0.0 for j in range(self.m)] +
                [1.0 for j in range(self.m)] +
                [1.0 for j in range(self.m)],
            names=["ub_speed_" + str(j) for j in range(self.m)] +
                  ["lb_speed_" + str(j) for j in range(self.m)] +
                  ["ub_acc_" + str(j) for j in range(self.m)] +
                  ["lb_acc_" + str(j) for j in range(self.m)])

    def set_model(self, veh, arrival_time, arrival_dist, dep_speed, is_lead=False):
        """
        Overrides the generic coefficients to build the specific model

        :param veh:             vehicle object that its trajectory is meant to be computed
        :param arrival_time:    time vehicle is scheduled to reach the stop bar
        :param arrival_dist:    distance vehicle is scheduled to reach the stop bar
        :param dep_speed:       speed vehicle is scheduled to reach the stop bar
        """
        delay = veh.scheduled_arrival - veh.earliest_arrival

        if delay < self.EPS and is_lead:  # do not have to solve LP since the earliest trj works
            return None
        else:  # solve LP
            trajectory = veh.trajectory
            amin, amax = veh.max_decel_rate, veh.max_accel_rate

            first_trj_point_indx = veh.first_trj_point_indx
            det_time, det_dist, det_speed = trajectory[:, first_trj_point_indx]
            arrival_time_relative = arrival_time - det_time  # traj will be from t in [0, t_rel]

            # We are looking to minimize the are under the trajectory curve
            # obj func = sum_0^{k-1} b_n t^{n+1}/(n+1) | 0 to t_rel
            # set the obj function coefficients
            self._lp_model.objective.set_linear(zip(
                ["beta_" + str(n) for n in range(self.k)],
                [arrival_time_relative ** n / n for n in range(1, self.k + 1)]))

            # set four rhs values for the equality constraints, and acceleration/decelerations for inequality constrains
            self._lp_model.linear_constraints.set_rhs([("match_det_dist", det_dist),
                                                       ("match_speed", -det_speed),
                                                       ("match_dep_dist", arrival_dist),
                                                       ("match_dep_speed", -dep_speed)] +
                                                      list(zip(["ub_acc_" + str(j) for j in range(self.m)],
                                                               [-amax for j in range(self.m)])) +
                                                      list(zip(["lb_acc_" + str(j) for j in range(self.m)],
                                                               [-amin for j in range(self.m)])))
            # f(t_rel) = sum_{n=0}^{k-1} b_n * t_rel^n
            # set the coefficients matching the departure distance
            dist_coeff = np.array([arrival_time_relative ** n for n in range(self.k)], dtype=float)
            var_name = ["beta_" + str(n) for n in range(self.k)]
            self._lp_model.linear_constraints.set_coefficients(zip(
                ["match_dep_dist" for n in range(self.k)], var_name, dist_coeff))

            # set the coefficients matching the departure speed
            speed_coeff = np.array([n * arrival_time_relative ** (n - 1) for n in range(self.k)])
            self._lp_model.linear_constraints.set_coefficients(
                list(zip(["match_dep_speed" for n in range(self.k)], var_name, speed_coeff)))

            # construct the m control points (exclusive of boundaries)
            control_points = np.linspace(arrival_time_relative / self.m, arrival_time_relative,
                                         self.m, endpoint=False)

            # adjust the coefficients for the control constraints
            for j, time in enumerate(control_points):
                speed_coeff = np.array([n * time ** (n - 1) for n in range(self.k)])
                acc_coeff = np.array([speed_coeff[n] * (n - 1) / time for n in range(self.k)])

                self._lp_model.linear_constraints.set_coefficients(
                    list(zip(["ub_speed_" + str(j) for n in range(self.k)], var_name, speed_coeff)) +
                    list(zip(["lb_speed_" + str(j) for n in range(self.k)], var_name, speed_coeff)) +
                    list(zip(["ub_acc_" + str(j) for n in range(self.k)], var_name, acc_coeff)) +
                    list(zip(["lb_acc_" + str(j) for n in range(self.k)], var_name, acc_coeff))
                )

            return self._lp_model  # should return the model since the follower optimizer is extending this class

    def solve(self, veh, model, arrival_time, max_speed):
        """
        Solves for connected vehicle (both lead and follower)

        :param veh:
        :param model:
        :param arrival_time:
        :param max_speed:
        :return: coefficients of the polynomial to the veh object and trajectory points to the trajectory attribute of it
        """

        if model is None:
            trajectory = veh.trajectory
            first_trj_point_indx = veh.first_trj_point_indx
            det_time, det_dist, det_speed = trajectory[:, first_trj_point_indx]
            arrival_time_relative = arrival_time - det_time  # traj will be from t in [0, t_rel]

            a = veh.max_accel_rate if det_speed <= max_speed else veh.max_decel_rate
            dist_to_max_speed = (max_speed ** 2 - det_speed ** 2) / (2 * a)
            if dist_to_max_speed <= det_dist:
                t_acc_or_dec = (max_speed - det_speed) / a
                d_acc_or_dec = det_dist - (0.5 * a * t_acc_or_dec ** 2 + det_speed * t_acc_or_dec)

                t = self.discretize_time_interval(0, arrival_time_relative)
                d = [det_dist - (0.5 * a * ti ** 2 + det_speed * ti) if ti <= t_acc_or_dec
                     else d_acc_or_dec - max_speed * (ti - t_acc_or_dec) for ti in t]
                s = [min(det_speed + a * ti, max_speed) for ti in t]
            else:  # not enough time and distance to accelerate/decelerate to max speed
                v_dest = np.sqrt(det_speed ** 2 + 2 * a * det_dist)
                t_acc_or_dec = (v_dest - det_speed) / a

                t = self.discretize_time_interval(0, t_acc_or_dec)
                d = [det_dist - (0.5 * a * ti ** 2 + det_speed * ti) for ti in t]
                s = [det_speed + a * ti for ti in t]

            self.set_trajectory(veh, t + det_time, d, s)
        else:
            trajectory = veh.trajectory
            # self._lp_model.write("model.lp")
            model.solve()

            dv = max_speed / 10
            dep_speed = max_speed - dv

            while model.solution.get_status() != 1 and dep_speed >= 0:
                model.linear_constraints.set_rhs([("match_dep_speed", -dep_speed)])
                # self._lp_model.write("model.lp")
                model.solve()

                dep_speed -= dv
            if dep_speed < 0:  # no optimal found in the while loop above
                raise Exception("CPLEX failed to find optimal trj for vehicle " + str(veh.ID))

            beta = np.flip(np.array(model.solution.get_values(["beta_" + str(n) for n in range(self.k)])), 0)
            f = np.poly1d(beta)
            f_prime = np.polyder(f)
            # set the polynomial
            veh.set_poly_coeffs(f)

            first_trj_point_indx = veh.first_trj_point_indx
            det_time = trajectory[0, first_trj_point_indx]

            t, d, s = self.compute_trj_points(f, f_prime, arrival_time - det_time)
            t += det_time

            self.set_trajectory(veh, t, d, s)

    def compute_trj_points(self, f, f_prime, arrival_time_relative):
        t = self.discretize_time_interval(0, arrival_time_relative)
        d = np.polyval(f, t)
        s = np.polyval(-f_prime, t)

        return t, d, s


# -------------------------------------------------------
# FOLLOWER CONNECTED AND AUTOMATED TRAJECTORY OPTIMIZER
# -------------------------------------------------------


class FollowerConnected(LeadConnected):
    HEASWAY_CONTROL_START = 2  # in seconds how frequent need to check for speed, acc/dec rate, and headway
    SAFE_MIN_GAP = 4.8  # minimum safe distance to keep from lead vehicles todo make it dependent to speed

    def __init__(self, max_speed, min_headway, k, m):
        """
        adds the safe headway constraints at the control points to the inherited model.
        :param max_speed:
        :param min_headway:
        :param k:
        :param m:
        """
        super().__init__(max_speed, min_headway, k, m)

        constraint = [["beta_" + str(n) for n in range(self.k)], [1 for n in range(self.k)]]
        self._lp_model.linear_constraints.add(
            lin_expr=[constraint for j in range(self.m)],
            senses=["G" for j in range(self.m)],
            rhs=[min_headway for j in range(self.m)],
            names=["min_headway_" + str(j) for j in range(self.m)])

    def set_model(self, veh, arrival_time, arrival_dist, dep_speed,
                  lead_poly, lead_det_time, lead_arrival_time):
        """
        Sets the LP model using the extra constraints to enforce the safe headway

        :param veh: follower connected vehicle that the trajectory model is constructed for
        :param arrival_time: scheduled arrival time for this vehicle
        :param arrival_dist: scheduled arrival distance for this vehicle
        :param dep_speed: scheduled arrival speed for this vehicle
        :param lead_poly: the lead vehicle polynomial to regenerate necessary info at the control points
        :param lead_det_time: lead vehicle departure time
        :param lead_arrival_time: scheduled arrival time for lead vehicle
        :return: the LP model to be solved by solve() method
        """
        self._lp_model = super().set_model(veh, arrival_time, arrival_dist, dep_speed)

        trajectory = veh.trajectory
        first_trj_point_indx = veh.first_trj_point_indx

        det_time, det_dist, det_speed = trajectory[:, first_trj_point_indx]

        start_relative_ctrl_time, end_relative_ctrl_time = self.HEASWAY_CONTROL_START, lead_arrival_time - det_time
        #  end_relative_ctrl_time is the time lead vehicle leaves relative to the time the follower vehicle was detected
        if end_relative_ctrl_time > start_relative_ctrl_time:
            # trajectories don't overlap over time. No need for min headway constraints
            control_points = np.linspace(start_relative_ctrl_time, end_relative_ctrl_time,
                                         self.m, endpoint=True)
            det_time_diff = det_time - lead_det_time
            min_dist_vec = np.array([np.polyval(lead_poly, control_points[j] + det_time_diff) for j in range(self.m)]) \
                           + self.SAFE_MIN_GAP
            self._lp_model.linear_constraints.set_rhs(zip(["min_headway_" + str(j) for j in range(self.m)],
                                                          min_dist_vec))

            var_name = ["beta_" + str(n) for n in range(self.k)]
            j = 0
            for time in control_points:
                dist_coeff = np.array([time ** n for n in range(self.k)], dtype=float)
                self._lp_model.linear_constraints.set_coefficients(zip(
                    ["min_headway_" + str(j) for n in range(self.k)], var_name, dist_coeff))
                j += 1

        return self._lp_model
