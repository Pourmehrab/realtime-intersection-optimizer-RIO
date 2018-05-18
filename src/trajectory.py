####################################
# File name: trajectory.py         #
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
    Is the abstract class for computing the trajectory points. Four subclasses inherited from this parent class:
        - :any:`LeadConventional`
        - :any:`FollowerConnected`
        - :any:`LeadConnected`
        - :any:`FollowerConventional`

    Any solve method under each class shall invoke :any:`set_trajectory` method at the end or does the assignment in-place.

    .. note:: If want to limit the trajectory planning, there are two options:
            - If a particular vehicle is intended to be skipped, simply set ``vehicle.reschedule_departure`` to ``False``
            - If the whole simulation is intended to be run without trajectory planer, set ``vehicle.reschedule_departure`` in ``main.py`` to False.


    :param RES: time difference between two consecutive trajectory points in seconds used in :any:`discretize_time_interval()` (be careful not to exceed max size of trajectory)
    :param EPS: small number that lower than that is approximated by zero

    :Author:
        Mahmoud Pourmehrab <pourmehrab@gmail.com>
    :Date:
        April-2018
    """

    RES = 1
    EPS = 0.01

    def __init__(self, max_speed, min_headway):
        """
        :param max_speed: Trajectories are designed to respect the this speed limit (in :math:`m/s`).
        :param min_headway: This is the minimum headway that vehicles in a lane can be served (in :math:`sec/veh`)
        """
        self._max_speed = max_speed
        self._min_headway = min_headway

    def discretize_time_interval(self, start_time, end_time):
        """
        Discretize the given time interval to a numpy array of time stamps

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

        .. note:: An assigned trajectory always is indexed from zero as the ``veh.set_first_trj_point_indx``.

        :param veh: the vehicle object that is owns the trajectory
        :type veh: Vehicle
        :param t: time stamps (seconds from the reference time)
        :param d: distances at each time stamp (in meters from the stop bar)
        :param s: speed at each time stamp (in :math:`m/s`)
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

Use Case:

    Instantiate like::

        $ lead_conventional_trj_estimator = LeadConventional(.)

    Perform trajectory computation by::

        $ lead_conventional_trj_estimator.solve(veh)


:Author:
    Mahmoud Pourmehrab <pourmehrab@gmail.com>
:Date:
    April-2018
    """

    def __init__(self, max_speed, min_headway):
        super().__init__(max_speed, min_headway)

    def solve(self, veh):
        """
        Constructs the trajectory of a lead conventional vehicle assuming the driver maintains its speed

        :param veh: the lead conventional vehicle
        :type veh: Vehicle
        """
        trajectory = veh.trajectory
        first_trj_point_indx = veh.first_trj_point_indx
        det_time, det_dist = trajectory[:2, first_trj_point_indx]

        v = det_dist / veh.scheduled_departure

        t = self.discretize_time_interval(det_time, veh.scheduled_departure)
        s = np.array([v] * len(t))
        d = np.array([det_dist - v * (t_i - det_time) for t_i in t])

        self.set_trajectory(veh, t, d, s)

        # -------------------------------------------------------
        # FOLLOWER CONVENTIONAL TRAJECTORY ESTIMATOR
        # -------------------------------------------------------


class FollowerConventional(Trajectory):
    """
    Estimates the trajectory for a follower conventional vehicle assuming a car following model. In the current implementation, Gipps car-following model [#]_ is used.

Use Case:

    Instantiate like::

        $ follower_conventional_trj_estimator = FollowerConventional(.)

    Perform trajectory computation by::

        $ follower_conventional_trj_estimator.solve(veh, .)

    :Author:
        Mahmoud Pourmehrab <pourmehrab@gmail.com>
    :Date:
        April-2018

.. [#] Gipps, Peter G. *A behavioural car-following model for computer simulation*. Transportation Research Part B: Methodological 15.2 (1981): 105-111 (`link <https://www.sciencedirect.com/science/article/pii/0191261581900370>`_).
    """

    def __init__(self, max_speed, min_headway):
        super().__init__(max_speed, min_headway)

    def solve(self, veh, lead_veh):
        """
        Gipps car following model is implemented. It is written in-place (does not call :any:`set_trajectory`)

        .. figure:: images/Gipps_formula.JPG
            :align: center
            :alt: map to buried treasure

            Gipps car following formula.

        .. note::
            - The only trajectory point index that changes is follower's last one.
            - This method relies on the fact that lead vehicle's first trajectory point is current.
            - Assumed the gap to lead vehicle cannot get lower than half length of the lead vehicle.

        :param veh: The follower conventional vehicle
        :type veh: Vehicle
        :param lead_veh: The vehicle in front of subject conventional vehicle
        :type lead_veh: Vehicle
        """
        follower_trajectory = veh.trajectory
        follower_desired_speed = veh.desired_speed
        follower_max_acc, follower_max_dec = veh.max_accel_rate, veh.max_decel_rate
        follower_trj_indx = veh.first_trj_point_indx

        lead_trajectory = lead_veh.trajectory
        lead_max_dec, lead_length = lead_veh.max_decel_rate, lead_veh.length
        lead_trj_indx = lead_veh.first_trj_point_indx
        lead_last_trj_point_indx = lead_veh.last_trj_point_indx

        if lead_trajectory[0, lead_trj_indx] - follower_trajectory[0, follower_trj_indx] <= self.EPS:
            lead_trj_indx += 1  # this shifts appropriately the trajectory computation

        while lead_trj_indx <= lead_last_trj_point_indx:
            next_trj_indx = follower_trj_indx + 1
            lead_speed = lead_trajectory[2, lead_trj_indx]
            follower_speed = follower_trajectory[2, follower_trj_indx]
            gap = follower_trajectory[1, follower_trj_indx] - lead_trajectory[1, lead_trj_indx]
            dt = lead_trajectory[0, lead_trj_indx] - follower_trajectory[0, follower_trj_indx]
            if lead_speed <= self.EPS:  # Gipps doesn't work well for near zero speed
                v_get_behind = (gap - lead_length) / dt
                v = min(v_get_behind, follower_desired_speed) if v_get_behind >= 0 else 0.0
                follower_trajectory[0, next_trj_indx] = lead_trajectory[0, lead_trj_indx]
                follower_trajectory[1, next_trj_indx] = follower_trajectory[1, follower_trj_indx] - v * dt
                follower_trajectory[2, next_trj_indx] = 0.0

            else:
                v1 = follower_speed + 2.5 * follower_max_acc * dt * (
                        1 - follower_speed / follower_desired_speed) * np.sqrt(
                    1 / 40 + follower_speed / follower_desired_speed)

                s2 = (follower_max_dec * (lead_max_dec * (2 * (lead_length - gap) + dt * (
                        follower_max_dec * dt + follower_speed)) + lead_speed ** 2)) / lead_max_dec

                if s2 >= 0:  # determines followers next speed
                    v2 = follower_max_dec * dt + np.sqrt(s2)  # might get negative
                    v = min(v1, v2) if v2 >= 0 else v1
                else:
                    v = v1

            follower_trajectory[0, next_trj_indx] = lead_trajectory[0, lead_trj_indx]
            follower_trajectory[2, next_trj_indx] = v
            a = (v - follower_speed) / dt
            follower_dist_to_stopbar = follower_trajectory[1, follower_trj_indx] - (
                    a * (follower_trajectory[0, next_trj_indx] ** 2 - follower_trajectory[
                0, follower_trj_indx] ** 2) / 2 + (follower_trajectory[2, follower_trj_indx] -
                                                   a * follower_trajectory[0, follower_trj_indx]) * dt)
            follower_trajectory[1, next_trj_indx] = max(follower_dist_to_stopbar, lead_trajectory[1, lead_trj_indx] +
                                                        lead_length / 2)

            follower_trj_indx += 1
            lead_trj_indx += 1

        # This part adds the end part of follower trajectory that might left out of the domain
        d_follower_end = follower_trajectory[1, follower_trj_indx]
        t_follower_end = follower_trajectory[0, follower_trj_indx]

        t_departure_relative = veh.scheduled_departure - t_follower_end
        if t_departure_relative > self.EPS:
            v_departure_relative = d_follower_end / t_departure_relative

            t_augment = self.discretize_time_interval(self.RES, t_departure_relative)
            d_augment = [d_follower_end - t * v_departure_relative for t in t_augment]
            v_augment = [v_departure_relative for t in t_augment]
        else:
            t_augment = []
            d_augment = []
            v_augment = []

        last_index = follower_trj_indx + len(t_augment) + 1
        follower_trajectory[0, follower_trj_indx + 1:last_index] = t_augment + t_follower_end
        follower_trajectory[1, follower_trj_indx + 1:last_index] = d_augment
        follower_trajectory[2, follower_trj_indx + 1:last_index] = v_augment
        veh.set_last_trj_point_indx(last_index - 1)


import cplex


# -------------------------------------------------------
# LEAD CONNECTED AND AUTOMATED TRAJECTORY OPTIMIZER
# -------------------------------------------------------

class LeadConnected(Trajectory):
    """
    .. note::
        - Trajectory function: :math:`f(t)   = \sum_{n=0}^{k-1} \\beta_n \\times t^n`
        - Negative of speed profile: :math:`f'(t)  = \sum_{n=1}^{k-1} n \\times \\beta_n \\times t^{n-1}`
        - Negative of acceleration profile: :math:`f''(t) = \sum_{n=2}^{k-1} n \\times (n-1) \\times  \\beta_n \\times t^{n-2}`

    :param NUM_DIGS: The accuracy to keep decimals
    :param SPEED_DECREMENT_SIZE: The final speed decrements from maximum to 0 by step-size defined by maximum speed divided by this

    Use Case:

        Instantiate like::

            $ lead_connected_trj_optimizer = LeadConnected(.)

        Perform trajectory computation by::

            $ lead_conventional_trj_estimator.solve(veh)

    :Author:
        Mahmoud Pourmehrab <pourmehrab@gmail.com>
    :Date:
        April-2018
    """
    NUM_DIGS = 3
    SPEED_DECREMENT_SIZE = 10

    def __init__(self, max_speed, min_headway, k, m):
        """
         Objectives:
            - Sets :math:`k, m` values
            - Sets the optimization direction
            - Adds variables (coefficients of the polynomial that'll represent the trajectory)
            - Adds two constraints that fixes the detected distance/speed on the polynomial
            - Adds generic form constraints with zero coefficients

        :param max_speed:
        :param min_headway:
        :param k: the size of array that keeps the polynomial (:math:`k-1` is the degree of polynomial)
        :param k: int
        :param m: number of points (exclusive of boundaries) to control speed/acceleration
        :param m: int
        """
        super().__init__(max_speed, min_headway)

        self.k, self.m = k, m

        self._lp_model = cplex.Cplex()
        self._lp_model.objective.set_sense(self._lp_model.objective.sense.minimize)
        self._lp_model.set_problem_name('CAV trajectory optimization')
        self._lp_model.set_problem_type(cplex.Cplex().problem_type.LP)
        self._lp_model.parameters.read.datacheck.set(2)
        # self._lp_model.set_log_stream(None)
        # self._lp_model.set_error_stream(None)
        # self._lp_model.set_warning_stream(None)
        # self._lp_model.set_results_stream(None)

        var_name = ["beta_" + str(n) for n in range(self.k)]
        self._lp_model.variables.add(obj=[1.0] * self.k, names=var_name, lb=[-cplex.infinity] * self.k)

        constraint = [var_name, [0.0] * self.k]
        self._lp_model.linear_constraints.add(
            lin_expr=
            [[["beta_0"], [1.0]], [["beta_1"], [1.0]]] + [constraint] * (2 + 4 * self.m),
            senses=["E"] * 4 + ["G"] * self.m + ["L"] * self.m + ["G"] * self.m + ["L"] * self.m,
            rhs=[0.0] * 4 + [-max_speed] * self.m + [0.0] * (3 * self.m),
            names=["match_det_dist", "match_det_speed", "match_dep_dist", "match_dep_speed"] + \
                  ["ub_speed_" + str(j) for j in range(self.m)] + \
                  ["lb_speed_" + str(j) for j in range(self.m)] + \
                  ["ub_acc_" + str(j) for j in range(self.m)] + \
                  ["lb_acc_" + str(j) for j in range(self.m)])

    def set_model(self, veh):

        """
        Overrides the generic coefficients to build the specific :term:`LP` model for the AV trajectory.

        This model solves an :term:`LP` model to compute trajectory of AVs.

        .. figure:: images/model.JPG
           :width: 10cm
           :align: center
           :alt: map to buried treasure

           Part of a sample CPLEX model.


        :param veh: vehicle object that its trajectory is meant to be computed
        :type veh: Vehicle
        :return: cplex LP model. Should return the model since the follower optimizer adds constraints to this model
        """
        dep_time, dep_dist, dep_speed = veh.trajectory[:, veh.last_trj_point_indx]

        trajectory = veh.trajectory
        amin, amax = veh.max_decel_rate, veh.max_accel_rate

        first_trj_point_indx = veh.first_trj_point_indx
        det_time, det_dist, det_speed = trajectory[:, first_trj_point_indx]
        arrival_time_relative = dep_time - det_time

        self._lp_model.objective.set_linear(zip(
            ["beta_" + str(n) for n in range(self.k)],
            [arrival_time_relative ** n / n for n in range(1, self.k + 1)]))

        self._lp_model.linear_constraints.set_rhs(
            [("match_det_dist", det_dist), ("match_det_speed", -det_speed), ("match_dep_dist", dep_dist),
             ("match_dep_speed", -dep_speed)] + list(
                zip(["ub_acc_" + str(j) for j in range(self.m)], [-amax] * self.m)) + list(
                zip(["lb_acc_" + str(j) for j in range(self.m)], [-amin] * self.m)))

        var_name = ["beta_" + str(n) for n in range(self.k)]
        self._lp_model.linear_constraints.set_coefficients(
            zip(["match_dep_dist"] * self.k, var_name,
                np.array([arrival_time_relative ** n for n in range(self.k)], dtype=float)))

        self._lp_model.linear_constraints.set_coefficients(
            list(zip(["match_dep_speed"] * self.k, var_name,
                     np.array([n * arrival_time_relative ** (n - 1) for n in range(self.k)]))))

        control_points = np.linspace(arrival_time_relative / self.m, arrival_time_relative, self.m, endpoint=False)
        for j, time in enumerate(control_points):
            speed_coeff = np.array([n * time ** (n - 1) for n in range(self.k)])
            acc_coeff = np.array([speed_coeff[n] * (n - 1) for n in range(self.k)]) / time
            self._lp_model.linear_constraints.set_coefficients(
                list(zip(["ub_speed_" + str(j)] * self.k, var_name, speed_coeff)) +
                list(zip(["lb_speed_" + str(j)] * self.k, var_name, speed_coeff)) +
                list(zip(["ub_acc_" + str(j)] * self.k, var_name, acc_coeff)) +
                list(zip(["lb_acc_" + str(j)] * self.k, var_name, acc_coeff)))

        return self._lp_model

    def solve(self, veh, model):
        """
        Solves an :term:`LP` model for connected vehicle (both lead and follower)

        :param veh:
        :type veh: Vehicle
        :param model:
        :type model: cplex
        :return: coefficients of the polynomial for the ``veh`` object and trajectory points to the trajectory attribute of it
        """
        trajectory = veh.trajectory
        det_time = trajectory[0, veh.first_trj_point_indx]
        dep_time, dep_dist, dep_speed = trajectory[:, veh.last_trj_point_indx]

        try:
            model.solve()
        except cplex.exceptions.CplexSolverError:
            print("Exception raised during solve")
            return

        dv = trajectory[2, veh.last_trj_point_indx] / self.SPEED_DECREMENT_SIZE
        dep_speed = trajectory[2, veh.last_trj_point_indx] - dv

        while model.solution.get_status() not in {1, } and dep_speed >= 0:
            model.linear_constraints.set_rhs([("match_dep_speed", -dep_speed)])
            try:
                model.solve()
            except CplexSolverError:
                print("Exception raised during solve")
                return
            dep_speed -= dv

        if dep_speed < 0:  # no optimal found in the while loop above
            model.write("model.lp")
            stat = model.get_stats()
            s = model.solution.get_linear_slacks()
            raise Exception("CPLEX failed to find optimal trajectory for vehicle " + str(veh.ID))

        beta = np.flip(np.array(model.solution.get_values(["beta_" + str(n) for n in range(self.k)])), 0)
        f = np.poly1d(beta)
        f_prime = np.polyder(f)

        t, d, s = self.compute_trj_points(f, f_prime, dep_time - det_time)
        t += det_time

        self.set_trajectory(veh, t, d, s)

        veh.set_poly(f, t[0])

    def compute_trj_points(self, f, f_prime, arrival_time_relative):
        t = self.discretize_time_interval(0, arrival_time_relative)
        d = np.polyval(f, t)
        s = np.polyval(-f_prime, t)

        return t, d, s


# -------------------------------------------------------
# FOLLOWER CONNECTED AND AUTOMATED TRAJECTORY OPTIMIZER
# -------------------------------------------------------


class FollowerConnected(LeadConnected):
    """
    Optimizes the trajectory of a follower CAV.

    Use Case:

        Instantiate like::

            $ follower_connected_trj_optimizer = FollowerConnected(.)

        Perform trajectory computation by::

            $ model = follower_connected_trj_optimizer.set_model(.)
            $ follower_connected_trj_optimizer.solve(veh, .)

    :param GAP_CTRL_STARTS: This is the relative time when gap control constraints get added
    :param SAFE_MIN_GAP: The minimum safe distance to keep from lead vehicles (in :math:`m`) [*can be speed dependent*]

    :Author:
        Mahmoud Pourmehrab <pourmehrab@gmail.com>
    :Date:
        April-2018
    """
    GAP_CTRL_STARTS = 2.0
    SAFE_MIN_GAP = 4.8

    def __init__(self, max_speed, min_headway, k, m):
        """
        Adds the safe headway constraints at the control points to the inherited model.

        :param max_speed:
        :param min_headway:
        :param k:
        :param m:

        """
        super().__init__(max_speed, min_headway, k, m)

        constraint = [["beta_" + str(n) for n in range(self.k)], [0.0] * self.k]
        self._lp_model.linear_constraints.add(lin_expr=[constraint] * self.m, senses=["G"] * self.m, rhs=[0.0] * self.m,
                                              names=["min_headway_" + str(j) for j in range(self.m)])

    def set_model(self, veh, lead_veh):
        """
        Sets the LP model using the extra constraints to enforce the safe headway. Either three cases happen here:
            - The lead is a :term:`CNV` and its trajectory overlaps and it has enough trajectory points
                - Enforce the constraint on last m trajectory points of the lead vehicle
            - The lead is a :term:`CAV` and its trajectory overlaps
                - Evaluate the polynomial over m points as defined in the paper
            - Otherwise
                - Relax the constraints (:math:`0.0 \geq -1.0` is always true)

        :param veh: follower connected vehicle that the trajectory model is constructed for
        :type veh: Vehicle
        :param lead_veh: the vehicle in front
        :type lead_veh: Vehicle
        :return: the cplex LP model to be solved by solve() method
        """

        self._lp_model = super().set_model(veh)

        follower_det_time = veh.trajectory[0, veh.first_trj_point_indx]
        lead_dep_time = lead_veh.trajectory[0, lead_veh.last_trj_point_indx]

        start_relative_ctrl_time, end_relative_ctrl_time = self.GAP_CTRL_STARTS, lead_dep_time - follower_det_time
        if end_relative_ctrl_time > start_relative_ctrl_time + self.m * self.EPS:
            n_traj_lead = lead_veh.last_trj_point_indx - lead_veh.first_trj_point_indx + 1
            # if lead_veh.veh_type == 1:
            #     ctrl_points_relative_time = np.linspace(start_relative_ctrl_time, end_relative_ctrl_time, self.m,
            #                                             endpoint=True)
            #     ctrl_points_relative_time_lead = ctrl_points_relative_time + follower_det_time - lead_veh.poly[
            #         'ref time']
            #     rhs = np.array([np.polyval(lead_veh.poly['coeffs'], ctrl_points_relative_time_lead[j]) for j in
            #                     range(self.m)]) + self.SAFE_MIN_GAP
            if n_traj_lead >= self.m:
                step = -int(n_traj_lead / self.m)

                ctrl_points_relative_time = lead_veh.trajectory[0,
                                            lead_veh.last_trj_point_indx:lead_veh.last_trj_point_indx + step * self.m:step]
                rhs = lead_veh.trajectory[1,
                      lead_veh.last_trj_point_indx:lead_veh.last_trj_point_indx + step * self.m:step] + self.SAFE_MIN_GAP
            else:
                ctrl_points_relative_time = np.zeros(self.m)
                rhs = np.zeros(self.m) - 1
        else:
            ctrl_points_relative_time = np.zeros(self.m)
            rhs = np.zeros(self.m) - 1

        self._lp_model.linear_constraints.set_rhs(zip(["min_headway_" + str(j) for j in range(self.m)], rhs))
        var_name, j = ["beta_" + str(n) for n in range(self.k)], 0
        for time in ctrl_points_relative_time:
            dist_coeff = np.array([time ** n for n in range(self.k)], dtype=float)
            self._lp_model.linear_constraints.set_coefficients(
                zip(["min_headway_" + str(j) for n in range(self.k)], var_name, dist_coeff))
            j += 1

        return self._lp_model
