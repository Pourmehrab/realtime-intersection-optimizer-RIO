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

        .. warning:: It is inclusion-wise of the beginning and end of the interval.

        """
        if end_time <= start_time - self.EPS:
            raise Exception('cannot go backward in time')
        elif (end_time - start_time) % self.RES > self.EPS:
            trj_time_stamps = np.append(np.arange(start_time, end_time, Trajectory.RES, dtype=float), end_time)
        else:
            trj_time_stamps = np.arange(start_time, end_time + self.EPS, Trajectory.RES, dtype=float)

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
        det_time, det_dist = trajectory[:2, veh.first_trj_point_indx]

        v = det_dist / (veh.scheduled_departure - det_time)

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
        - Trajectory function: :math:`f(t)   = \sum_{n=0}^{k-1} b_n \\times t^n`
        - Negative of speed profile: :math:`f'(t)  = \sum_{n=1}^{k-1} n \\times b_n \\times t^{n-1}`
        - Negative of acceleration profile: :math:`f''(t) = \sum_{n=2}^{k-1} n \\times (n-1) \\times  b_n \\times t^{n-2}`

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
        self._lp_model.set_log_stream(None)
        self._lp_model.set_error_stream(None)
        self._lp_model.set_warning_stream(None)
        self._lp_model.set_results_stream(None)

        var_name = ["b_" + str(n) for n in range(self.k)]
        self._lp_model.variables.add(obj=[1.0] * self.k, names=var_name, lb=[-cplex.infinity] * self.k)

        constraint = [var_name, [1.0] * self.k]  # default must be 1.0
        self._lp_model.linear_constraints.add(
            lin_expr=
            [[["b_0"], [1.0]], [["b_1"], [1.0]]] + [constraint] * (2 + 4 * self.m),
            senses=["E"] * 4 + ["G"] * self.m + ["L"] * self.m + ["G"] * self.m + ["L"] * self.m,
            rhs=[0.0] * 4 + [-max_speed] * self.m + [0.0] * (3 * self.m),
            names=["det_dist", "det_speed", "dep_dist", "dep_speed"] + ["ub_speed_" + str(j) for j in range(self.m)] + [
                "lb_speed_" + str(j) for j in range(self.m)] + ["ub_acc_" + str(j) for j in range(self.m)] + [
                      "lb_acc_" + str(j) for j in range(self.m)])

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
        departure_time_relative = dep_time - det_time

        self._lp_model.objective.set_linear(zip(
            ["b_" + str(n) for n in range(self.k)], np.array([1 / (n + 1) for n in range(self.k)], dtype=float)))

        self._lp_model.linear_constraints.set_rhs(
            [("det_dist", det_dist), ("det_speed", -det_speed * departure_time_relative), ("dep_dist", dep_dist),
             ("dep_speed", -dep_speed * departure_time_relative)] +
            list(zip(["ub_acc_" + str(j) for j in range(self.m)], [-amax] * self.m)) +
            list(zip(["lb_acc_" + str(j) for j in range(self.m)], [-amin] * self.m)))

        var_name = ["b_" + str(n) for n in range(self.k)]
        self._lp_model.linear_constraints.set_coefficients(list(zip(["dep_speed"] * self.k, var_name,
                                                                    np.arange(self.k, dtype=float))))

        M = self.m + 1
        for j in range(self.m):
            j_prime = j + 1
            speed_coeff = np.array([n * (j_prime / M) ** (n - 1) for n in range(self.k)],
                                   dtype=float) / departure_time_relative
            acc_coeff = np.array([0.0] * 2 + [(n - 1) * speed_coeff[n] for n in range(2, self.k)]) * M / (
                    j_prime * departure_time_relative)
            self._lp_model.linear_constraints.set_coefficients(
                list(zip(["ub_speed_" + str(j)] * self.k, var_name, speed_coeff)) +
                list(zip(["lb_speed_" + str(j)] * self.k, var_name, speed_coeff)) +
                list(zip(["ub_acc_" + str(j)] * self.k, var_name, acc_coeff)) +
                list(zip(["lb_acc_" + str(j)] * self.k, var_name, acc_coeff)))

        return self._lp_model

    def solve(self, veh, lead_veh, model):
        """
        Solves an :term:`LP` model for connected vehicle (both lead and follower)

        :param veh: subject vehicle
        :type veh: Vehicle
        :param lead_veh: lead vehicle which could be `None` if no vehicle is in front.
        :type lead_veh: Vehicle
        :param model:
        :type model: cplex
        :return: coefficients of the polynomial for the ``veh`` object and trajectory points to the trajectory attribute of it
        """
        trajectory = veh.trajectory
        det_time, det_dist, det_speed = trajectory[:, veh.first_trj_point_indx]
        dep_time, dep_dist, dep_speed = trajectory[:, veh.last_trj_point_indx]
        departure_time_relative = dep_time - det_time

        try:
            model.solve()
        except cplex.exceptions.CplexSolverError:
            raise Exception("Exception raised during solve")

        if model.solution.get_status() in {1, }:
            f = np.poly1d(np.flip(np.array(model.solution.get_values(["b_" + str(n) for n in range(self.k)]))
                                  / np.array([departure_time_relative ** n for n in range(self.k)], dtype=float), 0))
            f_prime = np.polyder(f)

            t, d, s = self.compute_trj_points(f, f_prime, dep_time - det_time)
            t += det_time
            veh.set_poly(f, t[0])
        elif lead_veh is None:
            t, d, s = self.optimize_lead_connected_trj(veh, departure_time_relative, dep_speed)
        else:
            t, d, s = self.optimize_follower_connected_trj(veh, lead_veh)  # is defined in the child class

        self.set_trajectory(veh, t, d, s)

    def compute_trj_points(self, f, f_prime, departure_time_relative):
        """
        Converts the polynomial trajectory to the trajectory points
        :param f:
        :param f_prime:
        :param departure_time_relative: span of the trajectory
        :return: t, d, s
        """
        t = self.discretize_time_interval(0, departure_time_relative)
        d = np.polyval(f, t)
        s = np.polyval(-f_prime, t)

        return t, d, s

    def optimize_lead_connected_trj(self, veh, departure_time_relative, dep_speed):
        """
        Computes a bi-linear trajectory for the vehicle

        :param veh: subject vehicle
        :type veh: Vehicle
        :return:
        """
        det_time, det_dist, det_speed = veh.trajectory[:, veh.first_trj_point_indx]
        t_rel_intersect = (det_dist - dep_speed * departure_time_relative) / (det_speed - dep_speed)
        if 0 <= t_rel_intersect <= departure_time_relative:
            t = self.discretize_time_interval(0, departure_time_relative)
            d = [(departure_time_relative - t_i) * dep_speed if t_i >= t_rel_intersect
                 else det_dist - det_speed * t_i for t_i in t]
            s = [dep_speed if t_i >= t_rel_intersect else det_speed for t_i in t]
            t += veh.trajectory[0, veh.first_trj_point_indx]

            return t, d, s
        else:
            raise Exception('look into the matter (intersection time is not within the range)')

    # def optimize_follower_connected_trj(self, veh, lead_veh):
    #     """
    #     A place holder for the optimize_follower_connected_trj() class.
    #
    #     :param veh:
    #     :param lead_veh:
    #     :return:
    #     """
    #     raise NotImplementedError("Must override optimize_follower_connected_trj() in the child class.")


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

        constraint = [["b_" + str(n) for n in range(self.k)], [0.0] * self.k]
        self._lp_model.linear_constraints.add(lin_expr=[constraint] * self.m, senses=["G"] * self.m, rhs=[0.0] * self.m,
                                              names=["min_gap_" + str(j) for j in range(self.m)])

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

        follower_det_time, follower_dep_time = veh.trajectory[0, veh.first_trj_point_indx], veh.trajectory[
            0, veh.last_trj_point_indx]
        lead_dep_time = lead_veh.trajectory[0, lead_veh.last_trj_point_indx]
        start_relative_ctrl_time, end_relative_ctrl_time = self.GAP_CTRL_STARTS, lead_dep_time - follower_det_time
        departure_time_relative = follower_dep_time - follower_det_time

        if end_relative_ctrl_time > start_relative_ctrl_time + self.m * self.EPS:
            n_traj_lead = lead_veh.last_trj_point_indx - lead_veh.first_trj_point_indx + 1
            if n_traj_lead > self.m:
                step = -int((n_traj_lead - 1) / self.m)
                ctrl_lead_relative_time = lead_veh.trajectory[0,
                                          lead_veh.last_trj_point_indx:lead_veh.last_trj_point_indx + step * self.m:step] - \
                                          follower_det_time
                rhs = lead_veh.trajectory[1,
                      lead_veh.last_trj_point_indx:lead_veh.last_trj_point_indx + step * self.m:step] + self.SAFE_MIN_GAP
            else:
                ctrl_lead_relative_time = np.zeros(self.m)
                rhs = np.zeros(self.m) - 1
        else:
            ctrl_lead_relative_time = np.zeros(self.m)
            rhs = np.zeros(self.m) - 1

        self._lp_model.linear_constraints.set_rhs(zip(["min_gap_" + str(j) for j in range(self.m)], rhs))
        var_name = ["b_" + str(n) for n in range(self.k)]
        for j, time in enumerate(ctrl_lead_relative_time):
            dist_coeff = np.array([(time / departure_time_relative) ** n for n in range(self.k)], dtype=float)
            self._lp_model.linear_constraints.set_coefficients(
                zip(["min_gap_" + str(j)] * self.k, var_name, dist_coeff))

        return self._lp_model

    def solve(self, veh, lead_veh, model):
        """

        :param veh: subject vehicle
        :type veh: Vehicle
        :param lead_veh: the vehicle in front of the subject
        :type lead_veh: Vehicle
        :param model:
        :return:
        """
        super().solve(veh, lead_veh, model)

    def optimize_follower_connected_trj(self, veh, lead_veh):
        """
        .. warning::
            This method is called in the parent class (:any:`FollowerConnected`) solve method.

        :param veh: subject vehicle
        :type veh: Vehicle
        :param lead_veh: lead vehicle which could be `None` if no vehicle is in front.
        :type lead_veh: Vehicle
        :return:
        """
        t, d, s = np.copy(lead_veh.trajectory[:, lead_veh.first_trj_point_indx:lead_veh.last_trj_point_indx + 1])
        dep_headway = veh.trajectory[0, veh.last_trj_point_indx] - lead_veh.trajectory[0, lead_veh.last_trj_point_indx]
        t += dep_headway  # to shift the trajectory

        det_time, det_dist = veh.trajectory[:2, veh.first_trj_point_indx]

        dt = t[0] - det_time
        v = (det_dist - d[0]) / dt
        t_augment = self.discretize_time_interval(0, dt)
        d_augment = [det_dist - v * t_i for t_i in t_augment]
        s_augment = [v] * len(t_augment)
        t_augment += det_time

        return map(np.concatenate, [[t_augment[:-1], t], [d_augment[:-1], d], [s_augment[:-1], s]])
