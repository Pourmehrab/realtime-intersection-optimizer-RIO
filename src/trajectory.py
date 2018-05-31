####################################
# File name: trajectory.py         #
# Author: Mahmoud Pourmehrab       #
# Email: pourmehrab@gmail.com      #
# Last Modified: May/30/2018       #
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

    :Author:
        Mahmoud Pourmehrab <pourmehrab@gmail.com>
    :Date:
        April-2018
    """

    def __init__(self, intersection):
        """
        :param max_speed: Trajectories are designed to respect the this speed limit (in :math:`m/s`).
        :param min_headway: This is the minimum headway that vehicles in a lane can be served (in :math:`sec/veh`)
        """
        self._max_speed, self._min_headway, self._small_positive_num, self._trj_time_resolution = map(
            intersection._general_params.get, ['max_speed', 'min_headway', 'small_positive_num', 'trj_time_resolution'])

    def discretize_time_interval(self, start_time, end_time):
        """
        Discretizes the given time interval at the granularity level of `trj_time_resolution` to an array of time stamps.


        .. warning:: It is inclusion-wise of the beginning and end of the interval.


        :Author:
            Mahmoud Pourmehrab <pourmehrab@gmail.com>
        :Date:
            April-2018
        """
        assert end_time > start_time - self._small_positive_num, "Cannot Go Backward in Time"

        if (end_time - start_time) % self._trj_time_resolution > self._small_positive_num:
            trj_time_stamps = np.append(np.arange(start_time, end_time, self._trj_time_resolution, dtype=float),
                                        end_time)
        else:
            trj_time_stamps = np.arange(start_time, end_time + self._small_positive_num, self._trj_time_resolution,
                                        dtype=float)
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
        

        :Author:
            Mahmoud Pourmehrab <pourmehrab@gmail.com>
        :Date:
            April-2018
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

        >>> lead_conventional_trj_estimator = LeadConventional(intersection)

    Perform trajectory computation by::

        >>> lead_conventional_trj_estimator.solve(veh)


    :Author:
        Mahmoud Pourmehrab <pourmehrab@gmail.com>
    :Date:
        April-2018
    """

    def __init__(self, intersection):
        super().__init__(intersection)

    def solve(self, veh):
        """
        Constructs the trajectory of a lead conventional vehicle assuming the driver maintains its speed

        :param veh: the lead conventional vehicle
        :type veh: Vehicle

        .. warning::
            Make sure the assumptions here are compatible with those in :any:`earliest_arrival_conventional`

        :Author:
            Mahmoud Pourmehrab <pourmehrab@gmail.com>
        :Date:
            April-2018
        """
        det_time, det_dist, _ = veh.get_arrival_schedule()
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

        >>> follower_conventional_trj_estimator = FollowerConventional(intersection)

    Perform trajectory computation by::

        >>> follower_conventional_trj_estimator.solve(veh, lead_veh)

    :Author:
        Mahmoud Pourmehrab <pourmehrab@gmail.com>
    :Date:
        April-2018

    .. [#] Gipps, Peter G. *A behavioural car-following model for computer simulation*. Transportation Research Part B: Methodological 15.2 (1981): 105-111 (`link <https://www.sciencedirect.com/science/article/pii/0191261581900370>`_).
    """

    def __init__(self, intersection):
        super().__init__(intersection)

    @staticmethod
    def wiedemann99(lead_d, lead_s, lead_a, lead_l, foll_d, foll_s, foll_s_des, cc0=1.50 * 0.9, cc1=1.30 * 0.9,
                    cc2=4.00 * 2, cc3=-12.00, cc4=-0.25 * 6, cc5=0.35 * 6, cc6=6.00 / 10 ** 4, cc7=0.25, cc8=2.00,
                    cc9=1.50):
        """

        :param lead_d: lead vehicle distance to stop bar
        :param lead_s: lead vehicle speed
        :param lead_a: lead vehicle acceleration rate
        :param lead_l: length of lead vehicle
        :param foll_d: follower vehicle distance to stop bar
        :param foll_s: follower vehicle speed
        :param foll_s_des: follower vehicle desired speed
        :param cc0: Standstill Distance in :math:`m`
        :param cc1: Spacing Time in :math:`s`
        :param cc2: Following Variation (*max drift*) in :math:`m`
        :param cc3: Threshold for Entering 'Following' in :math:`s`
        :param cc4: Negative *Following* Threshold in :math:`m/s`
        :param cc5: Positive *Following* Threshold in :math:`m/s`
        :param cc6: Speed Dependency of Oscillation in :math:`10^-4 rad/s`
        :param cc7: Oscillation Acceleration in :math:`m/s^2`
        :param cc8: Standstill Acceleration in :math:`m/s^2`
        :param cc9: Acceleration at 80 :math:`km/h` in :math:`m/s^2`
        :return: follower next acceleration rate

        :Author:
            Mahmoud Pourmehrab <pourmehrab@gmail.com>
        :Date:
            May-2018
        """
        dx = foll_d - lead_d - lead_l
        dv = lead_s - foll_s
        if lead_s <= 0:
            sdxc = cc0
        else:
            v_slower = foll_s if ((dv >= 0) or (lead_a < -1.0)) else lead_s + dv * np.random.uniform(-0.5, 0.5)
            sdxc = cc0 + cc1 * v_slower
        sdxo = sdxc + cc2
        sdxv, sdv = sdxo + cc3 * (dv - cc4), cc6 * dx ** 2
        sdvc = cc4 - sdv if lead_s > 0 else 0.0
        sdvo = sdv + cc5 if foll_s > cc5 else sdv
        a = 0.0
        flag = False
        if (dv < sdvo) and (dx <= sdxc):
            a = 0.0
            if foll_s > 0:
                if dv < 0:
                    a = min(lead_a + dv * dv / (cc0 - dx), a) if dx > cc0 else min(lead_a + 0.5 * (dv - sdvo), a)
                a = -cc7 if a > -cc7 else max(a, -10.0 + 0.5 * np.sqrt(foll_s))
        elif (dv < sdvc) and (dx < sdxv):
            a = max(0.5 * dv ** 2 / (-dx + sdxc - 0.1), -10.0)
        elif (dv < sdvo) and (dx < sdxo):
            a = min(a, -cc7) if a <= 0 else min(max(a, cc7), foll_s_des - foll_s)
        else:
            flag = True
            if dx > sdxc:
                if flag:
                    a = cc7
                else:
                    a_max = cc8 + cc9 * min(foll_s, 80 * 1000 / 3600) + np.random.uniform(0, 1)
                    a = min(dv * dv / (sdxo - dx), a_max) if dx < sdxo else a_max
                a = min(a, foll_s_des - foll_s)
        return a

    def gipps(self, lead_d, lead_s, lead_a, lead_l, foll_d, foll_s, foll_s_des, foll_a_min, foll_a_max, lead_a_min, dt):
        """
        Gipps car following model is implemented. It is written in-place (does not call :any:`set_trajectory`)

        .. figure:: images/Gipps_formula.JPG
            :align: center
            :alt: map to buried treasure

            Gipps car following formula.

        .. warning ::
            Theoretically, caution needed to address the cases where either the term under the square root or one of the
             speed values in the model becomes negative.

        :param lead_d: lead vehicle distance to stop bar
        :param lead_s: lead vehicle speed
        :param lead_a: lead vehicle acceleration
        :param lead_l: lead vehicle length
        :param foll_d: follower vehicle distance to stop bar
        :param foll_s: follower vehicle speed
        :param foll_s_des: follower desired speed
        :param foll_a_min: follower maximum deceleration rate
        :param foll_a_max: follower maximum acceleration rate
        :param lead_a_min: lead maximum deceleration rare
        :param dt: length of time interval the acceleration shall be calculated
        :return: follower next acceleration rate

        :Author:
            Mahmoud Pourmehrab <pourmehrab@gmail.com>
        :Date:
            April-2018
        """
        gap = foll_d - lead_d
        if lead_s <= self._small_positive_num:  # Gipps doesn't work well for near zero speed
            s_get_behind = (gap - lead_l) / dt
            next_foll_s = min(s_get_behind, foll_s_des) if s_get_behind >= 0 else 0.0
        else:
            s_gipps_1 = foll_s + 2.5 * foll_a_max * dt * (1 - foll_s / foll_s_des) * np.sqrt(
                1 / 40 + foll_s / foll_s_des)
            under_sqrt = (foll_a_min * (
                    lead_a_min * (2 * (lead_l - gap) + dt * (foll_a_min * dt + foll_s)) + lead_s ** 2)) / lead_a_min
            if under_sqrt >= 0:
                s_gipps_2 = foll_a_min * dt + np.sqrt(under_sqrt)  # might get negative
                next_foll_s = min(s_gipps_1, s_gipps_2) if s_gipps_2 >= 0 else s_gipps_1
            else:
                next_foll_s = s_gipps_1

        return (next_foll_s - foll_s) / dt

    def solve(self, veh, lead_veh):
        """

        .. note::
            - The only trajectory point index that changes is follower's last one.
            - This method relies on the fact that lead vehicle's first trajectory point is current.
            - Assumed the gap to lead vehicle cannot get lower than half length of the lead vehicle.
            - Compared to W99 requires acc/dec on follower, dec on lead, dt, and does not need lead acc.

        :param veh: The follower conventional vehicle
        :type veh: Vehicle
        :param lead_veh: The vehicle in front of subject conventional vehicle
        :type lead_veh: Vehicle

        :Author:
            Mahmoud Pourmehrab <pourmehrab@gmail.com>
        :Date:
            April-2018
        """
        start_lead_trj_indx = lead_veh.first_trj_point_indx + 1  # skip the first point to compute acceleration rate
        max_lead_traj_indx = lead_veh.last_trj_point_indx + 1
        foll_trj_indx = veh.first_trj_point_indx + 1
        lead_l, foll_s_des = lead_veh.length, veh.desired_speed
        curr_lead_t, curr_lead_d, curr_lead_s = lead_veh.get_arrival_schedule()
        curr_foll_t, curr_foll_d, curr_foll_s = veh.get_arrival_schedule()
        for lead_trj_indx in range(start_lead_trj_indx, max_lead_traj_indx):
            next_lead_t, next_lead_d, next_lead_s = lead_veh.trajectory[:, lead_trj_indx]
            lead_a = (next_lead_s - curr_lead_s) / (next_lead_t - curr_lead_t)
            # foll_a = self.wiedemann99(next_lead_d, next_lead_s, lead_a, lead_l, curr_foll_d, curr_foll_s, foll_s_des)
            foll_a = self.gipps(next_lead_d, next_lead_s, lead_a, lead_l, curr_foll_d, curr_foll_s, foll_s_des,
                                veh.max_decel_rate, veh.max_accel_rate, lead_veh.max_decel_rate,
                                next_lead_t - curr_foll_t)
            next_foll_t = next_lead_t
            next_foll_d, next_foll_s = self.comp_speed_distance(curr_foll_t, curr_foll_d, curr_foll_s, foll_a,
                                                                next_foll_t, veh.max_decel_rate, veh.max_accel_rate)
            assert next_foll_d > next_lead_d, "lead vehicle is made of solid; follower cannot pass through it"
            veh.trajectory[:, foll_trj_indx] = [next_foll_t, next_foll_d, next_foll_s]
            curr_foll_t, curr_foll_d, curr_foll_s, curr_lead_t, curr_lead_d, curr_lead_s = next_foll_t, next_foll_d, next_foll_s, next_lead_t, next_lead_d, next_lead_s
            foll_trj_indx += 1

        t_departure_relative = veh.scheduled_departure - curr_foll_t
        v_departure_relative = curr_foll_d / t_departure_relative
        assert 0 <= v_departure_relative <= veh.desired_speed, "the scheduled departure was too early or car following yielded slow speeds"
        t_augment = self.discretize_time_interval(self._trj_time_resolution, t_departure_relative)
        d_augment = [curr_foll_d - t * v_departure_relative for t in t_augment]
        v_augment = [v_departure_relative] * len(t_augment)
        last_index = foll_trj_indx + len(t_augment)
        veh.trajectory[:, foll_trj_indx:last_index] = t_augment + curr_foll_t, d_augment, v_augment
        veh.set_last_trj_point_indx(last_index - 1)

    @staticmethod
    def comp_speed_distance(t0, d0, v0, a, t, foll_a_min, foll_a_max):
        """
        If car-following models yielded unreasonable acceleration, fixes it.

        .. note::
            - Speed should be positive
            - Acceleration/deceleration constraints should be met.

        :param t0: the time at the beginning of the small interval that acceleration is constant
        :param d0: the distance at the beginning of the interval
        :param v0: the speed at the beginning of the interval
        :param a: the constant acceleration rate within the interval
        :param t: the end time of the interval
        :return: distance to stop bar and speed

        :Author:
            Mahmoud Pourmehrab <pourmehrab@gmail.com>
        :Date:
            April-2018
        """
        dt = t - t0
        d = d0 - (a * (t ** 2 - t0 ** 2) / 2 + (v0 - a * t0) * dt)
        s = a * dt + v0
        if s < 0:
            s, a_star = 0, -v0 / dt
            d = d0 - (a_star * (t ** 2 - t0 ** 2) / 2 + (v0 - a_star * t0) * dt)
            if foll_a_min > a_star:
                a_star = foll_a_min
                s, d = a_star * dt + v0, d0 - (a_star * (t ** 2 - t0 ** 2) / 2 + (v0 - a_star * t0) * dt)
            elif a_star > foll_a_max:
                a_star = foll_a_max
                s, d = a_star * dt + v0, d0 - (a_star * (t ** 2 - t0 ** 2) / 2 + (v0 - a_star * t0) * dt)

        return d, s


import cplex


# -------------------------------------------------------
# LEAD CONNECTED AND AUTOMATED TRAJECTORY OPTIMIZER
# -------------------------------------------------------

class LeadConnected(Trajectory):
    """
    .. note::
        - Trajectory function: :math:`f(t)   = \sum_{n=0}^{k-1} b_n \\times (t/t_0)^n`
        - Negative of speed profile: :math:`f'(t)  = \sum_{n=1}^{k-1} n \\times b_n \\times (t/t_0)^{n-1}`
        - Negative of acceleration profile: :math:`f''(t) = \sum_{n=2}^{k-1} n \\times (n-1) \\times  b_n \\times (t/t_0)^{n-2}`


    Use Case:

        Instantiate like::

            >>> lead_connected_trj_optimizer = LeadConnected(.)

        Perform trajectory computation by::

            >>> lead_conventional_trj_estimator.solve(veh)

    :Author:
        Mahmoud Pourmehrab <pourmehrab@gmail.com>
    :Date:
        April-2018
    """

    def __init__(self, intersection):
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

        :Author:
            Mahmoud Pourmehrab <pourmehrab@gmail.com>
        :Date:
            April-2018
        """
        super().__init__(intersection)

        self.k, self.m = map(intersection._general_params.get, ['k', 'm'])

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
            rhs=[0.0] * 4 + [-intersection._general_params.get('max_speed')] * self.m + [0.0] * (3 * self.m),
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
        :return: CPLEX LP model. Should return the model since the follower optimizer adds constraints to this model
        :Author:
            Mahmoud Pourmehrab <pourmehrab@gmail.com>
        :Date:
            April-2018
        """

        amin, amax = veh.max_decel_rate, veh.max_accel_rate
        det_time, det_dist, det_speed = veh.get_arrival_schedule()
        dep_time, dep_dist, dep_speed = veh.get_departure_schedule()
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
        :type model: CPLEX
        :return: coefficients of the polynomial for the ``veh`` object and trajectory points to the trajectory attribute of it

        :Author:
            Mahmoud Pourmehrab <pourmehrab@gmail.com>
        :Date:
            April-2018
        """
        det_time, det_dist, det_speed = veh.get_arrival_schedule()
        dep_time, dep_dist, dep_speed = veh.get_departure_schedule()
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
            t, d, s = self.optimize_lead_connected_trj(veh)
        else:
            t, d, s = self.optimize_follower_connected_trj(veh, lead_veh)  # is defined/used in the child class

        self.set_trajectory(veh, t, d, s)

    def compute_trj_points(self, f, f_prime, departure_time_relative):
        """
        Converts the polynomial trajectory to the trajectory points.

        :param f: the coefficients to define trajectory polynomial
        :param f_prime: the coeeficients to define the speed polynomial
        :param departure_time_relative: span of the trajectory
        :return: t, d, s

        :Author:
            Mahmoud Pourmehrab <pourmehrab@gmail.com>
        :Date:
            April-2018
        """
        t = self.discretize_time_interval(0, departure_time_relative)
        d = np.polyval(f, t)
        s = np.polyval(-f_prime, t)

        return t, d, s

    def optimize_lead_connected_trj(self, veh):
        """
        Computes a linear trajectory for the vehicle. This case should not happen except for the case the LP has no solution.

        :param veh: subject vehicle
        :type veh: Vehicle
        :return: trajectory of subject lead CAV

        :Author:
            Mahmoud Pourmehrab <pourmehrab@gmail.com>
        :Date:
            April-2018
        """
        det_time, det_dist, _ = veh.get_arrival_schedule()
        v = det_dist / (veh.scheduled_departure - det_time)

        t = self.discretize_time_interval(det_time, veh.scheduled_departure)
        s = np.array([v] * len(t))
        d = np.array([det_dist - v * (t_i - det_time) for t_i in t])

        return t, d, s


# -------------------------------------------------------
# FOLLOWER CONNECTED AND AUTOMATED TRAJECTORY OPTIMIZER
# -------------------------------------------------------


class FollowerConnected(LeadConnected):
    """
    Optimizes the trajectory of a follower CAV.

    Use Case:

        Instantiate like::

            >>> follower_connected_trj_optimizer = FollowerConnected(intersection)

        Perform trajectory computation by::

            >>> model = follower_connected_trj_optimizer.set_model(veh, lead_veh)
            >>> follower_connected_trj_optimizer.solve(veh, lead_veh)

    :param GAP_CTRL_STARTS: This is the relative time when gap control constraints get added
    :param SAFE_MIN_GAP: The minimum safe distance to keep from lead vehicles (in :math:`m`) [*can be speed dependent*]

    :Author:
        Mahmoud Pourmehrab <pourmehrab@gmail.com>
    :Date:
        April-2018
    """
    GAP_CTRL_STARTS = 2.0
    SAFE_MIN_GAP = 4.8

    def __init__(self, intersection):
        """
        Adds the safe headway constraints at the control points to the inherited model.

        :param max_speed:
        :param min_headway:
        :param k:
        :param m:

        :Author:
            Mahmoud Pourmehrab <pourmehrab@gmail.com>
        :Date:
            April-2018
        """
        super().__init__(intersection)

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
        :return: the CPLEX LP model to be solved by solve() method
        :Author:
            Mahmoud Pourmehrab <pourmehrab@gmail.com>
        :Date:
            April-2018
        """

        self._lp_model = super().set_model(veh)

        follower_det_time, _, _ = veh.get_arrival_schedule()
        follower_dep_time, _, _ = veh.get_departure_schedule()
        lead_dep_time, _, _ = lead_veh.get_departure_schedule()
        start_relative_ctrl_time, end_relative_ctrl_time = self.GAP_CTRL_STARTS, lead_dep_time - follower_det_time
        departure_time_relative = follower_dep_time - follower_det_time

        if end_relative_ctrl_time > start_relative_ctrl_time + self.m * self._small_positive_num:
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
        The only reason this class method exists is to access :any:`optimize_follower_connected_trj` method.

        :param veh: subject vehicle
        :type veh: Vehicle
        :param lead_veh: the vehicle in front of the subject
        :type lead_veh: Vehicle
        :param model: the follower CPLEX model

        :Author:
            Mahmoud Pourmehrab <pourmehrab@gmail.com>
        :Date:
            April-2018
        """
        super().solve(veh, lead_veh, model)

    def optimize_follower_connected_trj(self, veh, lead_veh):
        """
        Works based on the concept of hypothetical trajectory.

        :param veh: subject vehicle
        :type veh: Vehicle
        :param lead_veh: lead vehicle which could be `None` if no vehicle is in front.
        :type lead_veh: Vehicle
        :return: trajectory of the subject follower AV in case the LP has no solution.

        :Author:
            Mahmoud Pourmehrab <pourmehrab@gmail.com>
        :Date:
            April-2018
        """
        lead_dep_time, _, _ = lead_veh.get_departure_schedule()
        foll_det_time, foll_det_dist, _ = veh.get_arrival_schedule()
        foll_dep_time, foll_dep_dist, _ = veh.get_departure_schedule()
        dep_headway = foll_dep_time - lead_dep_time
        t, d, s = np.copy(lead_veh.trajectory[:, lead_veh.first_trj_point_indx:lead_veh.last_trj_point_indx + 1])
        t += dep_headway  # to shift the trajectory

        dt = t[0] - foll_det_time
        v = (foll_det_dist - d[0]) / dt
        t_augment = self.discretize_time_interval(0, dt)
        d_augment = [foll_det_dist - v * t_i for t_i in t_augment]
        s_augment = [v] * len(t_augment)
        t_augment += foll_det_time

        return map(np.concatenate, [[t_augment[:-1], t], [d_augment[:-1], d], [s_augment[:-1], s]])
