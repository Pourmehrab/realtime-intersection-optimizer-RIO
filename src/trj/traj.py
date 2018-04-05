####################################
# File name: traj.py               #
# Author: Mahmoud Pourmehrab       #
# Email: mpourmehrab@ufl.edu       #
# Last Modified: Apr/03/2018       #
####################################

import numpy as np


# -------------------------------------------------------
# TRAJECTORY SUPER CLASS
# -------------------------------------------------------

class Trajectory:
    '''
    Abstract class for trajectories

    four subclasses (LeadConnected, FollowerConnected, LeadConventional, and FollowerConventional) are
    inherited from this class
    '''

    LAG = 1  # lag time from start of green when a vehicle can depart
    RES = 1  # second (be careful not to exceed max size of trajectory
    EPS = 0.01  # small number that lower than that is approximated by zero

    def __init__(self, max_speed, min_headway):
        self._max_speed = max_speed
        self._min_headway = min_headway

    def vectorize_time_interval(self, start_time, end_time):
        '''
        vectorize by RES
        '''
        if end_time - start_time % self.RES > self.EPS:
            trj_time_stamps = np.append(np.arange(start_time, end_time, Trajectory.RES, dtype=float), end_time)
        else:
            trj_time_stamps = np.arange(start_time, end_time, Trajectory.RES, dtype=float)

        return trj_time_stamps

    def set_trajectory(self, veh, t, d, s):
        '''
        Sets trajectory of the vehicle. They're computed elsewhere. This is just to set them.
        '''
        n = len(t)
        veh.trajectory[:, 0:n] = [t, d, s]
        veh.set_last_trj_point_indx(n - 1)


# -------------------------------------------------------
# LEAD CONVENTIONAL TRAJECTORY ESTIMATOR
# -------------------------------------------------------
class LeadConventional(Trajectory):

    def __init__(self, max_speed, min_headway):
        super().__init__(max_speed, min_headway)

    def solve(self, veh, green_start_time, yellow_end_time):
        '''
        Constructs the trajectory of the lead conventional vehicle assuming they maintain their speed
        '''
        trajectory = veh.trajectory
        det_time, det_dist, det_speed = trajectory[:, 0]

        arrival_time = det_time + det_dist / det_speed

        if arrival_time < green_start_time:

            # assume vehicle slows down
            arrival_time = green_start_time  # todo check if it makes copies
            det_speed = det_dist / arrival_time

        elif arrival_time > yellow_end_time:
            print('The lead conventional vehicle will be departing after yellow ends!')

        t = self.vectorize_time_interval(det_time, arrival_time)
        s = np.array([det_speed for i in range(len(t))])
        d = np.array([det_dist - det_speed *
                      (t[i] - det_time) for i in range(len(t))])

        self.set_trajectory(veh, t, d, s)


# -------------------------------------------------------
# FOLLOWER CONVENTIONAL TRAJECTORY ESTIMATOR
# -------------------------------------------------------
class FollowerConventional(Trajectory):
    LARGE_SPEED = 99999  # when under sqrt goes negative we set this to rule out the speed in min()

    def __init__(self, max_speed, min_headway):
        super().__init__(max_speed, min_headway)

    def solve(self, follower_trajectory, follower_last_trj_point_indx,
              lead_trajectory, lead_last_trj_point_indx,
              green_start_time, yellow_end_time,
              follower_desired_speed, follower_max_acc, follower_max_dec, lead_max_dec,
              lead_length):
        '''
        Gipps car following model

        Refer to:
            Gipps, Peter G. "A behavioural car-following model for computer simulation."
            Transportation Research Part B: Methodological 15.2 (1981): 105-111.

        :return: speed profile of follower CNV
        '''

        # follower vehicle is set to lead for computational reason
        t, d, s = np.copy(lead_trajectory[:lead_last_trj_point_indx])

        t[0], d[0], s[0] = follower_trajectory[0]

        trj_indx = 0  # this goes over follower trajectory points as it builds up

        while trj_indx < lead_last_trj_point_indx:  # Gipps Car Following Implementation
            lead_speed = lead_trajectory[trj_indx, 2]

            gap = d[trj_indx] - lead_trajectory[trj_indx, 1]
            dt = t[trj_indx + 1] - t[trj_indx]

            s1 = 1 / 40 + s[trj_indx] / follower_desired_speed
            s2 = (follower_max_dec * (lead_max_dec * (2 * (lead_length - gap) + dt * (
                    follower_max_dec * dt + s[trj_indx])) + lead_speed ** 2)) / lead_max_dec

            v1 = s[trj_indx] + (5 / 2) * follower_max_acc * dt * (1 - s[trj_indx] / follower_desired_speed) * np.sqrt(
                s1) if s1 >= 0 else self.LARGE_SPEED
            v2 = follower_max_dec * dt + np.sqrt(s2) if s2 >= 0 else self.LARGE_SPEED

            s[trj_indx + 1] = min(v1, v2)
            a = (s[trj_indx + 1] - s[trj_indx]) / dt
            d[trj_indx + 1] = d[trj_indx] - (
                    a * (t[trj_indx + 1] ** 2 - t[trj_indx] ** 2) / 2 + (s[trj_indx + 1] - a * t[trj_indx]) * dt)

            trj_indx += 1

        # This part adds the end part that is out of Gipps CF domain
        arrival_time = d[-1] / follower_desired_speed

        t_augment = self.vectorize_time_interval(t[-1], t[-1] + arrival_time)
        d_augment = [d[-1] - (t_augment[i] - t[-1]) * follower_desired_speed for i in range(len(t_augment))]
        s_augment = [follower_desired_speed for i in range(len(t_augment))]

        np.append(t, t_augment)
        np.append(d, d_augment)
        np.append(s, s_augment)

        self.set_trajectory(follower_trajectory, follower_last_trj_point_indx, t, d, s)


import cplex


# Refer to: IBM(R) ILOG CPLEX Python API Reference Manual

# -------------------------------------------------------
# LEAD CONNECTED AND AUTOMATED TRAJECTORY OPTIMIZER
# -------------------------------------------------------

class LeadConnected(Trajectory):

    def __init__(self, max_speed, min_headway, k, m):
        super().__init__(max_speed, min_headway)

        self.k, self.m = k, m

        self._lp_model = cplex.Cplex()
        self._lp_model.set_log_stream(None)
        self._lp_model.set_error_stream(None)
        self._lp_model.set_warning_stream(None)
        self._lp_model.set_results_stream(None)
        # We are looking to minimize the are under the trajectory curve
        self._lp_model.objective.set_sense(self._lp_model.objective.sense.minimize)

        var_name = ["beta_" + str(n) for n in range(self.k)]
        self._lp_model.variables.add(obj=[1.0 for n in range(self.k)],
                                     names=var_name,
                                     lb=[-cplex.infinity for n in range(self.k)])

        self._lp_model.linear_constraints.add(lin_expr=[[["beta_0"], [1]], [["beta_1"], [1]]],
                                              senses=['E', 'E'],
                                              rhs=[0, 0],
                                              names=['match_det_dist', 'match_speed'])

        constraint = [var_name, [0 for n in range(self.k)]]
        self._lp_model.linear_constraints.add(
            lin_expr=[constraint, constraint],
            senses=['E', 'E'],
            rhs=[1, 1],
            names=['match_dep_dist', 'match_dep_speed'])

        self._lp_model.linear_constraints.add(
            lin_expr=[constraint for j in range(self.m)] +
                     [constraint for j in range(self.m)] +
                     [constraint for j in range(self.m)] +
                     [constraint for j in range(self.m)],
            senses=['G' for j in range(self.m)] +
                   ['L' for j in range(self.m)] +
                   ['G' for j in range(self.m)] +
                   ['L' for j in range(self.m)],
            rhs=[-max_speed for j in range(self.m)] +
                [0 for j in range(self.m)] +
                [1 for j in range(self.m)] +
                [1 for j in range(self.m)],
            names=['ub_speed_' + str(j) for j in range(self.m)] +
                  ['lb_speed_' + str(j) for j in range(self.m)] +
                  ['ub_acc_' + str(j) for j in range(self.m)] +
                  ['lb_acc_' + str(j) for j in range(self.m)])

    def set_model(self, veh, arrival_time, arrival_dist, dep_speed,
                  green_start_time, yellow_end_time):
        '''
        f(t)   = sum_0^{k-1} b_n t^n
        f'(t)  = sum_1^{k-1} n b_n t^{n-1}
        f''(t) = sum_2^{k-1} n (n-1) b_n t^{n-2}

        '''

        trajectory = veh.trajectory
        amin, amax = veh.max_decel_rate, veh.max_accel_rate

        det_time, det_dist, det_speed = trajectory[:, 0]
        arrival_time_relative = arrival_time - det_time

        self._lp_model.objective.set_linear(zip(
            ["beta_" + str(n) for n in range(self.k)],
            [arrival_time_relative ** (1 + n) / (1 + n) for n in range(self.k)]))

        self._lp_model.linear_constraints.set_rhs([('match_det_dist', det_dist),
                                                   ('match_speed', -det_speed),
                                                   ('match_dep_dist', arrival_dist),
                                                   ('match_dep_speed', -dep_speed)] +
                                                  list(zip(['ub_acc_' + str(j) for j in range(self.m)],
                                                           [-amax for j in range(self.m)])) +
                                                  list(zip(['lb_acc_' + str(j) for j in range(self.m)],
                                                           [-amin for j in range(self.m)])))

        dist_coeff = np.array([arrival_time_relative ** n for n in range(self.k)], dtype=float)
        var_name = ["beta_" + str(n) for n in range(self.k)]
        self._lp_model.linear_constraints.set_coefficients(zip(
            ['match_dep_dist' for n in range(self.k)], var_name, dist_coeff))

        control_points = np.linspace(arrival_time_relative / self.m, arrival_time_relative,
                                     self.m, endpoint=False)

        j = 0
        for time in control_points:
            speed_coeff = np.array([n * time ** (n - 1) for n in range(self.k)])
            acc_coeff = np.array([speed_coeff[n] * (n - 1) / time for n in range(self.k)])

            self._lp_model.linear_constraints.set_coefficients(
                list(zip(['ub_speed_' + str(j) for n in range(self.k)], var_name, speed_coeff)) +
                list(zip(['lb_speed_' + str(j) for n in range(self.k)], var_name, speed_coeff)) +
                list(zip(['ub_acc_' + str(j) for n in range(self.k)], var_name, acc_coeff)) +
                list(zip(['lb_acc_' + str(j) for n in range(self.k)], var_name, acc_coeff))
            )

            j += 1
        self._lp_model.linear_constraints.set_coefficients(
            list(zip(['match_dep_speed' for n in range(self.k)], var_name, speed_coeff)))

        return self._lp_model

    def solve(self, veh, model, arrival_time):
        trajectory = veh.trajectory

        model.write("model.lp")

        model.solve()

        try:
            assert 1 == model.solution.get_status()
        except:
            print('LP solution is not optimal. Check the inputs.')

        beta = np.flip(np.array(model.solution.get_values(["beta_" + str(n) for n in range(self.k)])), 0)
        f = np.poly1d(beta)
        f_prime = np.polyder(f)
        # set the polynomial
        veh.set_poly_coeffs(f)

        det_time = trajectory[0, 0]

        t, d, s = self.compute_trj_points(f, f_prime, arrival_time - det_time)
        t += det_time

        self.set_trajectory(veh, t, d, s)

    def compute_trj_points(self, f, f_prime, arrival_time_relative):
        t = self.vectorize_time_interval(0, arrival_time_relative)
        d = np.polyval(f, t)
        s = np.polyval(-f_prime, t)

        return t, d, s


# -------------------------------------------------------
# FOLLOWER CONNECTED AND AUTOMATED TRAJECTORY OPTIMIZER
# -------------------------------------------------------


class FollowerConnected(LeadConnected):
    HEASWAY_CONTROL_START = 2  # in seconds how frequent need to check for speed, acc/dec rate, and headway
    SAFE_MIN_GAP = 4.8  # minimum safe deistance to keep from lead vehicles todo make it dependent to speed

    def __init__(self, max_speed, min_headway, k, m):
        super().__init__(max_speed, min_headway, k, m)

        constraint = [["beta_" + str(n) for n in range(self.k)], [1 for n in range(self.k)]]
        self._lp_model.linear_constraints.add(
            lin_expr=[constraint for j in range(self.m)],
            senses=['G' for j in range(self.m)],
            rhs=[min_headway for j in range(self.m)],
            names=['min_headway_' + str(j) for j in range(self.m)])

        # self._lp_model.write("model.lp")

    def set_model(self, veh, arrival_time, arrival_dist, dep_speed,
                  green_start_time, yellow_end_time,
                  lead_poly, lead_det_time, lead_arrival_time):
        self._lp_model = super().set_model(veh, arrival_time, arrival_dist, dep_speed,
                                           green_start_time, yellow_end_time)

        trajectory = veh.trajectory

        det_time, det_dist, det_speed = trajectory[:, 0]

        start_relative_ctrl_time, end_relative_ctrl_time = self.HEASWAY_CONTROL_START, lead_arrival_time - det_time
        #  end_relative_ctrl_time is the time lead vehicle leaves relative to the time the follower vehicle was detected
        if end_relative_ctrl_time > start_relative_ctrl_time:
            # trajectories dont overlap over time. No need for min headway constraints
            control_points = np.linspace(start_relative_ctrl_time, end_relative_ctrl_time,
                                         self.m, endpoint=True)
            det_time_diff = det_time - lead_det_time
            min_dist_vec = np.array([np.polyval(lead_poly, control_points[j] + det_time_diff) for j in range(self.m)]) \
                           + self.SAFE_MIN_GAP
            self._lp_model.linear_constraints.set_rhs(zip(['min_headway_' + str(j) for j in range(self.m)],
                                                          min_dist_vec))

            var_name = ["beta_" + str(n) for n in range(self.k)]
            j = 0
            for time in control_points:
                dist_coeff = np.array([time ** n for n in range(self.k)], dtype=float)
                self._lp_model.linear_constraints.set_coefficients(zip(
                    ['min_headway_' + str(j) for n in range(self.k)], var_name, dist_coeff))
                j += 1

        return self._lp_model
