####################################
# File name: traj.py               #
# Author: Mahmoud Pourmehrab       #
# Email: mpourmehrab@ufl.edu       #
# Last Modified: Apr/01/2018       #
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

    def set_trajectory(self, trajectory, last_trj_point_indx, t, d, s):
        '''
        Sets trajectory of the vehicle. They're computed elsewhere. This is just to set them.
        '''
        n = len(t)
        trajectory[0:n, :] = np.transpose([t, d, s])
        last_trj_point_indx = n - 1


# -------------------------------------------------------
# LEAD CONVENTIONAL TRAJECTORY ESTIMATOR
# -------------------------------------------------------
class LeadConventional(Trajectory):

    def __init__(self, max_speed, min_headway):
        super().__init__(max_speed, min_headway)

    def solve(self, trajectory, last_trj_point_indx, green_start_time, yellow_end_time):
        '''
        Constructs the trajectory of the lead conventional vehicle assuming they maintain their speed
        '''

        det_time, det_dist, det_speed = trajectory[0]

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

        self.set_trajectory(trajectory, last_trj_point_indx, t, d, s)


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
        Constructs the trajectory of the lead conventional vehicle assuming they maintain their speed
        '''

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


# -------------------------------------------------------
# LEAD CONNECTED AND AUTOMATED TRAJECTORY OPTIMIZER
# -------------------------------------------------------

class LeadConnected(Trajectory):
    K = 10  # n will be in 0, ..., k-1
    M = 4  # to discretize the time interval

    def __init__(self, max_speed, min_headway):
        super().__init__(max_speed, min_headway)

        self._lp_model = cplex.Cplex()
        # We are looking to minimize the are under the trajectory curve
        self._lp_model.objective.set_sense(self._lp_model.objective.sense.minimize)

        self._lp_model.variables.add(obj=[1.0 for n in range(self.K)],
                                     names=["beta_" + str(n) for n in range(self.K)],
                                     lb=[-cplex.infinity for n in range(self.K)])

        self._lp_model.linear_constraints.add(lin_expr=[[["beta_0"], [1]], [["beta_1"], [1]]],
                                              senses=['E', 'E'],
                                              rhs=[0, 0],
                                              names=['match_det_dist', 'match_speed'])

        constraint = [["beta_" + str(n) for n in range(self.K)], [1 for n in range(self.K)]]
        self._lp_model.linear_constraints.add(
            lin_expr=[constraint],
            senses=['E'],
            rhs=[1],
            names=['match_dep_dist'])

        self._lp_model.linear_constraints.add(
            lin_expr=[constraint for j in range(1, self.M)] +
                     [constraint for j in range(1, self.M)] +
                     [constraint for j in range(1, self.M)] +
                     [constraint for j in range(1, self.M)],
            senses=['G' for j in range(1, self.M)] +
                   ['L' for j in range(1, self.M)] +
                   ['G' for j in range(1, self.M)] +
                   ['L' for j in range(1, self.M)],
            rhs=[-max_speed for j in range(1, self.M)] +
                [0 for j in range(1, self.M)] +
                [1 for j in range(1, self.M)] +
                [1 for j in range(1, self.M)],
            names=['ub_speed_' + str(j) for j in range(1, self.M)] +
                  ['lb_speed_' + str(j) for j in range(1, self.M)] +
                  ['ub_acc_' + str(j) for j in range(1, self.M)] +
                  ['lb_acc_' + str(j) for j in range(1, self.M)])

    def solve(self, trajectory, last_trj_point_indx,
              arrival_time, arrival_dist,
              amin, amax,
              green_start_time, yellow_end_time):
        det_time, det_dist, det_speed = trajectory[0]

        self._lp_model.objective.set_linear(zip(
            ["beta_" + str(n) for n in range(self.K)],
            [arrival_time / (1 + n) for n in range(self.K)]))

        self._lp_model.linear_constraints.set_rhs([('match_det_dist', det_dist),
                                                   ('match_speed', -det_speed),
                                                   ('match_dep_dist', arrival_dist), ] +
                                                  list(zip(['ub_acc_' + str(j) for j in range(1, self.M)],
                                                           [-amax for j in range(1, self.M)])) +
                                                  list(zip(['lb_acc_' + str(j) for j in range(1, self.M)],
                                                           [-amin for j in range(1, self.M)])))

        for j in range(1, self.M):
            var_name = ["beta_" + str(n) for n in range(self.K)]
            speed_coeff = [0] + [n * (j / (self.M + 1)) ** (n - 1) / arrival_time for n in range(1, self.K)]

            self._lp_model.linear_constraints.set_coefficients(zip(
                ['ub_speed_' + str(j) for n in range(self.K)], var_name, speed_coeff))

            self._lp_model.linear_constraints.set_coefficients(zip(
                ['lb_speed_' + str(j) for n in range(self.K)], var_name, speed_coeff))

            factor = (self.M + 1) / (arrival_time * j)
            acc_coeff = [0, 0] + [speed_coeff[n] * factor * (n - 1) for n in range(2, self.K)]

            self._lp_model.linear_constraints.set_coefficients(zip(
                ['ub_acc_' + str(j) for n in range(self.K)], var_name, acc_coeff))

            self._lp_model.linear_constraints.set_coefficients(zip(
                ['lb_acc_' + str(j) for n in range(self.K)], var_name, acc_coeff))

        self._lp_model.write("lead_CAV.lp")

        self._lp_model.solve()
        pass

        # -------------------------------------------------------
        # FOLLOWER CONNECTED AND AUTOMATED TRAJECTORY OPTIMIZER
        # -------------------------------------------------------

class FollowerConnected(LeadConnected):
    def __init__(self, max_speed, min_headway):
        super().__init__(max_speed, min_headway)

        constraint = [["beta_" + str(n) for n in range(self.K)], [1 for n in range(self.K)]]
        self._lp_model.linear_constraints.add(
            lin_expr=[constraint for j in range(1, self.M)],
            senses=['G' for j in range(1, self.M)],
            rhs=[min_headway for j in range(1, self.M)],
            names=['min_headway_' + str(j) for j in range(1, self.M)])

        self._lp_model.write("follower_CAV.lp")

    def solve(self, follower_trajectory, follower_last_trj_point_indx,
              lead_trajectory, lead_last_trj_point_indx,
              green_start_time, yellow_end_time):
        pass
