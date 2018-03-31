####################################
# File name: earliest.py           #
# Author: Mahmoud Pourmehrab       #
# Email: mpourmehrab@ufl.edu       #
# Last Modified: Mar/31/2018       #
####################################

'''
Estimated the earliest arrival time for CAV and conventional vehicles

Assumptions:
    CAVs, if not constrained by the vehicle ahead, see green and therefore can accelerate to the maximum
    reachable speed.
    Conventional vehicles tend to maintain their arrival speed
'''

import numpy as np


def earliest_arrival_connected(det_time, speed, dist, amin, amax, max_speed, min_headway=0, t_earliest=0):
    a = amax if speed <= max_speed else amin
    dist_to_max_speed = (max_speed ** 2 - speed ** 2) / (2 * a)

    if dist_to_max_speed <= dist:
        return max(
            (max_speed - speed) / a + (dist - dist_to_max_speed) / max_speed  # min time to get to stop bar
            , t_earliest + min_headway
        )

    else:  # not enough time and distance to accelerate/decelerate to max speed
        v_dest = np.sqrt(speed ** 2 + 2 * a * dist)
        return max(
            (max_speed - v_dest) / a  # min time to get to stop bar
            , t_earliest + min_headway
        )


def earliest_arrival_conventional(det_time, speed, dist, min_headway=0, t_earliest=0):
    return max(
        det_time + dist / speed
        , t_earliest + min_headway
    )
