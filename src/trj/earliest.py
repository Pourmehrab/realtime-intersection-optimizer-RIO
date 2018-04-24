####################################
# File name: earliest.py           #
# Author: Mahmoud Pourmehrab       #
# Email: mpourmehrab@ufl.edu       #
# Last Modified: Apr/22/2018       #
####################################


import numpy as np


def earliest_arrival_connected(det_time, speed, dist, amin, amax, max_speed, min_headway=0, t_earliest=0):
    """
    Uses the maximum of the followings to compute the earliest time vehicle can reach to the stop bar:
        1) Accelerate/Decelerate to the maximum allowable speed and maintain the speed till departure
        2) Distance is short, it accelerates/decelerated to the best speed and departs
        3) Departs at the minimum headway with its lead vehicle (only for followers close enough to their lead)

    :param det_time:
    :param speed:
    :param dist:
    :param amin:
    :param amax:
    :param max_speed:
    :param min_headway:
    :param t_earliest: earliest time of lead vehicle that is only needed if the vehicle is a follower vehicle
    :return:
    """
    a = amax if speed <= max_speed else amin
    dist_to_max_speed = (max_speed ** 2 - speed ** 2) / (2 * a)

    if dist_to_max_speed <= dist:
        return max(
            det_time + (max_speed - speed) / a + (dist - dist_to_max_speed) / max_speed  # min time to get to stop bar
            , t_earliest + min_headway)

    else:  # not enough time and distance to accelerate/decelerate to max speed
        v_dest = np.sqrt(speed ** 2 + 2 * a * dist)
        return max(
            det_time + (max_speed - v_dest) / a  # min time to get to stop bar
            , t_earliest + min_headway
        )


def earliest_arrival_conventional(det_time, speed, dist, min_headway=0, t_earliest=0):
    """
    Uses the maximum of the followings to compute the earliest time vehicle can reach to the stop bar:
        1) Maintain the detected speed till departure
        2) Depart at the minimum headway with the vehicle in front

    :param det_time:
    :param speed:
    :param dist:
    :param min_headway:
    :param t_earliest: earliest time of lead vehicle that is only needed if the vehicle is a follower vehicle
    :return:
    """
    return max(
        det_time + dist / speed
        , t_earliest + min_headway
    )
