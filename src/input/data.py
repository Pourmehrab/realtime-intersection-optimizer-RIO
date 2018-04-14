####################################
# File name: data.py               #
# Author: Mahmoud Pourmehrab       #
# Email: mpourmehrab@ufl.edu       #
# Last Modified: Feb/16/2018       #
####################################

'''
This python file stores data of the following intersection:
numbers are one-based (get converted to zero-based later)

    13th16th: a physical one google map it in Gainesville for the image

    reserv: reservation based model intersection: 12 incoming lanes (3 per approach and all lanes are exclusive)
    assume three discharge lanes (http://www.cs.utexas.edu/~aim/)

'''


def get_conflict_dict(inter_name):
    '''
    conf_dict has members that key is a lane : values are set of lanes that are in conflict with key lane (note the
    numbering starts from 1 not 0)

    :param inter_name: a string that gives the name of the intersection {13th16th, reserv}
    '''

    if inter_name == '13th16th':
        return {1: {7},
                2: {7, 8, 12, 16, 15, 14, 13},
                3: {7, 8, 12, 16, 15, 14, 9},
                4: {7, 16, 8, 15, 9, 11, 10},
                5: {16, 7, 15, 8, 11, 9, 10, 14},
                6: {10},
                7: {10, 15, 11, 16, 5, 4, 3, 2, 1},
                8: {10, 11, 15, 5, 4, 16, 3, 2, 12},
                9: {5, 10, 4, 11, 14, 12, 13, 3},
                10: {13, 14, 4, 5, 9, 8, 7, 6, 15},
                11: {13, 14, 9, 4, 5, 8, 7, 15, 16},
                12: {13, 14, 9, 3, 15, 16, 2, 8},
                13: {2, 3, 9, 12, 11, 10, 4},
                14: {2, 3, 12, 9, 11, 4, 10, 5},
                15: {2, 3, 12, 4, 5, 8, 7, 11, 6},
                16: {12, 2, 3, 8, 4, 7, 5, 11}}
        # (17, 9, 8, 15,) covers all lanes

    elif inter_name == 'reserv':
        return {1: {4, 5, 6, 7, 8, 9, 10, 11, 12},
                2: {4, 5, 6, 7, 8, 9, 10, 11, 12},
                3: {4, 5, 6, 7, 8, 9, 10, 11, 12},
                4: {1, 2, 3, 7, 8, 9, 10, 11, 12},
                5: {1, 2, 3, 7, 8, 9, 10, 11, 12},
                6: {1, 2, 3, 7, 8, 9, 10, 11, 12},
                7: {1, 2, 3, 4, 5, 6, 10, 11, 12},
                8: {1, 2, 3, 4, 5, 6, 10, 11, 12},
                9: {1, 2, 3, 4, 5, 6, 10, 11, 12},
                10: {1, 2, 3, 4, 5, 6, 7, 8, 9},
                11: {1, 2, 3, 4, 5, 6, 7, 8, 9},
                12: {1, 2, 3, 4, 5, 6, 7, 8, 9}}


def get_phases(inter_name):
    '''
    this is one based but the allowed set is zero based
    '''
    if inter_name == '13th16th':
        return {1: {1, 10, 16, },
                2: {6, 7, 12, },
                3: {1, 2, 6, 9, },
                4: {1, 5, 6, 13, },
                5: {1, 6, 8, 9, },
                6: {1, 6, 9, 16, },
                7: {1, 6, 11, 12, },
                8: {1, 9, 15, 16, },
                9: {1, 10, 11, 12, },
                10: {6, 7, 8, 9, },
                11: {1, 2, 3, 6, 11, },
                12: {1, 2, 3, 10, 11, },
                13: {1, 4, 5, 6, 12, },
                14: {1, 6, 8, 13, 14, },
                15: {1, 6, 13, 14, 16, },
                16: {1, 13, 14, 15, 16, },
                17: {6, 7, 8, 13, 14, },
                18: {1, 2, 3, 4, 5, 6, }}

    elif inter_name == 'reserv':
        # return None
        return {1: {1, 2, 3, },
                2: {4, 5, 6, },
                3: {7, 8, 9, },
                4: {10, 11, 12, }}
    else:
        raise Exception('Set of phases is not known for this intersection.')


def get_pretimed_parameters(inter_name):
    '''
    this return the parameters needed for pre-timed logic
    note the sequence field is zero-based
    required for pretimed control
    '''
    if inter_name == '13th16th':
        return None  # todo compute these

    elif inter_name == 'reserv':
        # return None
        return {'green_dur': (25.0, 25.0, 25.0, 25.0), 'phase_seq': (0, 1, 2, 3,), 'yellow': 1.5, 'all-red': 1.0}
    else:
        raise Exception('Pretimed parameters is not known for this intersection.')


def get_signal_params(inter_name):
    '''
    yellow, all-red, min green, max green all in seconds
    required for GA
    '''
    if inter_name == '13th16th':
        return 1.5, 1.0, 5.0, 25.0
    elif inter_name == 'reserv':
        return 1.5, 1.0, 5.0, 25.0
    else:
        raise Exception('Signal parameters are not known for this intersection.')


def get_general_params(inter_name):
    '''
    :return: max speed (m/s), min_headway (seconds), detection range (meters), k, m
        k =  # n will be in 0, ..., k-1
        m =  # to discretize the time interval
        required for trajectory optimization
    '''
    if inter_name == '13th16th':
        return 15.0, 2.0, 500.0, 10, 15
    elif inter_name == 'reserv':
        return 15.0, 2.0, 500.0, 10, 15
    else:
        raise Exception('Simulation parameters are not known for this intersection.')
