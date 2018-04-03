####################################
# File name: inter.py              #
# Author: Mahmoud Pourmehrab       #
# Email: mpourmehrab@ufl.edu       #
# Last Modified: Mar/29/2018       #
####################################

'''
    Intersection: gets parameters that are needed to specify the configuration of problem
    TODO: move the data from the python file into a text file
'''

from src.input.data import *


class Intersection:
    def __init__(self, int_name):
        self.name = int_name

        self._max_speed, self._min_headway, self._K, self._M = get_general_params(int_name)

        conf_dict = get_conflict_dict(self.name)
        # number of lanes
        self._num_lanes = len(conf_dict)

    def get_poly_params(self):
        return self._K, self._M

    def get_num_lanes(self):
        return self._num_lanes

    def get_max_speed(self):
        return self._max_speed

    def get_min_headway(self):
        return self._min_headway
