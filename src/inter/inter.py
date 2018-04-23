####################################
# File name: inter.py              #
# Author: Mahmoud Pourmehrab       #
# Email: mpourmehrab@ufl.edu       #
# Last Modified: Apr/22/2018       #
####################################

'''
    Intersection: gets parameters that are needed to specify the configuration of problem
'''

from src.input.data import *


class Intersection:
    """
    Goals:
        1) Keeps intersection parameters
    """

    def __init__(self, int_name):
        """
        :param int_name: comes from what user input in the command line as the intersection name
        """
        self.name = int_name
        self._max_speed, self._min_headway, self._det_range, self._K, self._M, self._num_lanes = get_general_params(
            int_name)

    def get_poly_params(self):
        """
        :return: K and M
        """
        return self._K, self._M

    def get_num_lanes(self):
        return self._num_lanes

    def get_max_speed(self):
        return self._max_speed

    def get_min_headway(self):
        return self._min_headway

    def get_det_range(self):
        return self._det_range
