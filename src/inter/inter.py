'''
By:     Mahmoud Pourmehrab
E-mail: mpourmehrab@ufl.edu
Date:        Nov 2017
Last update: Dec/08/2017
'''

import os

import numpy as np

from src.inpt.inpt import read_prms
from src.inter.phs import phenum


class MahmoudIntersection:
    def __init__(self, int_name):
        self.name = int_name
        self.nl = 0
        self.set_lli()

        self.move_share = MahmoudIntersection._read_mat(self, int_name, 'MS')
        self._set_phs()

        d = {"max_speed": '_v', "yellow_duration": '_y', "all_red_duration": '_ar', "opt_range": '_opt_range'}
        # max_speed : m/s, time : sec, distance : meters
        read_prms(self, int_name, 'int', d)

    def set_lli(self):
        self.lli = self._read_mat(self.name, 'CM')
        if self.lli.shape[0] != self.lli.shape[1]:
            raise Exception('Check CM.txt file for CM to be square matrix.')
        else:
            self._nl = self.lli.shape[1]

    def _read_mat(self, int_name, filename):
        filepath = os.path.join('data/' + int_name, filename + '.txt')
        if os.path.exists(filepath):
            return np.loadtxt(filepath, dtype='i', delimiter=',')
        else:
            raise Exception(filepath + ' was not found.')

    def _set_phs(self):
        filepath = os.path.join('data/' + self.name, 'PLI.txt')
        if os.path.exists(filepath):
            self._pli = np.loadtxt(filepath, dtype='i', delimiter=',')
        else:
            phenum(self._nl, self.lli, self.name)
        # print('PPI loaded.')

    def get_phs(self):
        return self._pli

    def get_num_lanes(self):
        return self._nl

    def get_max_speed(self):
        return self._v
