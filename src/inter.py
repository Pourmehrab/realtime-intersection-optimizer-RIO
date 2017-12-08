'''
By:     Mahmoud Pourmehrab
E-mail: mpourmehrab@ufl.edu
Date:        Nov 2017
Last update: Dec/08/2017
'''

import numpy as np
import os
from src.inpt import read_prms
from cmp.phs import phenum


class Intersection:
    def __init__(self, int_name):
        self.name = int_name
        self.nl = 0
        self.set_lli()

        self.move_share = Intersection._read_mat(self, int_name, 'MS')
        self._set_phs()

        d = {"v": "v", "y": "y", "ar": "ar", "optRange": "optRange"}
        # speed : m/s, time : sec, distance : meters
        read_prms(self, int_name, 'int', d)

    def set_lli(self):
        self.lli = self._read_mat(self.name, 'CM')
        if self.lli.shape[0] != self.lli.shape[1]:
            raise Exception('Check CM.txt file for CM to be square matrix.')
        else:
            self.nl = self.lli.shape[1]

    def _read_mat(self, int_name, filename):
        filepath = os.path.join('data/' + int_name, filename + '.txt')
        if os.path.exists(filepath):
            return np.loadtxt(filepath, dtype='i', delimiter=',')
        else:
            raise Exception(filepath + ' was not found.')

    def _set_phs(self):
        filepath = os.path.join('data/' + self.name, 'PPI.txt')
        if os.path.exists(filepath):
            self.ppi = np.loadtxt(filepath, dtype='i', delimiter=',')
        else:
            phenum(self.nl, self.lli, self.name)
