'''
By:     Mahmoud Pourmehrab
E-mail: mpourmehrab@ufl.edu
Last update: November/2017
'''

import numpy as np
from src.inpt import readprms


class Intersection:
    def __init__(self, intName):
        self.CM = Intersection.readMat(self, intName, 'CM')
        if self.CM.shape[0] != self.CM.shape[1]:
            print('Check CM file for CM to be square matrix.\n')
            quit()
        else:
            self.NoLanes = self.CM.shape[1]

        self.LS = Intersection.readMat(self, intName, 'LS')

        d = {"v": "v", "y": "y", "ar": "ar", "optRange": "optRange"}
        # speed : m/s, time : sec, distance : meters
        readprms(self, intName, 'int', d)

    def readMat(self, intName, filename):
        filepath = "./data/" + intName + "/" + filename + ".txt"
        input = np.loadtxt(filepath, dtype='i', delimiter=',')
        return input

    def readPhs(self):
        # TODO: don't hard-code this
        self.ph = (
            {'lanes': (0, 1, 2, 3), 'minG': 5, 'maxG': 25},
            {'lanes': (4, 5, 6), 'minG': 5, 'maxG': 25},
            {'lanes': (7, 8, 9, 10), 'minG': 5, 'maxG': 25},
            {'lanes': (11, 12, 13, 14, 15), 'minG': 5, 'maxG': 25}
        )
