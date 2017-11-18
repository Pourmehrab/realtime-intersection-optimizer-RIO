#!/usr/bin/python3
'''
By:     Mahmoud Pourmehrab
E-mail: mpourmehrab@ufl.edu
Last update: November/2017
'''

import time
import numpy as np
# import datetime
import sys
from src.inter import Intersection
from src.sim import Simulator
from src.trjsol import LVTOsol

if __name__ == "__main__":
    print('Python Path: ', sys.executable)
    print('Python Version: ', sys.version)

    t1 = time.clock()
    InterName = '13th16th'

    IntObj = Intersection(InterName)
    simObj = Simulator(InterName)

    LVTOsol(300, 30, 50, -1, 3, 0, 100, simObj)

    t2 = time.clock()
    print(' Elapsed Time: {} ms'.format(int(1000 * (t2 - t1))), end='')
