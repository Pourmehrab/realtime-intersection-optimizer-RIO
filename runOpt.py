#!/usr/bin/python3
'''
By:     Mahmoud Pourmehrab
E-mail: mpourmehrab@ufl.edu
Date:        Nov 2017
Last update: Dec/03/2017
'''

import time
# import datetime
import sys
from src.inter import Intersection
from src.sim import Simulator

if __name__ == "__main__":
    print('Python Path: ', sys.executable)
    print('Python Version: ', sys.version)

    inter_name = '13th16th'

    IntObj = Intersection(inter_name)
    simObj = Simulator(inter_name)

    t1 = time.clock()


    t2 = time.clock()
    print(' Elapsed Time: {} ms'.format(int(1000 * (t2 - t1))), end='')
