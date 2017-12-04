#!/usr/bin/python3
'''
By:     Mahmoud Pourmehrab
E-mail: mpourmehrab@ufl.edu
Date:        Nov 2017
Last update: Dec/03/2017
'''

import time
from cmp.phs import phenum
# import datetime
import sys
from src.inter import Intersection
from src.sim import Simulator

if __name__ == "__main__":
    print('Python Path: ', sys.executable)
    print('Python Version: ', sys.version)

    t1 = time.clock()
    InterName = '13th16th'

    IntObj = Intersection(InterName)
    simObj = Simulator(InterName)

    phenum(IntObj.NoLanes,IntObj.CM)

    t2 = time.clock()
    print(' Elapsed Time: {} ms'.format(int(1000 * (t2 - t1))), end='')
