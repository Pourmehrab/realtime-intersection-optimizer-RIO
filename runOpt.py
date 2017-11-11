#!/usr/bin/python3
''' By: Mahmoud Pourmehrab
E-mail: mpourmehrab@ufl.edu'''

import time
from src.inter import Intersection
from src.sim import Simulator


if __name__ == "__main__":

    t1 = time.clock()
    InterName = '13th16th'

    IntObj = Intersection(InterName)
    simObj = Simulator(InterName)

    t2 = time.clock()
    print(simObj,
          ' Elapsed Time: {} ms'.format(int(1000*(t2-t1))), end='')
