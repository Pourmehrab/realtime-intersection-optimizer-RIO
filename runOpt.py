#!/usr/bin/python3
'''
Code is written in Python 3
Install the list of packages in the Pipfile using PyEnv


By:     Mahmoud Pourmehrab
E-mail: mpourmehrab@ufl.edu
Date:        Nov 2017
Last update: Dec/08/2017
'''

import time
# import datetime
import sys
from cmp.inter import Intersection
from cmp.sim import Simulator
from cmp.veh import Lanes, Vehicle
from src.trjopt import MahmoudAVO

if __name__ == "__main__":
    print('Python Path: ', sys.executable)
    print('Python Version: ', sys.version)

    # Intersection name
    inter_name = '13th16th'

    # Initialization
    intersection = Intersection(inter_name)
    sim_prms = Simulator(inter_name)

    num_lanes = intersection.get_num_lanes()
    lanes = Lanes(num_lanes)

    t1 = time.clock()

    # add vehicles to the lane
    max_speed = intersection.get_max_speed()  # in m/s

    # work with vehicle data structure
    lanes.vehlist[0].add_last(Vehicle('xyz', 2, 27.7, 15, 320, max_speed))
    lanes.vehlist[0].add_last(Vehicle('abc', 2, 26.3, 17, 300, max_speed))
    lanes.vehlist[0].add_last(Vehicle('xyz', 0, 27.7, 15, 320, max_speed))

    print('{}'.format(len(lanes.vehlist[0])))
    p = lanes.vehlist[0].first()
    # lanes.vehlist[0].delete(p)

    # do some trj optimization
    lane = 0
    fol_veh = lanes.vehlist[lane].first()
    lead_veh = lanes.vehlist[lane].before(fol_veh)
    if lead_veh is None:
        x = MahmoudAVO(None, fol_veh.element(),100)
    else:
        x = MahmoudAVO(lead_veh.element(), fol_veh.element(),100)

    x.solve()
    # x.buildtrj(13.1345, 4.47734, -1.11671, -1.16845)
    # vis = MahmoudVisual(6)
    # vis.plotrj(x.indepVar, x.depVar, 2)
    # vis.makeplt()

    print()

    t2 = time.clock()
    print(' Elapsed Time: {} ms'.format(int(1000 * (t2 - t1))), end='')
