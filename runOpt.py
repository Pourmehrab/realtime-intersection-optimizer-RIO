#!/usr/bin/python3
'''
Code is written in Python 3
Install the list of packages in the Pipfile using PyEnv


By:     Mahmoud Pourmehrab
E-mail: mpourmehrab@ufl.edu
Date:        Nov 2017
Last update: Dec/08/2017
'''

# import datetime
import sys
import time

import numpy as np

from cmp.inter import Intersection
from cmp.sim import Simulator
from cmp.veh import Lanes, Vehicle
from cmp.mcfopt import SigNet
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
    ppi = intersection.get_phs()

    signal = SigNet(num_lanes, ppi)
    lanes_demand = [3, 2, 3, 4, 5, 2, 6, 3, 2, 7, 3, 5, 2, 5, 3, 6]
    signal.set_dem(lanes_demand)
    signal.solve()

    t1 = time.clock()

    # add vehicles to the lane
    max_speed = intersection.get_max_speed()  # in m/s

    # work with vehicle data structure
    det_time = 0
    for i in range(100):
        lane = np.random.randint(0, num_lanes)
        det_id = 'veh_' + str(np.random.rand())
        det_type = np.random.randint(0, 3)
        det_time += np.random.exponential(2)  # 2 sec is the saturation headway here
        speed = np.random.triangular(0.85 * max_speed, max_speed, 1.1 * max_speed)
        dist = np.random.uniform(150 - 10, 150 + 10)  # 150 m is the detection distance

        # add vehicle to the list
        lanes.vehlist[lane].add_last(Vehicle(det_id, det_type, det_time, speed, dist, max_speed))
        print('A new vehicle detected at lane {:d}, (total of {:d})'.format(lane, len(lanes.vehlist[lane])))

        # set earliest travel time for this vehicle
        last = lanes.vehlist[lane].last()
        last.element().set_earlst(det_time + dist / (.85 * max_speed))

    p = lanes.vehlist[0].first()
    # lanes.vehlist[0].delete(p)



    # do some trj optimization
    lane = 0
    fol_veh = lanes.vehlist[lane].first()
    lead_veh = lanes.vehlist[lane].before(fol_veh)
    if lead_veh is None:
        x = MahmoudAVO(None, fol_veh.element(), 100)
    else:
        x = MahmoudAVO(lead_veh.element(), fol_veh.element(), 100)

    x.solve()
    # x.buildtrj(13.1345, 4.47734, -1.11671, -1.16845)
    # vis = MahmoudVisual(6)
    # vis.plotrj(x.indepVar, x.depVar, 2)
    # vis.makeplt()

    print()

    t2 = time.clock()
    print(' Elapsed Time: {} ms'.format(int(1000 * (t2 - t1))), end='')
