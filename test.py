#!/usr/bin/python3
'''
Code is written in Python 3
Install the list of packages in the Pipfile using PyEnv


By:     Mahmoud Pourmehrab
E-mail: mpourmehrab@ufl.edu
Date:        Nov 2017
Last update: Dec/17/2017
'''

# import datetime
import sys
import time

import numpy as np

from src.inter.inter import Intersection
from src.inpt.sim import Simulator
from src.inter.veh import Lanes, Vehicle
from src.inter.mcfopt import SigNet
from src.trj.trjopt import MahmoudAVO
from src.vis.tikz import TikzDirectedGraph

if __name__ == "__main__":
    print('Python Path: ', sys.executable)
    print('Python Version: ', sys.version)

    # Intersection name
    inter_name = '13th16th'

    # Initialization
    intersection = Intersection(inter_name)
    num_lanes = intersection.get_num_lanes()
    ppi = intersection.get_phs()
    max_speed = intersection.get_max_speed()  # in m/s

    signal = SigNet(num_lanes, ppi)

    lanes = Lanes(num_lanes)

    sim_prms = Simulator(inter_name)

    t1 = time.clock()
    # sample signal optimization
    lanes_demand = [3, 2, 3, 4, 5, 2, 6, 3, 2, 7, 3, 5, 2, 5, 3, 6]
    signal.set_dem(lanes_demand)
    signal.solve()

    # make tikz graph
    tikzobj = TikzDirectedGraph(inter_name, num_lanes, ppi)
    tikzobj.set_mcf_orig()
    tikzobj.set_phase_graph()


    # add vehicles to the lane
    # work with vehicle data structure
    # det_time = 0
    # for i in range(100):
    #     lane = np.random.randint(0, num_lanes)
    #     det_id = 'veh_' + str(np.random.rand())
    #     det_type = np.random.randint(0, 3)
    #     det_time += np.random.exponential(2)  # 2 sec is the saturation headway here
    #     speed = np.random.triangular(0.85 * max_speed, max_speed, 1.1 * max_speed)
    #     dist = np.random.uniform(150 - 10, 150 + 10)  # 150 m is the detection distance
    #
    #     # add vehicle to the list
    #     lanes.vehlist[lane].add_last(Vehicle(det_id, det_type, det_time, speed, dist, max_speed))
    #     print('A new vehicle detected at lane {:d}, (total of {:d})'.format(lane, len(lanes.vehlist[lane])))
    #
    #     # set earliest travel time for this vehicle
    #     last = lanes.vehlist[lane].last()
    #     last.element().set_earlst(det_time + dist / (.85 * max_speed))
    #
    # p = lanes.vehlist[0].first()
    # # lanes.vehlist[0].delete(p)
    #
    # t2 = time.clock()
    # print(' Elapsed Time: {} ms'.format(int(1000 * (t2 - t1))), end='')
    # # do some trj optimization
    # lane = 0
    # fol_veh = lanes.vehlist[lane].first()
    # lead_veh = lanes.vehlist[lane].before(fol_veh)
    # if lead_veh is None:
    #     trjoptimizer = MahmoudAVO(None, fol_veh.element(), 100)
    # else:
    #     trjoptimizer = MahmoudAVO(lead_veh.element(), fol_veh.element(), 100)
    # trjoptimizer.solve()
