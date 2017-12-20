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

from src.inter.inter import MahmoudIntersection
from src.inpt.sim import MahmoudSimulator
from src.inter.veh import MahmoudLanes, MahmoudVehicle
from src.inter.mcfopt import MahmoudSigNet

if __name__ == "__main__":
    print('Python Path: ', sys.executable)
    print('Python Version: ', sys.version)

    # Intersection name
    inter_name = '13th16th'

    # Initialization
    intersection = MahmoudIntersection(inter_name)
    num_lanes = intersection.get_num_lanes()
    ppi = intersection.get_phs()
    max_speed = intersection.get_max_speed()  # in m/s

    signal = MahmoudSigNet(num_lanes, ppi)

    lanes = MahmoudLanes(num_lanes)

    sim_prms = MahmoudSimulator(inter_name)
