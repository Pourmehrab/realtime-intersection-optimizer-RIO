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

from src.inter.inter import Intersection
from src.inpt.sim import Simulator
from src.inter.veh import Lanes, Vehicle
from src.inter.mcfopt import SigNet

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
