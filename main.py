#!/usr/bin/python3

####################################
# File name: main.py               #
# Author: Mahmoud Pourmehrab       #
# Email: mpourmehrab@ufl.edu       #
# Last Modified: Apr/03/2018       #
####################################

'''
The program is to optimize the performance of an isolated intersection under traffic of AV and conventional vehicles.

By: Mahmoud Pourmehrab
'''

import os
# import datetime
import sys
import time
import warnings

from src.inpt.sim import Simulator
from src.inter.inter import Intersection, Signal
from src.trj.trj_planner import trj_planner
# from src.inter.mcfopt import SigMinCostNet # todo find out the issue with ortools import
from src.inter.veh import Lanes, Vehicle
from src.inpt.get_traffic import Traffic
from src.vis.tikzpans import TikZpanels, TikzDirectedGraph
from src.vis.vis import VisTrj


def mcf_signal_optimizer(intersection, num_lanes, ppi, max_speed, signal, lanes, sim_prms):
    '''
    This function optimizes the scenario under following assumptions:

    - The full set of phases is supported
    - Uses min cost flow model to pick the duration and seq of phases
    '''
    signal = SigMinCostNet(num_lanes, ppi)
    # makes min cost flow network

    # do sample signal optimization
    lanes_demand = [3, 2, 3, 4, 5, 2, 6, 3, 2, 7, 3, 5, 2, 5, 3, 6]
    signal.set_dem(lanes_demand)
    signal.solve()

    # make min cost flow graph
    tikzobj = TikzDirectedGraph(inter_name, num_lanes, ppi)
    tikzobj.set_mcf_orig()
    tikzobj.set_phase_graph()
    # Make panels of phases
    tikzobj = TikZpanels(inter_name, num_lanes, ppi)


def stochastic_optimizer_GA(intersection, traffic, num_lanes, allowable_phases, max_speed, lanes):
    '''
    Based on the paper submitted to XXXXX (April 2018)

    Assumptions:
    - The sequence and duration is decided optimally by a Genetic Algorithms
    - The trajectories are computed using:
        - Gipps car following model for conventional vehicles
        - Polynomial degree k area under curve minimization for Lead/Follower AVs

    :param allowable_phases: subset of all possible phases is used (no limitation but it should cover all movements)
    '''

    signal = Signal(intersection.name, allowable_phases)
    # define what subset of phase-lane incidence matrix should be used

    # get the time when first vehicle shows up
    t = traffic.get_first_arrival()
    # set the start time to it
    simulator = Simulator(t)

    while True:  # stops when all rows of csv are processed (a break statement controls this)
        t = simulator.get_clock()  # gets current simulation clock
        print('################################ INQUIRY @ {:2.2f} SEC #################################'.format(t))

        # UPDATE VEHICLES
        # remove/record served vehicles and phases
        traffic.update_at_stop_bar(lanes, t, num_lanes)
        signal.update_STaT(t)

        # add/update vehicles todo (Patrick): update part will be come to play in real-time mode
        traffic.update_within_det_range(lanes, t,
                                        max_speed)  # checks for new vehicles in all incoming lanes (also sets earliest time)

        # DO SIGNAL OPTIMIZATION
        signal.GA_on_SPaT(lanes, num_lanes, allowable_phases)
        # now we have sufficient SPaT to serve all

        # DO TRAJECTORY OPTIMIZATION
        trj_planner(signal, lanes, num_lanes, max_speed)

        # MOVE SIMULATION FORWARD
        if traffic.keep_simulating():
            if traffic.keep_scenario():
                simulator.next_sim_step()
            else:
                # simulation of a scenario ended move on to the next scenario
                traffic.reset_scenario()

                t = traffic.get_first_arrival()
                simulator = Simulator(t)

                lanes = Lanes(num_lanes)

                signal = Signal(intersection.name, allowable_phases)

        else:
            # simulation of all scenarios ended save the csv which has travel time column appended
            traffic.save_csv(intersection.name)
            break

        # # plot = VisTrj(lane)  # todo:(Mahmoud) do we need lane information in this file?
        # # plot.plotrj(fol_veh.element().trj_t, fol_veh.element().trj_d,fol_veh.element().trj_s)


def check_py_ver():
    # Checks for Python version
    expect_major, expect_minor, expect_rev = 3, 5, 4
    if sys.version_info[:3] != (expect_major, expect_minor, expect_rev):
        print(
            "INFO: Script developed and tested with Python " + str(expect_major) + "." + str(expect_minor) + "." + str(
                expect_rev))
        current_version = str(sys.version_info[0]) + "." + str(sys.version_info[1]) + "." + str(sys.version_info[2])
        if sys.version_info[:2] != (expect_major, expect_minor):
            warnings.warn("Current Python version was unexpected: Python " + current_version)
        else:
            print("      Current version is different: Python " + current_version)
        sys.exit(-1)
    else:
        print('Python version requirement is met.\n')


if __name__ == "__main__":
    # Check the interpreter to make sure using py version at least 3.5.4
    check_py_ver()

    print('Interpreter Information ###################################################################################')
    print('Python Path: ', sys.executable)
    print('Python Version: ', sys.version)
    print(
        'University of Florida. Written by Mahmoud Pourmehrab ######################################################\n')

    if len(sys.argv) != 3 or sys.argv[1] not in ['13th16th', 'h-aim', ] or sys.argv[2] not in ['stoch', 'mcf', ]:
        print("Check the input arguments and try again.")
        sys.exit(-1)
    else:
        # Set the intersection name (Options: reserv, 13th16th)
        inter_name = sys.argv[1]
        # optimization method
        method = sys.argv[2]
        # also look in src/inter/data.py

    intersection = Intersection(inter_name)
    # intersection keeps lane-lane and phase-lane incidence dictionaries
    num_lanes = intersection.get_num_lanes()
    max_speed = intersection.get_max_speed()  # in m/s

    # lanes is a dictionary where key is lane index and value is a doubly-linked list that keeps vehicles in that lane
    # it also has methods to keep track of last optimized vehicle in every lane
    lanes = Lanes(num_lanes)

    # load entire traffic generated in csv file
    traffic = Traffic(inter_name)

    t_start = time.clock()
    # here we start doing optimization for all scenarios which exist in the csv file

    if method == 'stoch':
        stochastic_optimizer_GA(intersection, traffic, num_lanes, (17, 9, 8, 15), max_speed, lanes)
    elif method == 'mcf':
        pass

    t_end = time.clock()
    print('### Elapsed Time: {:2.2f} sec ###'.format(int(1000 * (t_end - t_start)) / 1000), end='')
