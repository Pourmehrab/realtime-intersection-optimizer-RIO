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

import sys
import time

from src.input.get_traffic import Traffic
from src.input.sim import Simulator
from src.inter.inter import Intersection
from src.inter.lane import Lanes
# Signal Optimizers
from src.inter.signal import GA_SPaT, MinCostFlow_SPaT
from src.optional.TikZ.tikzpans import TikZpanels, TikzDirectedGraph


# Trajectory Optimizers


def mcf_signal_optimizer(intersection, num_lanes, ppi, max_speed, signal, lanes, sim_prms):
    '''
    todo: merge this to main below
    This function optimizes the scenario under following assumptions:

    - The full set of phases is supported
    - Uses min cost flow model to pick the duration and seq of phases
    '''
    signal = MinCostFlow_SPaT(num_lanes, ppi)
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


def check_py_ver():
    # Checks for Python version
    expect_major, expect_minor, expect_rev = 3, 5, 2
    if sys.version_info[0] >= expect_major and sys.version_info[1] >= expect_minor and sys.version_info[
        2] >= expect_rev:
        print('Python version requirement is met.\n')
    else:
        print(
            "INFO: Script developed and tested with Python " + str(expect_major) + "." + str(expect_minor) + "." + str(
                expect_rev))
        print('Please update Python')
        sys.exit(-1)


if __name__ == "__main__":

    print('Interpreter Information ###################################################################################')
    print('Python Path: ', sys.executable)
    print('Python Version: ', sys.version)
    print(
        'University of Florida. Written by Mahmoud Pourmehrab ######################################################\n')

    # Check the interpreter to make sure using py version at least 3.5.2
    check_py_ver()

    if len(sys.argv) != 3 or sys.argv[1] not in ['13th16th', 'reserv', ] or sys.argv[2] not in ['GA', 'MCF', ]:
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

    # lanes object keeps vehicles in it
    lanes = Lanes(num_lanes)

    # load entire traffic generated in csv file
    traffic = Traffic(inter_name)

    if method == 'GA':
        # define what subset of phase-lane incidence matrix should be used
        # minimal set of phase indices to cover all movements (17, 9, 8, 15)
        signal = GA_SPaT(intersection.name, (17, 9, 8, 15,), num_lanes)

    elif method == 'MCF':
        pass

    # get the time when first vehicle shows up
    t = traffic.get_first_arrival()

    # set the start time to it
    simulator = Simulator(t)

    # here we start doing optimization for all scenarios included in the csv file
    t_start = time.clock()  # to measure total run time
    while True:  # stops when all rows of csv are processed (a break statement controls this)
        t = simulator.get_clock()  # gets current simulation clock
        print('################################ INQUIRY @ {:2.2f} SEC #################################'.format(t))

        # UPDATE VEHICLES
        # remove/record served vehicles and phases
        traffic.update_at_stop_bar(lanes, t, num_lanes)
        signal.update_STaT(t)

        # add/update vehicles
        traffic.update_on_vehicles(lanes, t, max_speed)

        # DO SIGNAL OPTIMIZATION
        signal.solve(lanes)
        # now we have sufficient SPaT to serve all

        # DO TRAJECTORY OPTIMIZATION
        # trj_planner(signal, lanes, num_lanes, max_speed)

        # MOVE SIMULATION FORWARD
        if traffic.last_veh_in_last_sc_arrived():
            if traffic.keep_scenario():
                simulator.next_sim_step()
            else:
                # simulation of a scenario ended move on to the next scenario
                t_end = time.clock()
                traffic.set_elapsed_sim_time(t_end - t_start)
                # print('### Elapsed Time: {:2.2f} sec ###'.format(int(1000 * (t_end - t_start)) / 1000), end='')

                traffic.reset_scenario()

                t = traffic.get_first_arrival()
                simulator = Simulator(t)

                lanes = Lanes(num_lanes)

                signal.reset()

                t_start = time.clock()  # reset the timer


        else:
            if lanes.all_served(num_lanes):
                # all vehicles in the csv file are served
                # save the csv which has travel time column appended
                traffic.save_csv(intersection.name)
                break
            else:
                # this is the last scenario but still some vehicles have not crossed
                simulator.next_sim_step()

        # # plot = VisTrj(lane)  # todo:(Mahmoud) do we need lane information in this file?
        # # plot.plotrj(fol_veh.element().trj_t, fol_veh.element().trj_d,fol_veh.element().trj_s)
