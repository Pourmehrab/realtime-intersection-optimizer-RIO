#!/usr/bin/python3

####################################
# File name: test.py               #
# Author: Mahmoud Pourmehrab       #
# Email: mpourmehrab@ufl.edu       #
# Last Modified: Feb/25/2018       #
####################################

import os
# import datetime
import sys
import time
import numpy as np

from src.inpt.sim import Simulator
from src.inter.inter import Intersection, Signal
from src.trj.trjopt import Connected
from src.trj.trjest import Conventional
from src.inter.mcfopt import SigMinCostNet
from src.inter.veh import Lanes, Vehicle
from src.inpt.get_traffic import Traffic
from src.vis.tikzpans import TikZpanels, TikzDirectedGraph
from src.vis.vis import VisTrj


def mcf_signal_optimizer(intersection, num_lanes, ppi, max_speed, signal, lanes, sim_prms):
    '''
    todo: mention what's different here
    first, we're using min cost flow for signal optimization
    '''
    # makes min cost flow network
    signal = SigMinCostNet(num_lanes, ppi)

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


def stochastic_optimizer(intersection, traffic, num_lanes, allowable_phases, max_speed, lanes):
    '''
    Based on the paper submitted to IEEE Intelligent Transportation Systems Magazine
    :param pli: phase lane incidence dictionary
    :param allowable_phases: subset of all possible phases is used (one per approach, east/south/west/north bounds
    respectively, that covers all movements is suggested)
    '''

    # first define what rows of phase-lane incidence matrix should be used
    signal = Signal(intersection.name, allowable_phases)

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

        # add/update vehicles todo (Patrick): update part will be valid for in real-time mode
        traffic.update_within_det_range(lanes, t,
                                        max_speed)  # checks for new vehicles in all incoming lanes (also sets earliest time)

        # DO SIGNAL OPTIMIZATION
        signal.do_spat_decision(lanes, num_lanes, allowable_phases)
        # now we have sufficient SPaT to serve all

        # DO TRAJECTORY OPTIMIZATION

        # optimize like lead
        # for l in range(num_lanes):
        #     last_detected = lanes.vehlist[l].last()
        #     last_optimized = lanes.vehlist[l].last_indx
        #     if last_detected != None and last_optimized != last_optimized:  # there are vehicle(s) to optimize
        #         next_veh_indx = lanes.vehlist[l].after(last_optimized)
        #         while next_veh_indx != None:
        #             trj_planner = Connected(generic_lead, generic_follower, gs=45)
        #             trj_planner.solve(1)  # pass 1 for follower vehicle (when first argument is not None)

        # MOVE SIMULATION FORWARD
        if traffic.keep_simulating():
            if traffic.keep_scenario():
                simulator.next_sim_step()
            else:
                print('###### SIMULATION OF THE SCENARIO CONCLUDED ######')
                traffic.reset_scenario()

                t = traffic.get_first_arrival()
                simulator = Simulator(t)

                lanes = Lanes(num_lanes)

                signal = Signal(intersection.name, allowable_phases)

        else:
            # simulation ends save the csv which has travel time column in it
            traffic.save_csv(intersection.name)
            break

        # # plot = VisTrj(lane)  # todo:(Mahmoud) do we need lane information in this file?
        # # plot.plotrj(fol_veh.element().trj_t, fol_veh.element().trj_d,fol_veh.element().trj_s)


if __name__ == "__main__":
    print('Interpreter Information ###################################################################################')
    print('Python Path: ', sys.executable)
    print('Python Version: ', sys.version)
    print(
        '###########################################################################################################\n')

    # Intersection name
    # there should be a csv file under `data` (refer to readme for details)
    # also look in src/inter/data.py
    inter_name = '13th16th'  # Options: reserv, 13th16th

    # Instantiations of necessary objects
    # intersection keeps lane-lane and phase-lane incidence dictionaries
    intersection = Intersection(inter_name)
    num_lanes = intersection.get_num_lanes()
    max_speed = intersection.get_max_speed()  # in m/s

    # lanes is a dictionary where key is lane index and value is a doubly-linked list that keeps vehicles in that lane
    # it also has methods to keep track of last optimized vehicle in every lane
    lanes = Lanes(num_lanes)

    # load entire traffic generated in csv file
    traffic = Traffic(inter_name)

    t1 = time.clock()
    # here we start doing optimization for all scenarios which exist in the csv file read before
    # stochastic_optimizer(.) works based on the logic provided in paper submitted to IEEE ITS
    # find more details inside the function
    stochastic_optimizer(intersection, traffic, num_lanes, (17, 9, 8, 15), max_speed, lanes)
    t2 = time.clock()
    print('### Elapsed Time: {:2.2f} sec ###'.format(int(1000 * (t2 - t1)) / 1000), end='')
