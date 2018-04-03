#!/usr/bin/python3

####################################
# File name: main.py               #
# Author: Mahmoud Pourmehrab       #
# Email: mpourmehrab@ufl.edu       #
# Last Modified: Apr/01/2018       #
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
from src.inter.signal import GA_SPaT
from src.optional.TikZ.tikzpans import TikZpanels, TikzDirectedGraph
# Visualization
from src.optional.vistrj import VisualizeSpaceTime
# Trajectory Optimizers
from src.trj.traj import FollowerConnected, FollowerConventional, LeadConnected, LeadConventional


def mcf_signal_optimizer(intersection, num_lanes, ppi, max_speed, signal, lanes, sim_prms):
    '''
    todo: UNDER DEVELOPMENT. merge this to main below
    This function optimizes the scenario under following assumptions:

    - The full set of phases is supported
    - Uses min cost flow model to pick the duration and seq of phases
    '''

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
    min_headway = intersection.get_min_headway()  # in seconds
    k, m = intersection.get_poly_params()
    # lanes object keeps vehicles in it
    lanes = Lanes(num_lanes)

    # instantiate the visualizer
    myvis = VisualizeSpaceTime(num_lanes, det_range=300)

    # load entire traffic generated in csv file
    traffic = Traffic(inter_name)

    # initialize trajectory planners
    lead_conventional_trj_estimator = LeadConventional(max_speed, min_headway)
    lead_connected_trj_optimizer = LeadConnected(max_speed, min_headway, k, m)
    follower_conventional_trj_estimator = FollowerConventional(max_speed, min_headway)
    follower_connected_trj_optimizer = FollowerConnected(max_speed, min_headway, k, m)

    # decide on signal optimization scheme
    if method == 'GA':
        # define what subset of phase-lane incidence matrix should be used
        # minimal set of phase indices to cover all movements (17, 9, 8, 15)
        signal = GA_SPaT(intersection.name, (17, 9, 8, 15,), num_lanes)  # todo allow more phases

    elif method == 'MCF':
        pass  # todo develop this

    # get the time when first vehicle shows up
    t = traffic.get_first_arrival() + 10  # TODO FIX THIS AFTER TESTING

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
        traffic.update_on_vehicles(lanes, t, max_speed, min_headway, k)

        # DO SIGNAL OPTIMIZATION
        signal.solve(lanes)
        # now we have sufficient SPaT to serve all

        # DO TRAJECTORY OPTIMIZATION
        for lane in range(num_lanes):
            if bool(lanes.vehlist[lane]):  # not an empty lane
                veh = lanes.vehlist[lane][0]
                veh.set_earliest_arrival(55)  # todo comes from GA
                arrival_time = veh.get_earliest_arrival()
                arrival_dist = 0  # todo comes from GA
                dep_speed = 15  # todo comes from GA
                green_start_time, yellow_end_time = 30, 40  # todo comes from GA
                # send to optimizer
                model = lead_connected_trj_optimizer.set_model(veh, arrival_time, arrival_dist, dep_speed,
                                                               green_start_time, yellow_end_time)
                lead_connected_trj_optimizer.solve(veh, model, arrival_time)

                veh.save_trj_to_excel(inter_name)
                myvis.add_multi_trj_matplotlib(veh, lane)

                for veh_indx in range(1, len(lanes.vehlist[lane])):
                    veh = lanes.vehlist[lane][veh_indx]
                    veh.set_earliest_arrival(58)  # todo comes from GA
                    arrival_time = veh.get_earliest_arrival()
                    arrival_dist = 0  # todo comes from GA
                    dep_speed = 15  # todo comes from GA
                    green_start_time, yellow_end_time = 30, 40  # todo comes from GA
                    # send to optimizer
                    lead_veh = lanes.vehlist[lane][veh_indx - 1]
                    lead_poly = lead_veh.get_poly_coeffs()
                    lead_arrival_time = lead_veh.get_earliest_arrival()
                    model = follower_connected_trj_optimizer.set_model(veh, arrival_time, arrival_dist, dep_speed,
                                                                       green_start_time, yellow_end_time,
                                                                       lead_poly, lead_veh.init_time, lead_arrival_time)

                    follower_connected_trj_optimizer.solve(veh, model, arrival_time)

                    # veh.save_trj_to_excel(inter_name)

                    myvis.add_multi_trj_matplotlib(veh, lane)
        # plot trjs todo move this to its place after scenario ends
        myvis.export_matplot(traffic.active_sc)

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
