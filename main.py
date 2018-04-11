#!/usr/bin/python3

####################################
# File name: main.py               #
# Author: Mahmoud Pourmehrab       #
# Email: mpourmehrab@ufl.edu       #
# Last Modified: Apr/10/2018       #
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
from src.inter.signal import GA_SPaT, Pretimed
# Trajectory Optimizers
from src.trj.traj import FollowerConnected, FollowerConventional, LeadConnected, LeadConventional
# testing
from unit_tests import SimTest


def check_py_ver():
    # Checks for Python version
    expect_major, expect_minor, expect_rev = 3, 5, 2
    if sys.version_info[0] >= expect_major and sys.version_info[1] >= expect_minor and sys.version_info[
        2] >= expect_rev:
        print('Python version requirement is met. ################################')
    else:
        print(
            "INFO: Script developed and tested with Python " + str(expect_major) + "." + str(expect_minor) + "." + str(
                expect_rev))
        print('Please update Python')
        sys.exit(-1)


def get_max_arrival_time(lanes):
    last_served_time_stamp = 0.0
    for lane in range(num_lanes):
        if bool(lanes.vehlist[lane]):  # not an empty lane
            for veh in lanes.vehlist[lane]:
                t = veh.trajectory[0, veh.last_trj_point_indx]
                if t > last_served_time_stamp:
                    last_served_time_stamp = t
    return last_served_time_stamp


if __name__ == "__main__":

    print(
        'University of Florida. By Mahmoud Pourmehrab ######################\n')
    print('Interpreter Information ###################################')
    print('Python Path: ', sys.executable)
    print('Python Version: ', sys.version)
    # Check the interpreter to make sure using py version at least 3.5.2
    check_py_ver()

    if len(sys.argv) != 3 or sys.argv[1] not in ['13th16th', 'reserv', ] or sys.argv[2] not in ['GA', 'MCF', 'pretimed',
                                                                                                'actuated']:
        print("Check the input arguments and try again.")
        sys.exit(-1)
    else:
        # Set the intersection name (Options: reserv, 13th16th)
        inter_name = sys.argv[1]
        # optimization method
        method = sys.argv[2]
        # also look in src/inter/data.py

    test_mode, test_time = True, 0  # need to supply unit_tests.py
    # if test_mode:
    #     tester = SimTest(num_lanes)
    #     tester.add_traj_to_matplotlib(veh, lane, veh_type)
    # if test_mode and simulation_time >= test_time:
    #     tester.matplotlib_show_save(traffic.active_sc, det_range, simulation_time,
    #                                 get_max_arrival_time(lanes))

    intersection = Intersection(inter_name)
    # intersection keeps lane-lane and phase-lane incidence dictionaries
    num_lanes = intersection.get_num_lanes()
    max_speed = intersection.get_max_speed()  # in m/s
    min_headway = intersection.get_min_headway()  # in seconds
    det_range = intersection.get_det_range()  # detection range in meters
    k, m = intersection.get_poly_params()  # polynomial degree and discretization level for trajectory optimization of CAVs

    # lanes object keeps vehicles in it
    lanes = Lanes(num_lanes)

    # load entire traffic generated in csv file
    traffic = Traffic(inter_name, num_lanes)

    # initialize trajectory planners
    lead_conventional_trj_estimator = LeadConventional(max_speed, min_headway)
    lead_connected_trj_optimizer = LeadConnected(max_speed, min_headway, k, m)
    follower_conventional_trj_estimator = FollowerConventional(max_speed, min_headway)
    follower_connected_trj_optimizer = FollowerConnected(max_speed, min_headway, k, m)

    # Set the signal control method
    if method == 'GA':
        # define what subset of phase-lane incidence matrix should be used
        # minimal set of phase indices to cover all movements (17, 9, 8, 15) for 13th16th intersection
        phase_set =
        signal = GA_SPaT(inter_name, allowable_phases, num_lanes, min_headway)
    elif method == 'pretimed':
        signal = Pretimed(inter_name, num_lanes, min_headway, allowable_phases)

    elif method == 'MCF' or method == 'actuated':
        raise Exception('This signal control method is not complete yet.')  # todo develop these

    # get the time when first vehicle shows up
    first_detection_time = traffic.get_first_detection_time()

    # set the start time to it
    simulator = Simulator(first_detection_time)

    # here we start doing optimization for all scenarios included in the csv file
    t_start = time.clock()  # to measure total run time (IS NOT THE SIMULATION TIME)
    while True:  # stops when all rows of csv are processed (a break statement controls this)
        simulation_time = simulator.get_clock()  # gets current simulation clock
        print('\nUPDATE AT CLOCK: {:.2f} SEC #################################'.format(
            simulation_time))

        # UPDATE VEHICLES
        # remove/record served vehicles and phases
        traffic.update_at_stop_bar(lanes, simulation_time, num_lanes)
        signal.update_STaT(simulation_time)

        # add/update vehicles
        traffic.update_on_vehicles(lanes, num_lanes, simulation_time, max_speed, min_headway, k)
        # update space mean speed
        volumes = traffic.get_volumes(lanes, num_lanes, det_range)
        critical_volume_ratio = 3600 * volumes.max() / min_headway

        # DO SIGNAL OPTIMIZATION
        # signal.set_critical_volumes(volumes) # todo pass over this for GA
        signal.solve(lanes, critical_volume_ratio, num_lanes)
        # now we have sufficient SPaT to serve all

        # DO TRAJECTORY OPTIMIZATION
        for lane in range(num_lanes):
            if bool(lanes.vehlist[lane]):  # not an empty lane
                veh = lanes.vehlist[lane][0]
                veh_type = veh.get_vehicle_type()

                arrival_time = veh.get_scheduled_arrival()

                # send to optimizer
                if veh_type == 1:
                    model = lead_connected_trj_optimizer.set_model(veh, arrival_time, 0, max_speed)
                    lead_connected_trj_optimizer.solve(veh, model, arrival_time)
                else:
                    lead_conventional_trj_estimator.solve(veh)

                if test_mode and simulation_time >= test_time:
                    veh.print_trj_points(lane, 0)

                for veh_indx in range(1, len(lanes.vehlist[lane])):
                    veh = lanes.vehlist[lane][veh_indx]
                    arrival_time = veh.get_scheduled_arrival()

                    lead_veh = lanes.vehlist[lane][veh_indx - 1]

                    # send to optimizer
                    if veh_type == 1:
                        lead_poly = lead_veh.get_poly_coeffs()
                        lead_arrival_time = lead_veh.get_scheduled_arrival()
                        model = follower_connected_trj_optimizer.set_model(veh, arrival_time, 0, max_speed,
                                                                           lead_poly, lead_veh.init_time,
                                                                           lead_arrival_time)
                        follower_connected_trj_optimizer.solve(veh, model, arrival_time)
                    else:
                        # follower_conventional_trj_estimator.solve(veh, lead_veh) # todo activate this
                        lead_conventional_trj_estimator.solve(veh)

                    if test_mode and simulation_time >= test_time:
                        veh.print_trj_points(lane, veh_indx)

        # MOVE SIMULATION FORWARD
        if traffic.last_veh_in_last_sc_arrived():
            if traffic.keep_scenario():
                simulator.next_sim_step()
                simulation_time = simulator.get_clock()
            else:
                # simulation of a scenario ended move on to the next scenario
                t_end = time.clock()  # THIS IS NOT SIMULATION TIME! IT'S JUST TIMING THE ALGORITHM
                traffic.set_elapsed_sim_time(t_end - t_start)
                print('### Elapsed Time: {:2.2f} sec ###'.format(int(1000 * (t_end - t_start)) / 1000), end='')

                # plot trajectories
                if test_mode:
                    tester.matplotlib_show_save(traffic.active_sc, det_range, first_detection_time, simulation_time)

                traffic.reset_scenario()
                first_detection_time = traffic.get_first_detection_time()

                simulator = Simulator(first_detection_time)

                lanes = Lanes(num_lanes)

                signal.reset()

                t_start = time.clock()  # reset the timer

        else:
            if lanes.all_served(num_lanes):
                # all vehicles in the csv file are served
                # save the csv which has travel time column appended
                traffic.save_csv(intersection.name)
                traffic.close_trj_csv()
                if test_mode:
                    tester.matplotlib_show_save(traffic.active_sc, det_range, first_detection_time, simulation_time)
                break
            else:
                # this is the last scenario but still some vehicles have not been served
                simulator.next_sim_step()
