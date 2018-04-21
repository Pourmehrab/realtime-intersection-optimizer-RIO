#!/usr/bin/python3

####################################
# File name: main.py               #
# Author: Mahmoud Pourmehrab       #
# Email: mpourmehrab@ufl.edu       #
# Last Modified: Apr/15/2018       #
####################################


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
from unit_tests import test_scheduled_arrivals


def check_py_ver():
    # Checks for Python version
    expect_major, expect_minor, expect_rev = 3, 5, 2
    if sys.version_info[0] >= expect_major and sys.version_info[1] >= expect_minor and sys.version_info[
        2] >= expect_rev:
        print("Python version requirement is met. ################################")
    else:
        print(
            "INFO: Script developed and tested with Python " + str(expect_major) + "." + str(expect_minor) + "." + str(
                expect_rev))
        print("Please update Python")
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
    """ The program is to optimize the performance of an isolated intersection under traffic of AV and conventional 
    vehicles.

    By: Mahmoud Pourmehrab """
    ################### SET SOME PARAMETERS PN LOGGING AND PRINTING BEHAVIOUR
    do_traj_computation = False  # speeds up
    log_at_vehicle_level = True  # writes the <inter_name>_vehicle_level.csv
    log_at_trj_point_level = False  # writes the <inter_name>_trj_point_level.csv
    print_trj_info, test_time = False, 0  # prints arrival departures in command line
    print_signal_detail = False  # prints signal info in command line
    print_clock = False  # prints the timer in command line
    # if print_trj_info:
    #     tester = SimTest(num_lanes)

    print(
        "University of Florida.\nBy Mahmoud Pourmehrab ######################\n")
    print("Interpreter Information ###################################")
    print("Python Path: ", sys.executable)
    print("Python Version: ", sys.version)
    # Check the interpreter to make sure using py version at least 3.5.2
    check_py_ver()

    if len(sys.argv) != 3 or sys.argv[1] not in ["13th16th", "reserv", ] or sys.argv[2] not in ["GA", "MCF", "pretimed",
                                                                                                "actuated"]:
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
    det_range = intersection.get_det_range()  # detection range in meters
    k, m = intersection.get_poly_params()  # polynomial degree and discretization level for trajectory optimization of CAVs

    # lanes object keeps vehicles in it
    lanes = Lanes(num_lanes)

    # load entire traffic generated in csv file
    traffic = Traffic(inter_name, num_lanes, log_at_vehicle_level, log_at_trj_point_level)

    # initialize trajectory planners
    lead_conventional_trj_estimator = LeadConventional(max_speed, min_headway)
    lead_connected_trj_optimizer = LeadConnected(max_speed, min_headway, k, m)
    follower_conventional_trj_estimator = FollowerConventional(max_speed, min_headway)
    follower_connected_trj_optimizer = FollowerConnected(max_speed, min_headway, k, m)

    # Set the signal control method
    if method == "GA":
        # define what subset of phase-lane incidence matrix should be used
        # minimal set of phase indices to cover all movements (17, 9, 8, 15) for 13th16th intersection
        signal = GA_SPaT(inter_name, (0, 1, 2, 3,), num_lanes, min_headway, print_signal_detail)
    elif method == "pretimed":
        signal = Pretimed(inter_name, num_lanes, min_headway, print_signal_detail)

    elif method == "MCF" or method == "actuated":
        raise Exception("This signal control method is not complete yet.")  # todo develop these

    # get the time when first vehicle shows up
    first_detection_time = traffic.get_first_detection_time()

    # set the start time to it
    simulator = Simulator(first_detection_time)

    # here we start doing optimization for all scenarios included in the csv file
    if log_at_vehicle_level:
        t_start = time.clock()  # to measure total run time (IS NOT THE SIMULATION TIME)

    while True:  # stops when all rows of csv are processed (a break statement controls this)
        simulation_time = simulator.get_clock()  # gets current simulation clock
        if print_clock:
            print("\nUPDATE AT CLOCK: {:.2f} SEC #################################".format(
                simulation_time))

        # UPDATE VEHICLES
        # remove/record served vehicles and phases
        traffic.serve_update_at_stop_bar(lanes, simulation_time, num_lanes)
        signal.update_SPaT(simulation_time)

        # add/update vehicles
        traffic.update_vehicles_info(lanes, num_lanes, simulation_time, max_speed, min_headway, k)
        # update space mean speed
        volumes = traffic.get_volumes(lanes, num_lanes, det_range)
        critical_volume_ratio = 3600 * volumes.max() / min_headway

        # DO SIGNAL OPTIMIZATION
        if method == "GA":
            signal.solve(lanes, critical_volume_ratio, num_lanes)
        elif method == "pretimed":
            signal.solve(lanes, num_lanes, max_speed)

        elif method == "MCF" or method == "actuated":
            raise Exception("This signal control method is not complete yet.")  # todo develop these

        test_scheduled_arrivals(lanes, num_lanes)  # just for testing purpose
        # now we should have sufficient SPaT to serve all
        if do_traj_computation:
            # DO TRAJECTORY OPTIMIZATION
            for lane in range(num_lanes):
                if bool(lanes.vehlist[lane]):  # not an empty lane
                    for veh_indx, veh in enumerate(lanes.vehlist[lane]):
                        if veh.redo_trj():  # false if we want to keep previous trajectory
                            veh_type = veh.veh_type
                            arrival_time = veh.scheduled_arrival
                            if veh_indx > 0 and veh_type == 1:  # follower CAV
                                lead_veh = lanes.vehlist[lane][veh_indx - 1]
                                lead_poly = lead_veh.poly_coeffs
                                lead_arrival_time = lead_veh.scheduled_arrival
                                lead_det_time = lead_veh.trajectory[0:, lead_veh.first_trj_point_indx]
                                model = follower_connected_trj_optimizer.set_model(veh, arrival_time, 0, max_speed,
                                                                                   lead_poly, lead_det_time,
                                                                                   lead_arrival_time)
                                follower_connected_trj_optimizer.solve(veh, model, arrival_time, max_speed)
                            elif veh_indx > 0 and veh_type == 0:  # follower conventional
                                lead_veh = lanes.vehlist[lane][veh_indx - 1]
                                follower_conventional_trj_estimator.solve(veh, lead_veh)
                            elif veh_indx == 0 and veh_type == 1:  # lead CAV
                                model = lead_connected_trj_optimizer.set_model(veh, arrival_time, 0, max_speed, True)
                                lead_connected_trj_optimizer.solve(veh, model, arrival_time, max_speed)
                            elif veh_indx == 0 and veh_type == 0:  # lead conventional
                                lead_conventional_trj_estimator.solve(veh)

                            veh.test_trj_points(simulation_time)  # todo remove if not testing
                            veh.set_redo_trj_false()  # todo eventually works with the fusion outputs

                        if print_trj_info and simulation_time >= test_time:
                            veh.print_trj_points(lane, veh_indx)

        # MOVE SIMULATION FORWARD
        if traffic.last_veh_in_last_sc_arrived():
            if not lanes.all_served(num_lanes) or traffic.unarrived_vehicles():
                simulator.next_sim_step()
                simulation_time = simulator.get_clock()
            else:  # simulation of a scenario ended move on to the next scenario
                if log_at_vehicle_level:
                    t_end = time.clock()  # THIS IS NOT SIMULATION TIME! IT"S JUST TIMING THE ALGORITHM
                    traffic.set_elapsed_sim_time(t_end - t_start)
                    if print_clock:
                        print("### ELAPSED TIME: {:2.2f} sec ###".format(int(1000 * (t_end - t_start)) / 1000), end="")

                traffic.reset_scenario()
                first_detection_time = traffic.get_first_detection_time()

                simulator = Simulator(first_detection_time)

                lanes = Lanes(num_lanes)

                signal.reset()

                if log_at_vehicle_level:
                    t_start = time.clock()  # reset the timer
        else:
            if lanes.all_served(num_lanes):  # all vehicles in the csv file are served
                if log_at_vehicle_level:
                    t_end = time.clock()  # THIS IS NOT SIMULATION TIME! IT"S JUST TIMING THE ALGORITHM
                    traffic.set_elapsed_sim_time(t_end - t_start)

                    # save the csv which has travel time column appended
                    traffic.save_csv(intersection.name)
                if log_at_trj_point_level:
                    traffic.close_trj_csv()
                break
            else:  # this is the last scenario but still some vehicles have not been served
                simulator.next_sim_step()
