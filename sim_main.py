#!/usr/bin/python3

####################################
# File name: sim_main.py           #
# Author: Mahmoud Pourmehrab       #
# Email: pourmehrab@gmail.com      #
# Last Modified: Jun/24/2018       #
####################################


def run_avian(inter_name, method, sc, start_time_stamp, tester):
    """
    .. note:: The following assumptions are important to notice:
        - Trajectories must end at the stop bar, i.e. the distance to stop bar converges to zero, even if they are temporarily assigned.
        - The desired speed of vehicles shall not exceed the speed limit or they will be advised speeding
        - Use default values for pieces of information that are impossible to obtain, i.e. accel/decel rates and destination of conventional vehicles.

    :param inter_name: intersection name
    :type inter_name: str
    :param method: pretimed, GA, ...
    :type method: str
    :param sc: scenario number (*should match the appendix of the input CSV filename*)
    :type sc: int
    :param start_time_stamp: The local time stamp to name the CSV files
    :param tester: the test object
    :type tester: test.unit_tests.SimTest

    :Author:
        Mahmoud Pourmehrab <pourmehrab@gmail.com>
    :Date:
        April-2018
    :Organization:
        University of Florida
    """
    intersection = Intersection(inter_name)
    lanes = Lanes(intersection)
    traffic = Traffic(intersection, sc, start_time_stamp)
    first_detection_time = traffic.get_first_detection_time()

    if method == "GA":
        signal = GA_SPaT(first_detection_time, intersection, sc, start_time_stamp)
    elif method == "pretimed":
        signal = Pretimed(first_detection_time, intersection, sc, start_time_stamp)
    trajectory_planner = TrajectoryPlanner(intersection)
    simulator = Simulator(first_detection_time)

    if intersection._general_params.get('log_csv'):
        t_start = perf_counter()  # to measure the total run time (IS NOT THE SIMULATION TIME)

    while True:  # stops when all rows of CSV are processed (a break statement controls this)
        simulation_time = simulator.clock  # gets current simulation clock
        intersection._general_params.get('print_commandline') and print(
            "\n################################# CLOCK: {:>5.1f} SEC #################################".format(
                simulation_time))

        # update the assigned trajectories
        traffic.serve_update_at_stop_bar(lanes, simulation_time, intersection)
        tester is not None and intersection._general_params.get('do_traj_computation') and tester.check_order_in_lanes(
            lanes)
        # add/update the vehicles
        traffic.get_traffic_info(lanes, simulation_time, intersection)
        # update earliest departure schedule
        lanes.refresh_earliest_departure_times(lanes, intersection)
        # update the SPaT
        signal.update_SPaT(intersection, simulation_time, sc)

        # update the space mean speed
        volumes = traffic.get_volumes(lanes, intersection)
        critical_volume_ratio = 3_600 * volumes.max() / intersection._general_params.get('min_headway')

        # perform signal optimization
        signal.solve(lanes, intersection, critical_volume_ratio, trajectory_planner, tester)

        num_lanes = intersection._general_params.get('num_lanes')
        tester is not None and tester.test_departure_of_trj(lanes, intersection, [0] * num_lanes,
                                                            lanes.last_vehicle_indx)

        # move it forward
        if traffic.last_veh_arrived() and lanes.all_served(num_lanes):
            if intersection._general_params.get('log_csv'):
                elapsed_time = perf_counter() - t_start  # THIS IS NOT SIMULATION TIME! IT'S JUST FOR TIMING THE ALGORITHM
                simulator.record_sim_stats(sc, inter_name, start_time_stamp, elapsed_time)
                traffic.save_veh_level_csv(inter_name, start_time_stamp)
                traffic.close_trj_csv()
                signal.close_sig_csv()
                intersection._general_params.get('print_commandline') and print(
                    "\n### ELAPSED TIME: {:>5d} ms ###".format(int(1000 * elapsed_time)))
            return

        else:
            simulator.next_sim_step()


if __name__ == "__main__":
    import sys
    import os
    from datetime import datetime
    from time import perf_counter
    from src.simulator import Simulator
    from src.intersection import Intersection, Lanes, Traffic, TrajectoryPlanner
    from src.signal import GA_SPaT, Pretimed
    from src.sig_ctrl_interface import snmp_phase_ctrl

    try:
        from test.unit_tests import SimTest

        tester = SimTest()
        tester.py_version_test()
        tester.arguments_check()
        # tester = None
    except ModuleNotFoundError:
        tester = None

    print("Interpreter Information")
    print("Python Path: ", sys.executable)
    print("Python Version: ", sys.version)

    inter_name, method, run_mode = sys.argv[1], sys.argv[2], sys.argv[3]
    not os.path.isdir('./log/' + inter_name) and os.makedirs('./log/' + inter_name)

    if run_mode == 'simulation':
        print(
            "\nProgram Started ################# CLOCK: {:>5.1f} SEC #################################".format(0.0))
        start_time_stamp = datetime.now().strftime('%m-%d-%Y_%H-%M-%S')  # only for naming the CSV files
        run_avian(inter_name, method, 1, start_time_stamp, tester)
    elif run_mode == 'realtime':
        raise Exception('this script is just for optimization simulation.')

    print("\nProgram Terminated.")
