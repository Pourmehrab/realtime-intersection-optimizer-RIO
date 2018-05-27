#!/usr/bin/python3

####################################
# File name: main.py               #
# Author: Mahmoud Pourmehrab       #
# Email: pourmehrab@gmail.com      #
# Last Modified: Apr/24/2018       #
####################################

class Singleton(type):
    """
    Only to make singleton classes.

    The credit for this goes to `this <https://stackoverflow.com/q/6760685>`_ stackoverflow  post.
    """
    _instances = {}

    def __call__(cls, *args, **kwargs):
        if cls not in cls._instances:
            cls._instances[cls] = super(Singleton, cls).__call__(*args, **kwargs)
        return cls._instances[cls]


def check_py_ver():
    """ checks the python version to meet the requirement (``ver 3.6.4``)"""
    expect_major, expect_minor, expect_rev = 3, 6, 4
    if sys.version_info[0] >= expect_major and sys.version_info[1] >= expect_minor and sys.version_info[
        2] >= expect_rev:
        print("Python version requirement is met.\n")
    else:
        print(
            "INFO: Script developed and tested with Python " + str(expect_major) + "." + str(expect_minor) + "." + str(
                expect_rev))
        print("Please update python interpreter.")
        sys.exit(-1)


def run_avian(inter_name, method, sc, start_time_stamp):
    """
    .. note::
        - Trajectories must end at the stop bar, i.e. the distance to stop bar converges to zero, even if they are temporarily assigned.
        - list all the other assumptions here...

    :param inter_name: intersection name
    :type inter_name: str
    :param method: pretimed, GA, ...
    :type method: str
    :param sc: scenario number (*should match the appendix of the input CSV filename*)
    :type sc: int
    :param start_time_stamp: The UTC time stamp to name the CSV files
    :param log_csv: If ``True``, the results get stored in the CSV files.
    :param print_commandline: If ``True``, details will be shown on real-time in the command line
    :param optional_packages_found: optional packages for testing

    :Author:
        Mahmoud Pourmehrab <pourmehrab@gmail.com>
    :Date:
        April-2018
    :Organization:
        University of Florida
    """
    intersection = Intersection(inter_name)

    lanes = Lanes(intersection)

    # load entire traffic generated in CSV file
    traffic = Traffic(intersection, sc, start_time_stamp)

    # get the time when first vehicle shows up
    first_detection_time = traffic.get_first_detection_time()

    # Set the signal control method
    if method == "GA":
        # define what subset of phase-lane incidence matrix should be used
        # minimal set of phase indices to cover all movements (17, 9, 8, 15) for 13th16th intersection
        # NOTE TEH SET OF ALLOWABLE PHASE ARRAY IS ZERO-BASED (not like what inputted in data.py)
        signal = GA_SPaT(first_detection_time, intersection, sc, start_time_stamp)
    elif method == "pretimed":
        signal = Pretimed(first_detection_time, intersection, sc, start_time_stamp)

    trajectory_planner = TrajectoryPlanner(intersection)

    # set the start time to it
    time_keeper = TimeKeeper(first_detection_time)

    # here we start doing optimization for all scenarios included in the CSV file
    if intersection._general_params.get('log_csv'):
        t_start = perf_counter()  # to measure total run time (IS NOT THE SIMULATION TIME)

    while True:  # stops when all rows of CSV are processed (a break statement controls this)
        simulation_time = time_keeper.clock  # gets current simulation clock
        if intersection._general_params.get('print_commandline'):
            print("\n################################# CLOCK: {:>5.1f} SEC #################################".format(
                simulation_time))

        # UPDATE VEHICLES
        # remove/record served vehicles and phases
        traffic.serve_update_at_stop_bar(lanes, simulation_time, intersection)
        # add/update vehicles
        traffic.update_vehicles_info(lanes, simulation_time, intersection)
        # update earliest departure schedule
        lanes.refresh_earliest_departure_times(lanes, intersection)
        # update SPaT
        signal.update_SPaT(intersection, simulation_time, sc)

        # update space mean speed
        volumes = traffic.get_volumes(lanes, intersection)
        critical_volume_ratio = 3_600 * volumes.max() / intersection._general_params.get('min_headway')

        # DO SIGNAL OPTIMIZATION
        signal.solve(lanes, intersection, critical_volume_ratio, trajectory_planner)

        if intersection._general_params.get('test_mode'):  # todo remove after testing
            num_lanes = intersection._general_params.get('num_lanes')
            test_departure_of_trj(lanes, intersection, [0] * num_lanes, lanes.last_vehicle_indx)

        # MOVE SIMULATION FORWARD
        if traffic.last_veh_arrived() and lanes.all_served(num_lanes):
            if intersection._general_params.get('log_csv'):
                elapsed_time = perf_counter() - t_start  # THIS IS NOT SIMULATION TIME! IT'S JUST FOR TIMING THE ALGORITHM
                traffic.set_elapsed_sim_time(elapsed_time)
                if intersection._general_params.get('print_commandline'):
                    print("\n### ELAPSED TIME: {:>5d} ms ###".format(int(1000 * elapsed_time)))

                # save the csv which has travel time column appended
                traffic.save_veh_level_csv(inter_name, start_time_stamp)

            if intersection._general_params.get('log_csv'):
                traffic.close_trj_csv()  # cus this is written line y line
                signal.close_sig_csv()
            return  # this halts the program

        else:  # this is the last scenario but still some vehicles have not been served
            time_keeper.next_sim_step()


if __name__ == "__main__":
    # IMPORT NECESSARY PACKAGES
    import sys, os
    from datetime import datetime
    from time import perf_counter

    from src.time_keeper import TimeKeeper
    from src.intersection import Intersection, Lanes, Traffic, TrajectoryPlanner
    # Signal Optimizers
    from src.signal import GA_SPaT, Pretimed

    # testing
    try:
        from src.optional.test.unit_tests import test_departure_of_trj
    except ModuleNotFoundError:
        pass

    print("Interpreter Information")
    print("Python Path: ", sys.executable)
    print("Python Version: ", sys.version)

    check_py_ver()  # Check the interpreter to make sure using right python version

    if len(sys.argv) != 4 or \
            sys.argv[1] not in {"13th16th", "TERL", "reserv", } or \
            sys.argv[2] not in {"GA", "MCF", "pretimed", "actuated"} or \
            sys.argv[3] not in {"simulation", "realtime"}:

        raise Exception("Check the input arguments and try again.")
    else:  # input arguments are good, run the rest
        inter_name, method, run_mode = sys.argv[1], sys.argv[2], sys.argv[3]
        if not os.path.isdir('./log/' + inter_name):
            os.mkdir('./log/' + inter_name)

        if run_mode == 'simulation':
            print(
                "\n################################# CLOCK: {:>5.1f} SEC #################################".format(0.0))
            start_time_stamp = datetime.now().strftime('%m-%d-%Y_%H:%M:%S')  # only for naming the CSV files
            target_sc = 42
            for sc in range(target_sc, target_sc + 1):
                run_avian(inter_name, method, sc, start_time_stamp)
        elif run_mode == 'realtime':
            raise Exception('real-time mode is not available yet.')

    print("\nProgram Terminated.")
