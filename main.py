#!/usr/bin/python3

####################################
# File name: main.py               #
# Author: Mahmoud Pourmehrab       #
# Email: mpourmehrab@ufl.edu       #
# Last Modified: Apr/22/2018       #
####################################


def check_py_ver():
    """ checks the python version to meet the requirement (``ver 3.6.4``)"""
    expect_major, expect_minor, expect_rev = 3, 6, 4
    if sys.version_info[0] >= expect_major and sys.version_info[1] >= expect_minor and sys.version_info[
        2] >= expect_rev:
        print("Python version requirement is met.\n")
    else:
        print(
            "INFO: Script developed and tested with Python " + str(expect_major) + "." + str(
                expect_minor) + "." + str(expect_rev))
        print("Please update python interpreter.")
        sys.exit(-1)


def run_avian(inter_name, method, sc, do_traj_computation, log_at_vehicle_level, log_at_trj_point_level,
              log_signal_status, print_commandline, optional_packages_found):
    """
    For logging and printing of information set boolean variables:
        - ``log_at_trj_point_level`` saves a csv under ``\log`` directory that contains all trajectory points for all vehicles
        - ``log_at_vehicle_level`` saves a csv file under ``\log`` directory that contains departure times and elapsed times and vehicle IDs

    The work flow is as the following:
        - Tests for python version
        - Checks the input arguments to be valid
        - Instantiate:
            - :any:`Intersection`
            - :any:`Lanes`
            - :any:`Traffic`
            - trajectory planners: all bellow
                - :any:`LeadConventional`
                - :any:`LeadConnected`
                - :any:`FollowerConventional`
                - :any:`FollowerConnected`
            - signal: one of followings
                - :any:`GA_SPaT`
                - :any:`Pretimed`
        - set simulation start time to when first vehicle shows up
            - :any:`TimeKeeper`
        - main loop stops only when all vehicles in the provided input traffic csv file are assigned a departure time.
            - remove vehicles that are served
            - update SPaT
            - update vehicle information (includes addition too)
            - do signal
            - plan trajectories
            - update time and check of termination

    :param inter_name: intersection name
    :type inter_name: str
    :param method: pretimed, GA, ...
    :type method: str
    :param sc: scenario number (*should match the appendix of the input csv filename*)
    :type sc: int
    :param do_traj_computation:
    :param log_at_vehicle_level:
    :param log_at_trj_point_level:
    :param log_signal_status:
    :param print_commandline:
    :param optional_packages_found: optional packages for testing
    :return:


    :Author:
        Mahmoud Pourmehrab <pourmehrab@gmail.com>
    :Date:
        April-2018
    :Organization:
        University of Florida
    """
    # to mark saved csv file
    if log_at_vehicle_level or log_at_trj_point_level or log_signal_status:
        start_time_stamp = 'output'
        # start_time_stamp = datetime.utcnow().strftime('%B %d %Y - %H:%M:%S')

    intersection = Intersection(inter_name)
    # get some useful values
    num_lanes = intersection.get_num_lanes()
    max_speed = intersection.get_max_speed()  # in m/s
    min_headway = intersection.get_min_headway()  # in seconds
    det_range = intersection.get_det_range()  # detection range in meters
    k, m = intersection.get_poly_params()  # polynomial degree and discretization level for trajectory optimization of CAVs

    lanes = Lanes(num_lanes)

    # load entire traffic generated in csv file
    traffic = Traffic(inter_name, sc, log_at_vehicle_level, log_at_trj_point_level, print_commandline, start_time_stamp)

    # Set the signal control method
    if method == "GA":
        # define what subset of phase-lane incidence matrix should be used
        # minimal set of phase indices to cover all movements (17, 9, 8, 15) for 13th16th intersection
        # NOTE TEH SET OF ALLOWABLE PHASE ARRAY IS ZERO-BASED (not like what inputted in data.py)
        allowable_phase = (0, 1, 2, 3,)
        signal = GA_SPaT(inter_name, allowable_phase, num_lanes, min_headway, log_signal_status, sc, start_time_stamp,
                         do_traj_computation, print_commandline,optional_packages_found)
    elif method == "pretimed":
        signal = Pretimed(inter_name, num_lanes, min_headway, log_signal_status, sc, start_time_stamp,
                          do_traj_computation, print_commandline,optional_packages_found)

    elif method == "MCF" or method == "actuated":
        raise Exception("This signal control method is not complete yet.")  # todo develop these

    trajectory_planner = TrajectoryPlanner(max_speed, min_headway, k, m)

    # get the time when first vehicle shows up
    first_detection_time = traffic.get_first_detection_time()
    # set the start time to it
    time_keeper = TimeKeeper(first_detection_time)

    # here we start doing optimization for all scenarios included in the csv file
    if log_at_vehicle_level:
        t_start = perf_counter()  # to measure total run time (IS NOT THE SIMULATION TIME)

    while True:  # stops when all rows of csv are processed (a break statement controls this)
        simulation_time = time_keeper.clock  # gets current simulation clock
        if print_commandline:
            print("\nUPDATE AT CLOCK: {:>5.1f} SEC #################################".format(
                simulation_time))

        # UPDATE VEHICLES
        # remove/record served vehicles and phases
        traffic.serve_update_at_stop_bar(lanes, simulation_time, num_lanes, print_commandline)
        # add/update vehicles
        traffic.update_vehicles_info(lanes, simulation_time, max_speed, min_headway, k)
        # update SPaT
        signal.update_SPaT(simulation_time, sc)

        # update space mean speed
        volumes = traffic.get_volumes(lanes, num_lanes, det_range)
        critical_volume_ratio = 3_600 * volumes.max() / min_headway

        # DO SIGNAL OPTIMIZATION
        if method in ("GA", "pretimed"):
            signal.solve(lanes, num_lanes, max_speed, critical_volume_ratio, trajectory_planner)
        else:
            raise Exception("The chosen signal method is not developed yet.")

        if optional_packages_found:
            test_scheduled_arrivals(lanes, num_lanes, max_speed)

        # MOVE SIMULATION FORWARD
        if traffic.last_veh_arrived() and lanes.all_served(num_lanes):
            if log_at_vehicle_level:
                elapsed_time = perf_counter() - t_start  # THIS IS NOT SIMULATION TIME! IT"S JUST TIMING THE ALGORITHM
                traffic.set_elapsed_sim_time(elapsed_time)
                if print_commandline:
                    print("\n### ELAPSED TIME: {:>5d} ms ###".format(int(1000 * elapsed_time)))

                # save the csv which has travel time column appended
                traffic.save_veh_level_csv(inter_name, start_time_stamp)

            if log_at_trj_point_level:
                traffic.close_trj_csv()  # cus this is written line y line
            if log_signal_status:
                signal.close_sig_csv()
            return  # this halts the program

        else:  # this is the last scenario but still some vehicles have not been served
            time_keeper.next_sim_step()

    # Nothing after the while loop gets executed


if __name__ == "__main__":
    # IMPORT NECESSARY PACKAGES
    import sys
    from time import perf_counter

    from src.time_keeper import TimeKeeper
    from src.intersection import Intersection, Lanes, Traffic, TrajectoryPlanner
    # Signal Optimizers
    from src.signal import GA_SPaT, Pretimed

    # testing
    try:
        from src.optional.test.unit_tests import test_scheduled_arrivals

        optional_packages_found = True
    except ModuleNotFoundError:
        optional_packages_found = False

    # ################## SET SOME PARAMETERS ON LOGGING AND PRINTING BEHAVIOUR
    do_traj_computation = True
    log_at_vehicle_level = True
    log_at_trj_point_level = True
    log_signal_status = True
    print_commandline = True

    print("Interpreter Information")
    print("Python Path: ", sys.executable)
    print("Python Version: ", sys.version)

    # Check the interpreter to make sure using right python version
    check_py_ver()

    if len(sys.argv) != 4 or \
            sys.argv[1] not in ["13th16th", "TERL", "reserv", ] or \
            sys.argv[2] not in ["GA", "MCF", "pretimed", "actuated"] or \
            sys.argv[3] not in ["simulation", "realtime"]:

        raise Exception("Check the input arguments and try again.")
    else:  # input arguments are good, run the rest
        inter_name, method, run_mode = sys.argv[1], sys.argv[2], sys.argv[3]

        if run_mode == 'simulation':
            target_sc = 1
            for sc in range(1, target_sc + 1):
                run_avian(inter_name, method, sc, do_traj_computation, log_at_vehicle_level, log_at_trj_point_level,
                          log_signal_status, print_commandline, optional_packages_found)

        elif run_mode == 'realtime':
            raise Exception('real-time mode is not available yet.')

    print("\nProgram Terminated.")
