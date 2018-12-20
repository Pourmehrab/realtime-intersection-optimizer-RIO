#!/usr/bin/python3

#######################################
# File name: main.py                  #
# Author: Ash Omidvar                 #
# Email: aschkan@ufl.edu              #
# Last Modified: Dec/12/2018 - Ash    #
#######################################


def run_rio(inter_name, sc, start_time_stamp, run_duration, offline=True):
    """
    .. note:: Assumptions for trajectory generation :
        - Trajectories must show departure at the stop bar, i.e. the distance to stop bar converges to zero, even if
        they are temporarily assigned.
        - The desired speed of vehicles shall not exceed the speed limit (see config file)
        - Certain parameter value estimation has been considered in rio (e.g., accel/decel rates and destination of
        conventional vehicles.

    :param inter_name: Intersection name (should match the name given in ``config.py``)
    :type inter_name: str
    :param sc: In offline mode this parameter refers to certain log file, and it should match
    ``/data/<intersection name>/<intersection name>_<scenario number>.<log format>``. In online mode, this parameter is
    used for logging purpose. set arbitrarily or according to traffic listener.
    :type sc: int
    :param start_time_stamp: The local time stamp (used for logging purpose mainly)
    :param offline: if True, RIO runs offline by reading arrival information from a log file, dumping it in a pandas
    structure and occupying ``vehlist`` data structure from it. offline = False (or ``online mode`` is designed for the
    real-world field implementation and arrival info should be obtained from traffic listener (the fusion module.)
    :type offline: boolean
    :param run_duration: The duration we want to run RIO (in seconds.miliseconds). This parameter is in effect in online
     mode. Leave the field blank in offline mode
    :type run_duration: float
    :Author:
        Ash Omidvar <aschkan@ufl.edu>
    :Date:
       Dec-2018
    :Organization:
        University of Florida
    """
    import time, sched

    intersection = Intersection(inter_name)
    lanes = Lanes(intersection)
    traffic = OffTraffic(intersection, sc, start_time_stamp)
    first_detection_time = 0  # Start point of operation
    num_lanes = intersection._inter_config_params.get("num_lanes")

    # Load MCF_SPaT optimization module for initial SPat
    signal = MCF_SPaT(first_detection_time, intersection, sc, start_time_stamp)

    # Load trajectory optimization sub-models and planner
    trajectory_generator = TrajectoryPlanner(intersection)
    timer = Timer(first_detection_time)

    # to measure the total RIO run time for performance measure (Different from RIO clock)
    if intersection._inter_config_params.get("log_csv"):
        t_start = perf_counter()

    while True:  # stop when pandas gets empty in offline mode, and when run duration has been reached in online mode.
        opt_run_time = timer.opt_clock  # get current RIO clock
        traf_run_time = timer.traf_clock
        # nscheduler = sched.scheduler(time.time, time.sleep)
        intersection._inter_config_params.get("print_commandline") and print(
            "\n################################# CLOCK: {:>5.1f} SEC #################################".format(
                traf_run_time))

        # update the assigned trajectories
        traffic.serve_update_at_stop_bar(lanes, traf_run_time, intersection)

        # add/update the vehicles
        if offline:
            traffic.get_traffic_info(lanes, traf_run_time, intersection)
        # else:
            # TODO: @Pat: Listen to Pat's OffTraffic Listener.

        # update earliest departure time
        lanes.reset_earliest_departure_times(lanes, intersection)

        # update SPaT
        signal.update_SPaT(intersection, traf_run_time, sc)

        # update the space mean speed
        volumes = traffic.get_volumes(lanes, intersection)
        critical_volume_ratio = 3_600 * volumes.max() / intersection._inter_config_params.get(
            "min_CAV_headway")

        # perform signal optimization

        signal.solve(lanes, intersection, critical_volume_ratio, trajectory_generator, None)

        # call the proper phase on ATC Controller # ToDo: comment the line below when not connected to signal controller
        # snmp_phase_ctrl(signal.SPaT_sequence[0], inter_name)

        # Wrap up and log
        if offline: #TODO" @Pat: You may add additional logging or time measurement code here.
            if traffic.last_veh_arr() and lanes.all_served(num_lanes):
                if intersection._inter_config_params.get("log_csv"):
                    elapsed_process_time = perf_counter() - t_start
                    timer.log_time_stats(sc, inter_name, start_time_stamp, elapsed_process_time, )  # log timings
                    traffic.save_veh_level_csv(inter_name, start_time_stamp)
                    traffic.close_trj_csv()
                    signal.close_sig_csv()
                    intersection._inter_config_params.get("print_commandline") and print(
                        "\n### Elapsed Process Time: {:>5d} ms ###".format(int(1000 * elapsed_process_time)), "\n"
                        "### Actual RIO run start time: {:>5d} micro sec. ###".format(int(1000000 * t_start)))
                return

            else:
                timer.next_traf_time_step()
                time.sleep(timer.traf_res)
        else:  # online mode
            if run_duration <= opt_run_time:
                if intersection._inter_config_params.get("log_csv"):
                    elapsed_process_time = perf_counter() - t_start
                    timer.log_time_stats(sc, inter_name, start_time_stamp, elapsed_process_time, )  # log timings
                    traffic.save_veh_level_csv(inter_name, start_time_stamp)
                    traffic.close_trj_csv()
                    signal.close_sig_csv()
                    intersection._inter_config_params.get("print_commandline") and print(
                        "\n### Elapsed Process Time: {:>5d} ms ###".format(int(1000 * elapsed_process_time)), "\n"
                        "### Actual RIO run start time: {:>5d} micro sec. ###".format(int(1000000 * t_start)))


if __name__ == "__main__":
    import sys
    import os

    from datetime import datetime
    from time import perf_counter
    from src.time_tracker import Timer
    from src.intersection import Intersection, Lanes, OffTraffic, TrajectoryPlanner
    from src.signal import MCF_SPaT  # , GA_SPaT
    from src.sig_ctrl_interface import snmp_phase_ctrl


    print("Interpreter Information")
    print("Python Path: ", sys.executable)
    print("Python Version: ", sys.version)

    inter_name = "RTS"  # look in config file for the target intersection names.

    not os.path.isdir("./log/" + inter_name) and os.makedirs("./log/" + inter_name)

    print("\nProgram Started ################# CLOCK: {:>5.1f} SEC #################################".format(0.0))
    start_time_stamp = datetime.now().strftime("%m-%d-%Y_%H-%M-%S")  # only for naming the CSV files
    run_rio(inter_name, 1, start_time_stamp, 300.00, True)
    print("\nProgram Terminated.")
