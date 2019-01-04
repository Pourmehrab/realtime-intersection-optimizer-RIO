#!/usr/bin/python3
import sys
import os
import argparse
from time import perf_counter
from src.time_tracker import Timer
from src.intersection import Intersection, Lanes, TrajectoryPlanner
from src.traffic import SimTraffic, RealTimeTraffic
from src.traffic_io import TrafficListener, TrafficPublisher
from src.signal import MCF_SPaT
from src.util import *


def run_rio(args):
    """
    .. note:: Assumptions for trajectory generation:
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

    :Authors:
        Ash Omidvar <aschkan@ufl.edu>

        Mahmoud Pourmehrab <pourmehrab@gmail.com>
    :Date:
       Dec-2018
    :Organization:
        University of Florida
    """
    if args.run_with_signal_control:
        from src.sig_ctrl_interface import snmp_phase_ctrl

    intersection = Intersection(args.intersection)
    lanes = Lanes(intersection)
    start_time_stamp_name = datetime.now().strftime("%m-%d-%Y_%H-%M-%S")  # only for naming the CSV files
    if args.mode == "sim":
        traffic = SimTraffic(intersection, args.sc, start_time_stamp_name, args)
        initial_time_stamp = 0
        resolution = 1. / args.loop_freq
    else:
        tl = TrafficListener(args.traffic_listener_ip, args.traffic_listener_port)
        tp = TrafficPublisher(args.traffic_publisher_ip, args.traffic_publisher_port)
        traffic = RealTimeTraffic(tl.get_vehicle_data_queue(), tl.get_track_split_merge_queue(),
                                  tp.get_cav_traj_queue(), intersection, args.sc, args.do_logging)
        initial_time_stamp = datetime.utcnow()
        resolution = datetime.timedelta(seconds=(1. / args.loop_freq))
    time_tracker = Timer.get_timer(args.mode, initial_time_stamp, resolution)
    num_lanes = intersection._inter_config_params.get("num_lanes")

    # Load MCF_SPaT optimization module for initial SPat
    signal = MCF_SPaT(0, intersection, args.sc, start_time_stamp_name)
    # Load trajectory optimization sub-models and planner
    trajectory_generator = TrajectoryPlanner(intersection)

    # to measure the total RIO run time for performance measure (Different from RIO clock)
    if args.do_logging:
        t_start = perf_counter()
    try:
        optimizer_call_ctr = 0
        solve_freq = int(args.loop_freq / args.solve_freq)
        while True:  # stop when pandas gets empty in offline mode, and when run duration has been reached in online mode.
            run_time = time_tracker.get_elapsed_time()  # get current RIO clock
            intersection._inter_config_params.get("print_commandline") and print(
                "\n################################# CLOCK: {:>5.1f} SEC #################################".format(
                    run_time))

            # update the assigned trajectories
            traffic.serve_update_at_stop_bar(lanes, run_time, intersection)

            # add/update the vehicles
            if args.mode == "sim":
                traffic.get_traffic_info(lanes, run_time, intersection)
            else:
                traffic.get_traffic_info(lanes, time_tracker)

            # update earliest departure time
            lanes.reset_earliest_departure_times(lanes, intersection)

            if optimizer_call_ctr % solve_freq == 0:
                # update SPaT
                signal.update_SPaT(intersection, run_time, args.sc)
                # perform signal optimization
                signal.solve(lanes, intersection, trajectory_generator)
                # Send out IAMs to all CAVs
                traffic.publish(lanes)
            optimizer_call_ctr += 1

            # call the proper phase on ATC Controller 
            if args.run_with_signal_control:
                snmp_phase_ctrl(signal.SPaT_sequence[0], args.intersection)

            # Wrap up and log
            if (args.mode == "sim" and (traffic.last_veh_arr() and lanes.all_served(num_lanes))) or \
                    (args.mode == "realtime" and (args.run_duration < run_time)):
                if args.do_logging:
                    # elapsed_process_time = perf_counter() - t_start
                    # timer.log_time_stats(sc, inter_name, start_time_stamp, elapsed_process_time, )  # log timings
                    traffic.save_veh_level_csv(args.intersection, start_time_stamp_name)
                    traffic.close_trj_csv()
                    signal.close_sig_csv()
                    intersection._inter_config_params.get("print_commandline") and print(
                        "\n### Elapsed Process Time: {:>5d} ms ###".format(int(1000 * run_time)), \
                        "\n### Actual RIO run start time: {:>5d} micro sec. ###".format(int(1000000 * t_start)))
                break
            time_tracker.step()

    except KeyboardInterrupt:
        print("RIO got KeyboardInterrupt, shutting down threads")
        # close TrafficListener


if __name__ == "__main__":
    parser = argparse.ArgumentParser("Runtime arguments for RIO")
    parser.add_argument("--intersection", type=str, default="RTS",
                        help="The name of the intersection")
    parser.add_argument("--mode", type=str, default="sim",
                        help="Run with traffic from CSV (sim) or sensor fusion (realtime)")
    parser.add_argument("--run-duration", type=int, default=300,
                        help="Seconds to run until termination.")
    parser.add_argument("--loop-freq", type=float, default=0.5,
                        help="Frequency (Hz) to run the main loop")
    parser.add_argument("--solve-freq", type=float, default=0.5,
                        help="Frequency (Hz) to call the optimizer")
    parser.add_argument("--sc", type=int, default=1, help="Scenario code number")
    parser.add_argument("--traffic-listener-ip", type=str, default="localhost",
                        help="The IP address to connect via UDP to receive incoming traffic messages")
    parser.add_argument("--traffic-listener-port", type=int, default=7000,
                        help="The port number to connect via UDP to receive incoming traffic messages")
    parser.add_argument("--traffic-publisher-ip", type=str, default="localhost",
                        help="The IP address to connect via UDP to publish outgoing traffic messages")
    parser.add_argument("--traffic-publisher-port", type=int, default=7001,
                        help="The port number to connect via UDP to publish outgoing traffic messages")
    parser.add_argument("--do-logging", type=str_to_bool, default="False",
                        help="Toggle logging")
    parser.add_argument("--run-with-signal-control", type=str_to_bool, default="False")

    args = parser.parse_args()
    print("Interpreter Information")
    print("Python Path: ", sys.executable)
    print("Python Version: ", sys.version)

    if args.do_logging:
        suffix = "_" + time.strftime("%y%m%d_%H%M%S")
        args.log_dir = os.path.join("log", args.intersection + suffix)
        os.makedirs(args.log_dir)
        print("creating log dir {}...".format(args.log_dir))

    print("\nProgram Started ################# CLOCK: {:>5.1f} SEC #################################".format(0.0))
    run_rio(args)
    print("\nProgram Terminated.")
