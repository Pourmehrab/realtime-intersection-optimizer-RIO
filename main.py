#!/usr/bin/python3
import sys
import os
import argparse
import time
from time import perf_counter
import datetime as dt
from src.time_tracker import Timer
from src.intersection import Intersection, Lanes
from src.trajectory import TrajectoryPlanner
from src.traffic import SimTraffic, RealTimeTraffic
from src.data_io import TrafficListener, TrafficPublisher
from src.signal import MCF_SPaT
from src.util import *
from datetime import datetime
from multiprocessing import Process, Pipe


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

    intersection = Intersection(args.intersection)
    lanes = Lanes(intersection)
    start_time_stamp_name = datetime.now().strftime("%m-%d-%Y_%H-%M-%S")  # only for naming the CSV files
    if args.mode == "sim":
        traffic = SimTraffic(intersection, args.sc, start_time_stamp_name, args)
    else:
        tl = TrafficListener(args.traffic_listener_ip, args.traffic_listener_port)
        tp = TrafficPublisher(intersection, args.traffic_publisher_ip, args.traffic_publisher_port, args)
        traffic = RealTimeTraffic(tl.get_vehicle_data_queue(), tl.get_track_split_merge_queue(),
                                  tp.get_cav_traj_queue(), datetime.utcnow(), intersection, args)
        tl.start()
        tp.start()
    resolution = 1. / args.loop_freq
    num_lanes = intersection._inter_config_params.get("num_lanes")

    # Load MCF_SPaT optimization module for initial SPat
    signal = MCF_SPaT(args, 0, intersection, args.sc, start_time_stamp_name)
    # Load trajectory optimization sub-models and planner
    trajectory_generator = TrajectoryPlanner(intersection)

    # to measure the total RIO run time for performance measure (Different from RIO clock)
    if args.do_logging:
        t_start = perf_counter()
    # Start the signal controller
    if args.run_with_signal_control:
        print("Initializing signal controller...")
        from src.sig_ctrl_interface import main as sig_ctrl_main
        parent, child = Pipe()
        ps = Process(target=sig_ctrl_main, args=(args.intersection,
                     args.signal_controller_ip, args.signal_controller_port,
                     parent, child))
        ps.daemon = True # if this proc crashes, dont cause things to hang
        ps.start()
        child.close()
        print("Done initiailizing signal controller...")

    try:
        optimizer_call_ctr = 0
        solve_freq = int(args.loop_freq / args.solve_freq)
        time_tracker = Timer.get_timer(args.mode, resolution)
        while True:  # stop when pandas gets empty in offline mode, and when run duration has been reached in online mode.
            elapsed_time, absolute_time = time_tracker.get_time()  # get current RIO clock
            intersection._inter_config_params.get("print_commandline") and print(
                "\n################################# CLOCK: {:>5.1f} SEC #################################".format(
                    elapsed_time))

            # update the assigned trajectories
            traffic.update_trj_or_serve_at_stop_bar(lanes, elapsed_time, intersection)

            # add/update the vehicles
            if args.mode == "sim":
                traffic.get_traffic_info(lanes, elapsed_time, intersection)
            else:
                traffic.get_traffic_info(lanes, time_tracker)

            # update earliest departure time
            lanes.set_earliest_departure_times(lanes, intersection)

            if optimizer_call_ctr % solve_freq == 0:
                # update SPaT
                time_since_last_arrival = traffic.get_time_since_last_arrival(absolute_time)
                signal.update_SPaT(intersection, elapsed_time, args.sc, time_since_last_arrival, absolute_time)
                # perform signal optimization
                signal.solve(lanes, intersection, trajectory_generator, absolute_time)
                # Send out IAMs to all CAVs
                traffic.publish(lanes, absolute_time)

                # call the proper phase on ATC Controller
                if args.run_with_signal_control:
                    #snmp_phase_ctrl(signal.SPaT_sequence[0] + 1, args.intersection)
                    parent.send(("SPaT", signal.SPaT_sequence[0] + 1))

            optimizer_call_ctr += 1

            # Wrap up and log
            if (args.mode == "sim" and (traffic.last_veh_arr() and lanes.all_served(num_lanes))) or \
                    (args.mode == "realtime" and (args.run_duration < elapsed_time)):
                if args.do_logging:
                    # elapsed_process_time = perf_counter() - t_start
                    # timer.log_time_stats(sc, inter_name, start_time_stamp, elapsed_process_time, )  # log timings
                    if args.mode == "sim":
                        traffic.save_veh_level_csv(args.intersection, start_time_stamp_name)
                    if args.mode == "realtime":
                        traffic.close_arrs_deps_csv()
                    traffic.close_trj_csv()
                    signal.close_sig_csv()
                    intersection._inter_config_params.get("print_commandline") and print(
                        "\n### Elapsed Process Time: {:>5d} ms ###".format(int(1_000 * elapsed_time)),
                        "\n### Actual RIO run start time: {:>5d} micro sec. ###".format(int(1_000_000 * t_start)))
                if args.mode == "realtime":
                    tl.stop()
                    tp.stop()
                break
            time_tracker.step()

    except KeyboardInterrupt:
        print("RIO got KeyboardInterrupt, exiting")
        # close

        # Clean up data IO
        if args.mode == "realtime":
            tl.stop()
            tp.stop()

        # Clean up signal control interface
        if args.run_with_signal_control:
            parent.close()
            ps.join()

if __name__ == "__main__":
    parser = argparse.ArgumentParser("Runtime arguments for RIO")
    parser.add_argument("--intersection", type=str, default="RTS",
                        help="The name of the intersection")
    parser.add_argument("--mode", type=str, default="sim",
                        help="Run with traffic from CSV (sim) or sensor fusion (realtime)")
    parser.add_argument("--run-duration", type=int, default=900,
                        help="Seconds to run until termination.")
    parser.add_argument("--loop-freq", type=float, default=1.0,
                        help="Frequency (Hz) to run the main loop")
    parser.add_argument("--solve-freq", type=float, default=1.0,
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
    parser.add_argument("--signal-controller-ip", type=str, default="192.168.91.71",
                        help="The IP address for the signal controller snmp conn")
    parser.add_argument("--signal-controller-port", type=int, default=161,
                        help="The port number for the signal controller snmp conn")

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

    run_rio(args)
    print("\nProgram Terminated.")
