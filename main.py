#!/usr/bin/python3
import sys
import os
import argparse
import csv
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
from src.visualizer import plot_SPaT_and_trajs
from datetime import datetime
from multiprocessing import Process, Pipe


def run_rio(args, min_dist):
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

    intersection = Intersection(args.intersection, min_dist)
    lanes = Lanes(intersection)
    start_time_stamp_name = datetime.now().strftime("%m-%d-%Y_%H-%M-%S")  # only for naming the CSV files
    if args.mode == "sim":
        traffic = SimTraffic(intersection, args.sc, start_time_stamp_name, args, min_dist)
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
    signal = MCF_SPaT(args, 0, intersection, args.sc, start_time_stamp_name, min_dist)
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

            # add/update the vehicles
            if args.mode == "sim":
                # update the assigned trajectories
                traffic.update_trj_or_serve_at_stop_bar(lanes, elapsed_time, intersection)
                traffic.get_traffic_info(lanes, elapsed_time, intersection)
            else:
                traffic.update_trj_or_serve_at_stop_bar(lanes, elapsed_time, intersection,
                    time_tracker.start_time)
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
                
                if args.show_viz or args.save_viz:
                    if args.save_viz:
                        save_loc = os.path.join(args.log_dir, 'imgs')
                        if not os.path.exists(save_loc):
                            os.makedirs(save_loc)
                    else:
                        save_loc = ''
                    plot_SPaT_and_trajs(lanes, signal, intersection, elapsed_time, args.show_viz, save_loc)

            optimizer_call_ctr += 1

            # Wrap up and log
            if (args.mode == "sim" and (traffic.last_veh_arr() and lanes.all_served(num_lanes))) or \
                    (args.mode == "realtime" and (args.run_duration < elapsed_time)):
                if args.do_logging:
                    if args.mode == "sim":
                        traffic.save_veh_level_csv(args.intersection, start_time_stamp_name)
                    if args.mode == "realtime":
                        traffic.close_arrs_deps_csv()
                    traffic.close_trj_csv()
                    signal.log_current_SPaT(args.sc, absolute_time)
                    signal.close_sig_csv()
                    intersection._inter_config_params.get("print_commandline") and print(
                        "\n### Elapsed Process Time: {:.3f} s ###".format(elapsed_time))
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

def delay_analysis(min_dist):
    Total_delay = 0
    trj_veh_level = os.path.join('log', args.intersection + suffix, 'trj_veh_level_' + str(min_dist) + '.csv')
    trj_point_level = os.path.join('log', args.intersection + suffix, 'trj_point_level_' + str(min_dist) + '.csv')
    veh_delay = os.path.join('log', args.intersection + suffix, 'veh_delay_' + str(min_dist) + '.csv')
    trj_point_sort = os.path.join('log', args.intersection + suffix, 'trj_point_sort_' + str(min_dist) + '.csv')
    signal_csv = os.path.join('log', args.intersection + suffix, 'sig_level_' + str(min_dist) + '.csv')
    with open(trj_veh_level) as csv_file:
        csv_reader = csv.reader(csv_file, delimiter=',')
        line_count = 0
        for row in csv_reader:
            if line_count == 0:
                line_count += 1
            else:
                Veh_delay = float(row[18]) - float(row[7]) - float(row[14]) / float(row[13])
                line_count += 1
                Total_delay = Veh_delay + Total_delay
                with open(veh_delay, 'a') as csv_file:
                    writer = csv.writer(csv_file, lineterminator='\n')
                    writer.writerow(["Vehicle " + str(line_count - 2), str("%.4f" % Veh_delay)])

    with open(veh_delay, 'a') as csv_file:
        writer = csv.writer(csv_file, lineterminator='\n')
        writer.writerow(["Total travel time delay", str("%.4f" % Total_delay)])

    import operator
    import matplotlib.pyplot as plt
    import numpy as np

    sample = open(trj_point_level, 'r')

    csv1 = csv.reader(sample, delimiter=',')

    sort = sorted(csv1, key=operator.itemgetter(1))
    sort2 = sorted(sort, key=operator.itemgetter(3))

    for row in sort2:
        with open(trj_point_sort, 'a') as csv_file:
            writer = csv.writer(csv_file, lineterminator='\n')
            if row[0] != 'sc':
                writer.writerow(row)
            else:
                writer.writerow([0, 'aaaaaa', 0, 0, 0, 0, 0, 0])

    vehicles = []
    lanes = {}
    with open(trj_point_sort) as csv_file:
        csv_reader = csv.reader(csv_file, delimiter=',')
        vehicle = -1
        lanenum = 1
        for row in csv_reader:
            if row[3] == lanenum and row[1][5] != vehicle:
                vehicles.append(row[1][5])
                vehicle = row[1][5]
            elif row[3] == lanenum:
                vehicle = row[1][5]
            else:
                lanes[str(lanenum)] = [int(i) for i in vehicles]
                lanenum = row[3]
                vehicles = []

    time = []
    distance = []
    timefl = {}
    distancefl = {}
    maxtime = 0
    with open(trj_point_sort) as csv_file:
        csv_reader = csv.reader(csv_file, delimiter=',')
        vehicle = 0
        for row in csv_reader:
            if row[1][5] == vehicle:
                time.append(row[4])
                distance.append(row[5])
                timestamp = round(float(row[4]))
                if timestamp >= maxtime:
                    maxtime = round(float(row[4]))
            else:
                timefl[str(vehicle)] = [float(i) for i in time]
                distancefl[str(vehicle)] = [float(i) for i in distance]
                time = []
                distance = []
                vehicle = row[1][5]
                time.append(row[4])
                distance.append(row[5])

    graphs = {}
    x = np.linspace(0, maxtime)
    y = 0 * x + min_dist
    fig = plt.figure(figsize=(20, 15))
    fig.suptitle(
        'Executed Trajectories for Each Vehicle by Lane, ' + str(min_dist) + ' meters min_distance_to_stop_bar')
    place = 1
    for number in lanes:
        ax1 = fig.add_subplot(2, 2, int(place))
        graphs[str(number)] = ax1
        graphs[str(number)].set_title('Lane ' + str(number))
        graphs[str(number)].set_xlabel('Time(s)')
        graphs[str(number)].set_ylabel('Distance to Stop Bar (m)')
        place +=1
    for number in lanes:
        graphs[str(number)].plot(x, y, '--')
        for veh_in_lane in lanes[number]:
            graphs[str(number)].plot(timefl[str(veh_in_lane)], distancefl[str(veh_in_lane)],
                                     label='Vehicle ' + str(veh_in_lane))
            graphs[str(number)].legend(loc='best')

    # phasing, will have to change this depending on intersection
    with open(signal_csv) as csv_file:
        csv_reader = csv.reader(csv_file, delimiter=',')
        line_count = 0
        for row in csv_reader:
            if line_count == 0:
                line_count += 1
            elif int(row[1]) == 0:
                graphs['1'].plot([float(row[2]), float(row[3])], [0, 0], color='red', linewidth=4)
                graphs['3'].plot([float(row[2]), float(row[3])], [0, 0], color='red', linewidth=4)
                graphs['5'].plot([float(row[2]), float(row[3])], [0, 0], color='green', linewidth=4)
                graphs['7'].plot([float(row[2]), float(row[3])], [0, 0], color='red', linewidth=4)
            elif int(row[1]) == 1:
                graphs['5'].plot([float(row[2]), float(row[3])], [0, 0], color='red', linewidth=4)
                graphs['7'].plot([float(row[2]), float(row[3])], [0, 0], color='red', linewidth=4)
                graphs['1'].plot([float(row[2]), float(row[3])], [0, 0], color='green', linewidth=4)
                graphs['3'].plot([float(row[2]), float(row[3])], [0, 0], color='red', linewidth=4)
            elif int(row[1]) == 2:
                graphs['5'].plot([float(row[2]), float(row[3])], [0, 0], color='red', linewidth=4)
                graphs['7'].plot([float(row[2]), float(row[3])], [0, 0], color='green', linewidth=4)
                graphs['1'].plot([float(row[2]), float(row[3])], [0, 0], color='red', linewidth=4)
                graphs['3'].plot([float(row[2]), float(row[3])], [0, 0], color='red', linewidth=4)
            else:
                graphs['5'].plot([float(row[2]), float(row[3])], [0, 0], color='red', linewidth=4)
                graphs['7'].plot([float(row[2]), float(row[3])], [0, 0], color='red', linewidth=4)
                graphs['1'].plot([float(row[2]), float(row[3])], [0, 0], color='red', linewidth=4)
                graphs['3'].plot([float(row[2]), float(row[3])], [0, 0], color='green', linewidth=4)

    with open(signal_csv) as csv_file:
        csv_reader = csv.reader(csv_file, delimiter=',')
        line_count = 0
        xticks = []
        for row in csv_reader:
            if line_count == 0:
                line_count += 1
            else:
                xticks.append(float(row[3]))
        for number in lanes:
            graphs[str(number)].set_xticks(xticks)
    os.path.join('log', args.intersection + suffix, 'veh_delay')
    # for fig in figs:
    # figs[str(fig)].savefig('Lane ' + str(fig))
    figpath = os.path.join('log', args.intersection + suffix, 'min_dist_to_stop_bar_' + str(min_dist))
    fig.savefig(figpath)
    return Total_delay


if __name__ == "__main__":
    parser = argparse.ArgumentParser("Runtime arguments for RIO")
    parser.add_argument("--intersection", type=str, default="GaleStadium",
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

    parser.add_argument("--do-logging", type=str_to_bool, default="True",
                        help="Toggle logging")
    parser.add_argument("--run-with-signal-control", type=str_to_bool, default="False")
    parser.add_argument("--show-viz", type=str_to_bool, default="False",
                        help="Display the Matplotlib plot of traffic after each solve call")
    parser.add_argument("--save-viz", type=str_to_bool, default="True",
                        help="Save traffic viz plots to the log dir")

    args = parser.parse_args()
    print("Interpreter Information")
    print("Python Path: ", sys.executable)
    print("Python Version: ", sys.version)

    if args.do_logging:
        suffix = "_" + time.strftime("%y%m%d_%H%M%S")
        args.log_dir = os.path.join("log", args.intersection + suffix)
        os.makedirs(args.log_dir)
        print("creating log dir {}...".format(args.log_dir))

    veh_delay_total = os.path.join('log', args.intersection + suffix, 'veh_delay_total.csv')
    for min_dist in range(20, 21):
        run_rio(args, min_dist)
        delay = delay_analysis(min_dist)
        with open(veh_delay_total, 'a') as csv_file:
            writer = csv.writer(csv_file, lineterminator='\n')
            writer.writerow([str(min_dist), 'Total travel time delay', str("%.4f" % delay)])

    print("\nProgram Terminated.")
