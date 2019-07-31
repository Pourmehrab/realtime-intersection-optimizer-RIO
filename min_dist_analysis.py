import sys
import os
import argparse
from src.util import *
import csv
import operator
import matplotlib.pyplot as plt
import numpy as np
from main import run_rio

def set_up_log_files(args, min_dist):
    """
    :param args: provides path for csv files
    :param min_dist: the current min_dist_too_stop_bar simulated in the experiment
    :return: renames the sig_level.csv, trj_point_level.csv and trj_veh_level.csv files created in run_rio to reflect
    the simulated min_dist value
    """
    os.rename(str(args.log_dir + "\sig_level.csv"), str(args.log_dir + "\sig_level_" + str(min_dist) + ".csv"))
    os.rename(str(args.log_dir + "\\trj_point_level.csv"), str(args.log_dir + "\\trj_point_level_" + str(min_dist) + ".csv"))
    os.rename(str(args.log_dir + "\\trj_veh_level.csv"), str(args.log_dir + "\\trj_veh_level_" + str(min_dist) + ".csv"))

def delay_analysis(args, min_dist):
    """
    :param args: provides path for csv files
    :param min_dist: the current min_dist_too_stop_bar simulated in the experiment
    :return: The total travel time delay experienced at the intersection: the sum of individual vehicle travel delay.
    Creates a csv file, "veh_delay_'min_dis'.csv", that contains individual vehicle delay and total intersection delay.
    """
    Total_delay = 0
    # set variable for trj_veh_level.csv file, which will be read to calculate travel time delay
    trj_veh_level = os.path.join(args.log_dir, 'trj_veh_level_' + str(min_dist) + '.csv')
    # create "veh_delay_'min_dis'.csv" file
    veh_delay = os.path.join(args.log_dir, 'veh_delay_' + str(min_dist) + '.csv')
    with open(trj_veh_level) as current_csv_file:
        veh_trj_reader = csv.reader(current_csv_file, delimiter=',')
        line_count = 0
        for row in veh_trj_reader:
            if line_count == 0:
                # set titles to csv file
                with open(veh_delay, 'a') as current_csv_file:
                    veh_delay_writer = csv.writer(current_csv_file, lineterminator='\n')
                    veh_delay_writer.writerow(["Vehicle #", 'Vehicle Delay (s)'])
                line_count += 1
            else:
                # calculate veh delay (departure time - arrival time(in detection range) - (dist traveled/desSpd)) or
                # (time travelled - ideal time travelled)
                Veh_delay = float(row[18]) - float(row[7]) - float(row[14]) / float(row[13])
                line_count += 1
                # track total delay
                Total_delay = Veh_delay + Total_delay
                # input into csv file
                with open(veh_delay, 'a') as current_csv_file:
                    veh_delay_writer = csv.writer(current_csv_file, lineterminator='\n')
                    veh_delay_writer.writerow(["Vehicle " + str(line_count - 2), str("%.4f" % Veh_delay)])

    # input total travel time delay in csv file
    with open(veh_delay, 'a') as csv_file:
        writer = csv.writer(csv_file, lineterminator='\n')
        writer.writerow(["Total travel time delay", str("%.4f" % Total_delay)])

    return Total_delay

def plot_executed_trajectories (args, min_dist):
    """
    :param args: provides path for csv files
    :param min_dist: the current min_dist_too_stop_bar simulated in the experiment
    :return: trj_point_level csv file sorted by vehicle and then by lane
    plots executed trajectories for each vehicle in their respective lane
    """
    # set variables for paths to needed csv files
    trj_point_level = os.path.join(args.log_dir, 'trj_point_level_' + str(min_dist) + '.csv')
    signal_csv = os.path.join(args.log_dir, 'sig_level_' + str(min_dist) + '.csv')
    # create file for sorted trajectory points
    trj_point_sort = os.path.join(args.log_dir, 'trj_point_sort_' + str(min_dist) + '.csv')
    with open(trj_point_sort, 'a') as current_csv_file:
        writer = csv.writer(current_csv_file, lineterminator='\n')
        writer.writerow(['sc', 'VehID', 'type', 'lane', 'time', 'distance', 'speed'])

    # sort trajectory points in "trj_point_level.csv" file
    open_trj_point = open(trj_point_level, 'r')
    trj_file_reader = csv.reader(open_trj_point, delimiter=',')
    sort_by_vehicle = sorted(trj_file_reader, key=operator.itemgetter(1))
    sort_by_lane = sorted(sort_by_vehicle, key=operator.itemgetter(3))

    # write sorted points to csv file
    for row in sort_by_lane:
        with open(trj_point_sort, 'a') as current_csv_file:
            sort_writer = csv.writer(current_csv_file, lineterminator='\n')
            if row[0] != 'sc':
                sort_writer.writerow(row)
            else:
                # this line is needed so that the code will read the last vehicle in the sorted file.
                sort_writer.writerow([0, '1aaaaa', 0, 0, 0, 0, 0, 0])

    # create dictionary "lanes" which lists vehicles to their respective lanes.
    vehicles = []
    lanes = {}
    with open(trj_point_sort) as current_csv_file:
        sort_reader = csv.reader(current_csv_file, delimiter=',')
        vehicle = -1
        lanenum = 1
        line_count = 0
        for row in sort_reader:
            if line_count == 0:
                line_count += 1
            else:
                if row[1][0] != '1':
                    csv_vehicle = int(row[1][4]) * 10 + int(row[1][5])
                if row[3] == lanenum and csv_vehicle != vehicle:
                    vehicles.append(csv_vehicle)
                    vehicle = csv_vehicle
                elif row[3] == lanenum:
                    vehicle = csv_vehicle
                else:
                    lanes[str(lanenum)] = [int(i) for i in vehicles]
                    lanenum = row[3]
                    vehicles = []

    # create dictionaries that assign distance and time points to each vehicle.
    time = []
    distance = []
    timefl = {}
    distancefl = {}
    maxtime = 0
    with open(trj_point_sort) as current_csv_file:
        sort_reader = csv.reader(current_csv_file, delimiter=',')
        vehicle = 0
        line_count = 0
        for row in sort_reader:
            if line_count == 0:
                line_count += 1
            else:
                if row[1][0] != '1':
                    csv_vehicle = int(row[1][4]) * 10 + int(row[1][5])
                else:
                    csv_vehicle = -1
                if csv_vehicle == vehicle:
                    time.append(row[4])
                    distance.append(row[5])
                    timestamp = round(float(row[4]))
                    if timestamp >= maxtime:
                        # keeps track of total time of the simulation in order to plot min_dist line
                        maxtime = round(float(row[4]))
                else:
                    timefl[str(vehicle)] = [float(i) for i in time]
                    distancefl[str(vehicle)] = [float(i) for i in distance]
                    time = []
                    distance = []
                    vehicle = csv_vehicle
                    time.append(row[4])
                    distance.append(row[5])

    graphs = {}
    # plot min_dist line
    x = np.linspace(0, maxtime)
    y = 0 * x + min_dist
    fig = plt.figure(figsize=(20, 15))
    fig.suptitle(
        'Executed Trajectories for Each Vehicle by Lane, ' + str(min_dist) + ' meters min_distance_to_stop_bar', fontsize = 30)
    place = 1
    # create plots for each lane
    for number in lanes:
        ax1 = fig.add_subplot(2, 2, int(place))
        graphs[str(number)] = ax1
        graphs[str(number)].set_title('Lane ' + str(number), fontsize = 20)
        graphs[str(number)].set_xlabel('Time(s)', fontsize = 20)
        graphs[str(number)].set_ylabel('Distance to Stop Bar (m)', fontsize = 20)
        graphs[str(number)].xaxis.set_tick_params(labelsize=10)
        graphs[str(number)].yaxis.set_tick_params(labelsize=10)
        place +=1
    # assign vehicle trajectories to each plot
    for number in lanes:
        graphs[str(number)].plot(x, y, '--')
        for veh_in_lane in lanes[number]:
            graphs[str(number)].plot(timefl[str(veh_in_lane)], distancefl[str(veh_in_lane)],
                                     label='Vehicle ' + str(veh_in_lane))
            graphs[str(number)].legend(loc='best')

    # plots phasing, will have to change this depending on intersection phasing
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

    # formatting
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
    # save figure to log file
    figpath = os.path.join(args.log_dir, 'min_dist_to_stop_bar_' + str(min_dist))
    fig.savefig(figpath)


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
    parser.add_argument("--save-viz", type=str_to_bool, default="False",
                        help="Save traffic viz plots to the log dir")
    # "True" if you want to calculate delay
    parser.add_argument("--delay-analysis", type=str_to_bool, default="True",
                        help="Display csv files with individual vehicle and total intersection travel time delay")
    # "True if you want to plot executed trajectories
    parser.add_argument("--executed-trajectories-viz", type=str_to_bool, default="True",
                        help="Display the Matplotlib plot of executed trajectories")

    args = parser.parse_args()
    print("Interpreter Information")
    print("Python Path: ", sys.executable)
    print("Python Version: ", sys.version)

    if args.do_logging:
        suffix = "_" + time.strftime("%y%m%d_%H%M%S")
        args.log_dir = os.path.join("log", args.intersection + suffix)
        os.makedirs(args.log_dir)
        print("creating log dir {}...".format(args.log_dir))

#adjuct min_dist_to_stop_bar range here
min_dist_range = list(range(37,38))

# create csv file that keeps track of total travel time delay for each min_dist value
if args.delay_analysis:
    total_delay = os.path.join(args.log_dir, 'total_travel_time_delay.csv')
    with open(total_delay, 'a') as current_csv_file:
        total_delay_writer = csv.writer(current_csv_file, lineterminator='\n')
        total_delay_writer.writerow(["min_dist_to_stop_bar", 'total travel time delay (s)'])

if __name__ == '__main__':
    for min_dist in min_dist_range:
        args.min_dist_to_stop_bar = min_dist
        run_rio(args, experiment = True)
        set_up_log_files(args, min_dist)
        if args.delay_analysis:
            delay = delay_analysis(args, min_dist)
            with open(total_delay, 'a') as csv_file:
                writer = csv.writer(csv_file, lineterminator='\n')
                writer.writerow([str(min_dist), str("%.4f" % delay)])
        if args.executed_trajectories_viz:
            plot_executed_trajectories(args, min_dist)

print("\nProgram Terminated.")