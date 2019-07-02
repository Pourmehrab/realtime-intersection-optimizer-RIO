import sys
import os
import argparse
from src.util import *
import csv
import operator
import matplotlib.pyplot as plt
import numpy as np
from main import run_rio

def log_files(args, min_dist):
    # rename sig_level.csv file, trj_point_level.csv file and trj_veh_level.csv file
    os.rename(str(args.log_dir + "\sig_level.csv"), str(args.log_dir + "\sig_level_" + str(min_dist) + ".csv"))
    os.rename(str(args.log_dir + "\\trj_point_level.csv"), str(args.log_dir + "\\trj_point_level_" + str(min_dist) + ".csv"))
    os.rename(str(args.log_dir + "\\trj_veh_level.csv"), str(args.log_dir + "\\trj_veh_level_" + str(min_dist) + ".csv"))
    #filepath_sig_level = os.path.join(args.log_dir, "sig_level.csv")
    #sig_level_csv_current = open(filepath_sig_level, 'r', newline = '')
    #reader = csv.reader(sig_level_csv_current, delimiter=',')
    #filepath_sig_min_dist = os.path.join(args.log_dir, "sig_level_" + str(min_dist) + ".csv")
    #sig_level_csv_file = open(filepath_sig_min_dist, 'w', newline = '')
    #writer = csv.writer(sig_level_csv_file, delimiter = ',')
    #for row in reader:
     #   writer.writerow(row)

    # copy trj_point_level.csv file
    #filepath_trj_point = os.path.join(args.log_dir, "trj_point_level.csv")
    #trj_point_csv_current = open(filepath_trj_point, 'r', newline='')
    #reader = csv.reader(trj_point_csv_current, delimiter=',')
    #filepath_trj_point_min = os.path.join(args.log_dir, "trj_point_level_" + str(min_dist) + ".csv")
    #trj_point_min_csv = open(filepath_trj_point_min, 'w', newline='')
    #writer = csv.writer(trj_point_min_csv, delimiter=',')
    #for row in reader:
     #   writer.writerow(row)

    # copy trj_veh_level.csv file
    #filepath_trj_veh = os.path.join(args.log_dir, "trj_veh_level.csv")
    #trj_veh_csv_current = open(filepath_trj_veh, 'r', newline='')
    #reader = csv.reader(trj_veh_csv_current, delimiter=',')
    #filepath_trj_veh_min = os.path.join(args.log_dir, "trj_veh_level_" + str(min_dist) + ".csv")
    #trj_veh_csv_min = open(filepath_trj_veh_min, 'w', newline='')
    #writer = csv.writer(trj_veh_csv_min, delimiter=',')
    #for row in reader:
     #   writer.writerow(row)

    # open a file to store individual vehicle delay
    filepath_delay = os.path.join(args.log_dir, 'veh_delay_' + str(min_dist) + '.csv') # open a file to store individual vehicle delay
    veh_delay_csv_file = open(filepath_delay, 'w', newline='')
    writer = csv.writer(veh_delay_csv_file, delimiter=',')
    writer.writerow(["Vehicle #", 'Vehicle Delay'])

    # open a file to store sorted trajectory points
    filepath_sort = os.path.join(args.log_dir, 'trj_point_sort_' + str(min_dist) + '.csv')

def delay_analysis(args, min_dist):
    Total_delay = 0
    trj_veh_level = os.path.join(args.log_dir, 'trj_veh_level_' + str(min_dist) + '.csv')
    veh_delay = os.path.join(args.log_dir, 'veh_delay_' + str(min_dist) + '.csv')
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

    return Total_delay

def executed_trajectory (args, min_dist):
    trj_point_level = os.path.join(args.log_dir, 'trj_point_level_' + str(min_dist) + '.csv')
    trj_point_sort = os.path.join(args.log_dir, 'trj_point_sort_' + str(min_dist) + '.csv')
    signal_csv = os.path.join(args.log_dir, 'sig_level_' + str(min_dist) + '.csv')

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
        'Executed Trajectories for Each Vehicle by Lane, ' + str(min_dist) + ' meters min_distance_to_stop_bar', fontsize = 30)
    place = 1
    for number in lanes:
        ax1 = fig.add_subplot(2, 2, int(place))
        graphs[str(number)] = ax1
        graphs[str(number)].set_title('Lane ' + str(number), fontsize = 20)
        graphs[str(number)].set_xlabel('Time(s)', fontsize = 20)
        graphs[str(number)].set_ylabel('Distance to Stop Bar (m)', fontsize = 20)
        graphs[str(number)].xaxis.set_tick_params(labelsize=10)
        graphs[str(number)].yaxis.set_tick_params(labelsize=10)
        place +=1
    for number in lanes:
        graphs[str(number)].plot(x, y, '--')
        for veh_in_lane in lanes[number]:
            graphs[str(number)].plot(timefl[str(veh_in_lane)], distancefl[str(veh_in_lane)],
                                     label='Vehicle ' + str(veh_in_lane))
            graphs[str(number)].legend(loc='best')

    # phasing, will have to change this depending on intersection phasing
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

    args = parser.parse_args()
    print("Interpreter Information")
    print("Python Path: ", sys.executable)
    print("Python Version: ", sys.version)

    if args.do_logging:
        suffix = "_" + time.strftime("%y%m%d_%H%M%S")
        args.log_dir = os.path.join("log", args.intersection + suffix)
        os.makedirs(args.log_dir)
        print("creating log dir {}...".format(args.log_dir))

min_dist_range = list(range(20,25))

veh_delay_total = os.path.join(args.log_dir, 'veh_delay_total.csv')

if __name__ == '__main__':
    for min_dist in min_dist_range:
        args.min_dist_to_stop_bar = min_dist
        run_rio(args, experiment = True)
        log_files(args, min_dist)
        delay = delay_analysis(args, min_dist)
        with open(veh_delay_total, 'a') as csv_file:
            writer = csv.writer(csv_file, lineterminator='\n')
            writer.writerow([str(min_dist), 'Total travel time delay', str("%.4f" % delay)])
        executed_trajectory(args, min_dist)

print("\nProgram Terminated.")