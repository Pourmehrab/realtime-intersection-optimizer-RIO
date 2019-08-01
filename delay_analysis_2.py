import csv
import os

# locate vehicle exit speed
def vehicle_exit_speed(args, min_dist):
    trj_point_sort = os.path.join(args.log_dir, 'trj_point_sort_' + str(min_dist) + '.csv')
    vehicle_exit_speed = {}
    with open(trj_point_sort) as current_csv_file:
        sort_reader = csv.reader(current_csv_file, delimiter=',')
        line_count = 0
        for row in sort_reader:
            if line_count == 0:
                line_count += 1
            else:
                if row[1][0] != '1':
                    csv_vehicle = int(row[1][4]) * 10 + int(row[1][5])
                    vehicle_exit_speed[csv_vehicle] = float(row[6])
    return vehicle_exit_speed

# calculate time to reach desired speed
def time_to_reach_desired_speed(vehicle_exit_speed, args, min_dist):
    trj_veh_level = os.path.join(args.log_dir, 'trj_veh_level_' + str(min_dist) + '.csv')
    vehicle_time = {}
    with open(trj_veh_level) as current_csv_file:
        veh_trj_reader = csv.reader(current_csv_file, delimiter=',')
        line_count = 0
        for row in veh_trj_reader:
            if line_count == 0:
                line_count += 1
            else:
                csv_vehicle = int(row[0][2]) * 10 + int(row[0][3])
                vehicle_time[csv_vehicle] = (float(row[13]) - vehicle_exit_speed[csv_vehicle]) / float(row[9])
    return vehicle_time

# calculate distance covered to reach desired speed
def distance_covered(vehicle_time, vehicle_exit_speed, args, min_dist):
    trj_veh_level = os.path.join(args.log_dir, 'trj_veh_level_' + str(min_dist) + '.csv')
    vehicle_distance = {}
    with open(trj_veh_level) as current_csv_file:
        veh_trj_reader = csv.reader(current_csv_file, delimiter=',')
        line_count = 0
        for row in veh_trj_reader:
            if line_count == 0:
                line_count += 1
            else:
                csv_vehicle = int(row[0][2]) * 10 + int(row[0][3])
                vehicle_distance[csv_vehicle] = (vehicle_exit_speed[csv_vehicle] * vehicle_time[csv_vehicle]) + (
                            .5 * float(row[9]) * (vehicle_time[csv_vehicle] ** 2))
    return vehicle_distance

# calculate time required to reach 20 meters
def time_to_20_meters(vehicle_distance, args, min_dist):
    vehicle_dist_to_20 = {}
    for vehicle in vehicle_distance:
        vehicle_dist_to_20[vehicle] = 20 - vehicle_distance[vehicle]
    trj_veh_level = os.path.join(args.log_dir, 'trj_veh_level_' + str(min_dist) + '.csv')
    vehicle_time_to_20 = {}
    with open(trj_veh_level) as current_csv_file:
        veh_trj_reader = csv.reader(current_csv_file, delimiter=',')
        line_count = 0
        for row in veh_trj_reader:
            if line_count == 0:
                line_count += 1
            else:
                csv_vehicle = int(row[0][2]) * 10 + int(row[0][3])
                vehicle_time_to_20[csv_vehicle] = vehicle_dist_to_20[csv_vehicle] / float(row[13])
    return vehicle_time_to_20

# calculate total travel time for 20 meters
def total_20time(vehicle_time_to_20, vehicle_time):
    total_20time = {}
    for vehicle in vehicle_time:
        total_20time[vehicle] = vehicle_time[vehicle] + vehicle_time_to_20[vehicle]
    return total_20time

# calculate ideal travel from detection range to 20 meters past stop bar
def ideal_time(args, min_dist):
    trj_veh_level = os.path.join(args.log_dir, 'trj_veh_level_' + str(min_dist) + '.csv')
    ideal_travel = {}
    with open(trj_veh_level) as current_csv_file:
        veh_trj_reader = csv.reader(current_csv_file, delimiter=',')
        line_count = 0
        for row in veh_trj_reader:
            if line_count == 0:
                line_count += 1
            else:
                csv_vehicle = int(row[0][2]) * 10 + int(row[0][3])
                ideal_travel[csv_vehicle] = (float(row[14]) + 20) / float(row[13])
    return ideal_travel

# calculate total actual travel time
def actual_time(total_time, args, min_dist):
    trj_veh_level = os.path.join(args.log_dir, 'trj_veh_level_' + str(min_dist) + '.csv')
    actual_travel = {}
    with open(trj_veh_level) as current_csv_file:
        veh_trj_reader = csv.reader(current_csv_file, delimiter=',')
        line_count = 0
        for row in veh_trj_reader:
            if line_count == 0:
                line_count += 1
            else:
                csv_vehicle = int(row[0][2]) * 10 + int(row[0][3])
                actual_travel[csv_vehicle] = float(row[18]) - float(row[7]) + total_time[csv_vehicle]
    return actual_travel

# calculate delay
def vehicle_delay(ideal_travel, actual_travel):
    vehicle_delay = {}
    for vehicle in ideal_travel:
        vehicle_delay[vehicle] = actual_travel[vehicle] - ideal_travel[vehicle]
    return vehicle_delay

# log results
def log_delay(vehicle_delay, args, min_dist):
    veh_delay_twenty = os.path.join(args.log_dir, 'veh_delay_twenty' + str(min_dist) + '.csv')
    with open(veh_delay_twenty, 'a') as current_csv_file:
        veh_delay_writer = csv.writer(current_csv_file, lineterminator='\n')
        veh_delay_writer.writerow(["Vehicle #", 'Vehicle Delay (s)'])
    total_delay = 0
    for vehicle in vehicle_delay:
        with open(veh_delay_twenty, 'a') as current_csv_file:
            veh_delay_writer = csv.writer(current_csv_file, lineterminator='\n')
            veh_delay_writer.writerow(["Vehicle " + str(vehicle), str("%.4f" % vehicle_delay[vehicle])])
        total_delay += vehicle_delay[vehicle]
    with open(veh_delay_twenty, 'a') as csv_file:
        with open(veh_delay_twenty, 'a') as current_csv_file:
            writer = csv.writer(current_csv_file, lineterminator='\n')
            writer.writerow(["Total travel time delay", str("%.4f" % total_delay)])
    return total_delay

