# -*- coding: utf-8 -*-
"""
Created on Fri Jul 12 13:07:34 2019

@author: zjerome
"""

# generates traffic and formats it in csv file for RUN_RIO

import numpy as np
import math
import csv
import os

# set variables
vehicleVIN = 1000
sc = 1
flow_code = 1
AV_Ratio = 0
Detection = 92
types = 1
arrival_time = 0
L = 4.8
maxAcc = 2
maxDec = -6
dest = 2
curSpd = 0
desSpd = 8.94
dist = {1: 59, 3: 190, 5: 156, 7: 50}
count = 1
total_count = 0
green_phase = 4
departure_time = 0

filepath = os.path.join('GaleStadium_test.csv') 
traffic_generation = open(filepath, 'w', newline = '')
writer = csv.writer(traffic_generation, delimiter = ',')
writer.writerow(['vehicleVIN', 'sc', 'flow code', 'AV Ratio', 'Detection', 'lane', 'type', 'arrival time', 'L', 'maxAcc', 'maxDec', 'dest', 'curSpd', 'desSpd', 'dist', 'count', 'total count', 'green phase', 'departure time'])
u = 3.5
for key, value in dist.items():
    lane = key
    # set simulation length
    while arrival_time <= 600:
        number = np.random.exponential(scale=1.00, size = None)
        if number < 0.564718122007759:
            t = u * (-math.log(number))
            arrival_time += t
            curSpd = np.random.uniform(low=6.7056, high=11.176, size=None)
            departure_time = arrival_time + dist[lane]/desSpd
            writer.writerow([vehicleVIN, sc, flow_code, AV_Ratio, Detection, lane, types, arrival_time, L, maxAcc, maxDec, dest, curSpd, desSpd, dist[lane], count, total_count, green_phase, departure_time])
            vehicleVIN += 1
    arrival_time = 0