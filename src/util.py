################################################
# File name: util.py
# Authors: Patrick Emami
# Email: pemami@ufl.edu
# Updated: Dec 2018
################################################
import numpy as np
from datetime import datetime
import time

def meters_to_feet(m):
    """Converts a measurement in meters to feet."""
    return m * 3.2808399

def euclidean_dist(x1,y1,x2,y2):
    return np.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)

def str_to_bool(any_string):
    if any_string == "True" or any_string == "true":
        return True
    else:
        return False

def periodic_sleep(period):
    """
    Will sleep a thread until the next period'th of a second. I.e., 
    if you want a function to be called at intervals of 0.1 seconds.
    This will compute the amount to sleep necessary to achieve this rate.

    ```
    while True:
        do_func()
        periodic_sleep(0.1)
    ```

    :param period: time between calls in seconds
    """
    time_dt = datetime.timedelta(milliseconds=1000*period)
    timenow = datetime.utcnow()
    t = datetime(timenow.year, timenow.month, timenow.day,
            timenow.hour, timenow.minute, timenow.second, 0)
    splits = [t]
    for _ in range(int(1./period)):
        t = t + time_dt
        splits.append(t)
    # get next largest
    next_time_split = splits[0]
    for sp in splits:
        if timenow < sp:
            next_time_split = sp
            break
    sleep_for = (next_time_split - timenow).total_seconds()
    if sleep_for > 0:
        time.sleep(sleep_for)