import numpy as np
import datetime as dt
from datetime import datetime
import time

def meters_to_feet(m):
    """Converts a measurement in meters to feet."""
    return m * 3.2808399

def feet_to_meters(ft):
    """Converts a measurement in feet to meters."""
    return ft * 0.3047999

def euclidean_dist(x1,y1,x2,y2):
    return np.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)

def str_to_bool(any_string):
    if any_string == "True" or any_string == "true":
        return True
    else:
        return False

def heading_from_velocity(vel):
    """
    Given a velocity vector, compute the heading in degrees
    counter-clockwise from true north. The velocity is assumed
    to be a list of floats, where vel[0] is easting spd and
    vel[1] is northing spd (UTM coordinates).
    Units of speed are irrelevant, but will normally be m/s.

    :param vel: the velocity vector
    :type List: [easting_spd, northing_spd]
    """
    vel = [vel[1], vel[0]] # swap 
    # signed angle in radians between ray ending at origin and passing
    # through the point (0,1) and the ray ending at origin and passing
    # throw point (x2,x1)
    t = np.arctan2(*vel) 
    if t < 0:
        t += 2*np.pi
    # convert to angle in radians from true north (up axis)
    t -= np.pi/2
    if t < 0:
        t += 2*np.pi
    return np.rad2deg(t)
    
def periodic_sleep(period):
    """
    Will sleep a thread until the next desired time interval. i.e., 
    if you want a function to be called at intervals of 0.1 seconds.
    This will compute the amount of ms needed to sleep 
    until the next tenth of a second.

    N.b. the first call to periodic_sleep will only sleep the thread for
    fraction of the period needed to wake the thread 
    at the next interval. 

    Example:
    ```
    while True:
        do_func()
        periodic_sleep(0.1)
    ```
    will call do_func() at
        12:00:00.0
        12:00:00.1
        12:00:00.2
        ...
        
    :param period: time between calls (seconds)
    :type float:
    """
    time_dt = dt.timedelta(milliseconds=1000*period)
    timenow = datetime.utcnow()
    t = datetime(timenow.year, timenow.month, timenow.day,
            timenow.hour, timenow.minute, timenow.second, 0)
    splits = [t]
    if period < 1.:
        for _ in range(int(1./period)):
            t = t + time_dt
            splits.append(t)
    else:
        splits.append(t + time_dt)
    # get next largest
    next_time_split = splits[0]
    for sp in splits:
        if timenow < sp:
            next_time_split = sp
            break
    sleep_for = (next_time_split - timenow).total_seconds()
    if sleep_for > 0:
        time.sleep(sleep_for)
