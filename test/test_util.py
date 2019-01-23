import pytest
import context
import time
import src.util as util

def test_euclidean_dist():
    x1 = 0; y1 = 1
    x2 = 0; y2 = 0
    assert util.euclidean_dist(x1,y1,x2,y2) == 1

def test_heading_from_velocity():
    easting_spd = 0
    northing_spd = -5
    assert util.heading_from_velocity([easting_spd, northing_spd]) == 180.
    easting_spd = -2
    northing_spd = 2
    assert util.heading_from_velocity([easting_spd, northing_spd]) == 45.

def test_periodic_sleep():
    period = 0.05
    count = 20
    x = 0
    first_time = True
    start = time.time()
    for i in range(count):
        x += i
        util.periodic_sleep(period)
        end = time.time()
        if not first_time:
            assert abs((end - start) - period) < 2e-3, i  # within 2 ms
        else:
            first_time = False
        start = time.time()
