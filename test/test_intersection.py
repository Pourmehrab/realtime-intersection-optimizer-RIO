import pytest
import context
import utm
import numpy as np
from src.intersection import Intersection

def test_GPS_lane_detection():
    inter = Intersection("RTS")
    lat, lon = 29.6417314, -82.3221626
    easting, northing, utmzone, utmletter = utm.from_latlon(lat, lon)
    class Msg:
        pos = [easting, northing]
    veh = Msg()
    assert inter.detect_lane(veh) == 1

def test_LLA_and_UTM_to_distance_from_stopbar():
    inter = Intersection("RTS")
    dist = 27 # meters
    lat, lon = 29.641557, -82.322354
    lane = 1
    assert dist - 0.5 <= inter.LLA_to_distance_from_stopbar(lat, lon, lane) <= dist + 0.5

    dist = 53.1
    lat, lon = 29.642181, -82.321674
    lane = 2
    assert dist - 0.5 <= inter.LLA_to_distance_from_stopbar(lat, lon, lane) <= dist + 0.5

def test_distance_from_stopbar_to_LLA():
    inter = Intersection("RTS")
    distance = 30.5 # meters
    lane = 2
    lat, lon = inter.distance_from_stopbar_to_LLA(distance, lane)
    gt_lat, gt_lon = 29.642039, -82.321835
    assert np.isclose(lat, gt_lat, rtol=1e-3)
    assert np.isclose(lon, gt_lon, rtol=1e-3) 