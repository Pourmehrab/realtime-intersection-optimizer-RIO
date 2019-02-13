import pytest
import context
import utm
import numpy as np
from src.intersection import Intersection
from src.data_io import VehicleMsg

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
    
    # Test small negative value
    distance = -0.1 # meters
    lane = 1
    lat, lon = inter.distance_from_stopbar_to_LLA(distance, lane)
    # GPS of the stopbar for Lane 1
    gt_lat, gt_lon = 29.6417314,  -82.3221626
    assert np.isclose(lat, gt_lat, rtol=1e-3)
    assert np.isclose(lon, gt_lon, rtol=1e-3)

def test_optimization_zone():
    def generate_vehicle_msg(pos, vel):
        return VehicleMsg(
                timestamp="",
                track_id="",
                dsrc_id="",
                pos=pos,
                vel=vel,
                pos_rms=[],
                vel_rms=[],
                veh_type=0,
                veh_len=4,
                max_accel=1,
                max_decel=1,
                road="NB")
    # lane 2 heading is 137' CCW from true north
    inter = Intersection("RTS")
    # test vehicle in opt zone
    lat, lon = 29.642138, -82.321726
    heading = 92.1 # ' from true north
    speed = 7 # m/s
    vel = [np.cos(np.deg2rad(heading + 90)) * speed, np.sin(np.deg2rad(heading + 90)) * speed]
    easting, northing, _, _ = utm.from_latlon(lat, lon)
    vm = generate_vehicle_msg([easting, northing], vel)
    assert inter.in_optimization_zone(vm, lane=2)
    
    # test vehicle with correct heading but not
    # spatially located in opt zone
    lat, lon = 29.642481, -82.321375
    easting, northing, _, _ = utm.from_latlon(lat, lon)
    vm = generate_vehicle_msg([easting, northing], vel)
    assert not inter.in_optimization_zone(vm, lane=2)

    # test vehicle with incorrect heading but 
    # spatially located in opt zone
    lat, lon = 29.642138, -82.321726
    heading = 91 # ' from true north
    speed = 7 # m/s
    vel = [np.cos(np.deg2rad(heading + 90)) * speed, np.sin(np.deg2rad(heading + 90)) * speed]
    easting, northing, _, _ = utm.from_latlon(lat, lon)
    vm = generate_vehicle_msg([easting, northing], vel)
    assert not inter.in_optimization_zone(vm, lane=2)
    
    # test small positive heading
    lat, lon = 29.641540, -82.321824
    heading = 5 # ' from true north
    speed = 7 # m/s
    vel = [np.cos(np.deg2rad(heading + 90)) * speed, np.sin(np.deg2rad(heading + 90)) * speed]
    easting, northing, _, _ = utm.from_latlon(lat, lon)
    vm = generate_vehicle_msg([easting, northing], vel)
    assert inter.in_optimization_zone(vm, lane=4)

