import pytest
import context
import src.data_io as data_io
from src.intersection import Lanes
from src.traffic import RealTimeTraffic
from src.time_tracker import Timer
import pickle
import os
from datetime import datetime
from collections import deque

TEST_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "data")

class Args:
    do_logging = False
    sc = 1

def test_get_traffic_info_and_vehicle_update():
    listener = data_io.TrafficListener("localhost", 4200)
    data_queue = listener.get_vehicle_data_queue()
    tsm_queue = listener.get_track_split_merge_queue()
    cav_traj_queue = deque(maxlen=1)  # mock this one
    inter = pickle.load(open(os.path.join(TEST_DIR, "RTS_intersection.pkl"), "rb"))
    lanes = Lanes(inter)
    time_tracker = Timer.get_timer("realtime", 0.1)

    rt_traffic = RealTimeTraffic(data_queue, tsm_queue, cav_traj_queue, inter, Args())
    
    # new vehicle, in zone
    with open(os.path.join(TEST_DIR, "test_vehicle_data_RTS_lane_1_in_zone.txt"), "r") as f:
        msg = f.readline()
        listener.handle(msg)
        rt_traffic.get_traffic_info(lanes, time_tracker)
        assert lanes.find_and_return_vehicle_by_id(1, "0:6234496") is not None

    # remove vehicle from lane
    lanes.remove_srv_vehs(1, 0)

    # new vehicle, not in zone
    with open(os.path.join(TEST_DIR, "test_vehicle_data_RTS_lane_1_no_zone.txt"), "r") as f:
        msg = f.readline()
        listener.handle(msg)
        rt_traffic.get_traffic_info(lanes, time_tracker)
        assert lanes.find_and_return_vehicle_by_id(1, "0:6234496") is None
    
    # old vehicle, in zone
    with open(os.path.join(TEST_DIR, "test_vehicle_data_RTS_lane_1_in_zone.txt"), "r") as f:
        msg = f.readline()
        listener.handle(msg)
        rt_traffic.get_traffic_info(lanes, time_tracker)
        v1 = lanes.find_and_return_vehicle_by_id(1, "0:6234496")
        v1_dist = float(v1.trajectory[1,0])
        with open(os.path.join(TEST_DIR, "test_vehicle_data_RTS_lane_1_in_zone_update.txt"), "r") as f:
            msg = f.readline()
            listener.handle(msg)
            rt_traffic.get_traffic_info(lanes, time_tracker)
            v2 = lanes.find_and_return_vehicle_by_id(1, "0:6234496")
            assert v2.trajectory[1,0] < v1_dist # distance to stopbar has decreasd

def test_serve_update_at_stopbar():
    listener = data_io.TrafficListener("localhost", 4200)
    data_queue = listener.get_vehicle_data_queue()
    tsm_queue = listener.get_track_split_merge_queue()
    cav_traj_queue = deque(maxlen=1)  # mock this one
    inter = pickle.load(open(os.path.join(TEST_DIR, "RTS_intersection.pkl"), "rb"))
    lanes = Lanes(inter)
    time_tracker = Timer.get_timer("realtime", 0.1)

    rt_traffic = RealTimeTraffic(data_queue, tsm_queue, cav_traj_queue, inter, Args())
    
    # new vehicle, in zone
    with open(os.path.join(TEST_DIR, "test_vehicle_data_RTS_lane_1_in_zone.txt"), "r") as f:
        msg = f.readline()
        listener.handle(msg)
        rt_traffic.get_traffic_info(lanes, time_tracker)
    
    assert lanes.find_and_return_vehicle_by_id(1, "0:6234496") is not None

    # update is pass the stop bar
    with open(os.path.join(TEST_DIR, "test_vehicle_data_RTS_lane_1_cross_stopbar.txt"), "r") as f:
        msg = f.readline()
        listener.handle(msg)
        rt_traffic.get_traffic_info(lanes, time_tracker)

    rt_traffic.serve_update_at_stop_bar(lanes, None, inter)
    assert lanes.find_and_return_vehicle_by_id(1, "0:6234496") is None

def test_traffic_publish():
    listener = data_io.TrafficListener("localhost", 4200)
    data_queue = listener.get_vehicle_data_queue()
    tsm_queue = listener.get_track_split_merge_queue()
    cav_traj_queue = deque(maxlen=1)  # mock this one
    inter = pickle.load(open(os.path.join(TEST_DIR, "RTS_intersection.pkl"), "rb"))
    test_vehicle = pickle.load(open(os.path.join(TEST_DIR, "test_vehicle.pkl"), "rb"))
    lanes = Lanes(inter)
    time_tracker = Timer.get_timer("realtime", 0.1)
    rt_traffic = RealTimeTraffic(data_queue, tsm_queue, cav_traj_queue, inter, Args())

    test_vehicle.got_trajectory = True
    test_vehicle.ID = "0:12340"
    lanes.vehlist[1] += [test_vehicle]
    lanes.inc_last_veh_pos(1)

    _, curr_time = time_tracker.get_time()
    rt_traffic.publish(lanes, curr_time)

    next_veh, lane, timestamp = cav_traj_queue.pop()
    assert next_veh.ID == test_vehicle.ID
