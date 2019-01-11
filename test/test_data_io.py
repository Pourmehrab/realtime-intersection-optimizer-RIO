import pytest
import context
import src.data_io as data_io
from src.intersection import Vehicle
import pickle
import os
from datetime import datetime

TEST_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "data")

def test_publisher_veh_to_IAM():
    inter = pickle.load(open(os.path.join(TEST_DIR, "RTS_intersection.pkl"), "rb"))
    veh = pickle.load(open(os.path.join(TEST_DIR, "test_vehicle.pkl"), "rb" ))
    veh.ID = veh.ID + ":234"
    tp = data_io.TrafficPublisher(inter, "localhost", 4200)
    IAM = tp.veh_to_IAM(veh, lane=1, timestamp=datetime.utcnow())
    tp.close()

def test_listener_handle():
    listener = data_io.TrafficListener("localhost", 4200)
    data_queue = listener.get_vehicle_data_queue()
    with open(os.path.join(TEST_DIR, "test_vehicle_data_msg.txt"), "r") as f:
        msg = f.readline()
        listener.handle(msg)

        parsed_msg = data_queue.pop()
        assert parsed_msg
