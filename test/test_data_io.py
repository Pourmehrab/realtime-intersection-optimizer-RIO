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
