import pytest
import context
import os
import src.config as config

def test_load_optimization_zone_constraints():
    inter_name = "RTS"
    opt_zone_info = config.load_optimization_zone_constraints(inter_name)
    orientations = [317,137,220,47.5]
    assert opt_zone_info[1].orientation == orientations[0]
    assert opt_zone_info[2].orientation == orientations[1]
    assert opt_zone_info[3].orientation == orientations[2]
    assert opt_zone_info[4].orientation == orientations[3]
    assert len(opt_zone_info[1].easting) == 4
    assert len(opt_zone_info[1].northing) == 4

def test_load_lane_geom():
    inter_name = "RTS"
    lane_info = config.load_lane_geom(inter_name)
    assert len(lane_info[1].distances) == len(lane_info[1].easting)
    assert len(lane_info[1].distances) == len(lane_info[1].northing)
