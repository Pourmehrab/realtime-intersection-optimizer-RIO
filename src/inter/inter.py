####################################
# File name: inter.py              #
# Author: Mahmoud Pourmehrab       #
# Email: mpourmehrab@ufl.edu       #
# Last Modified: Feb/16/2018       #
####################################

import os

import numpy as np

from src.inpt.inpt import read_prms
from src.inter.phs import phenum
from src.inpt.data import *


class Intersection:
    def __init__(self, int_name):
        self.name = int_name
        self._set_lli()

        self._set_pli()

        # d = {"max_speed": '_v', "yellow_duration": '_y', "all_red_duration": '_ar', "opt_range": '_opt_range'}
        # max_speed : m/s, time : sec, distance : meters
        # read_prms(self, int_name, 'int', d)
        self._v, self._y, self._ar, self._opt_range = get_intersection_params(int_name)

    def _set_lli(self):
        '''
            This converts a dictionary of form:
            key is a lane : vals are lanes that are in conflict with key (note numbering starts from 1 not 0)
            to lli which includes the conflict matrix |L|x|L| where element ij is 1 if i and j are conflicting movements

        :return:
        '''
        # gets the conflict dictionary for this intersection
        conf_dict = get_conflict_dict(self.name)

        # number of lanes
        self._nl = len(conf_dict)

        # lane-lane incidence dictionary (lane: set of lanes in conflict with lane)
        self._lli = {l: set([]) for l in range(self._nl)}

        # the whole following loop makes lanes zero-based
        for l, conf in conf_dict.items():
            for j in conf:
                self._lli[l - 1].add(j - 1)  # these are conflicting lanes

    def _set_pli(self):
        pli_one_based = get_phases(self.name)
        if pli_one_based is None:
            pli_one_based = phenum(self._nl, self._lli, self.name)

        self._pli = {p: set([]) for p in range(len(pli_one_based))}

        # the whole following loop makes lanes and phases zero-based
        for l, conf in pli_one_based.items():
            for j in conf:
                self._pli[l - 1].add(j - 1)  # these are lanes that belong to this phase

    def get_phs(self):
        return self._pli

    def get_num_lanes(self):
        return self._nl

    def get_max_speed(self):
        return self._v


class Signal():
    Y = 2  # yellow time sec
    AR = 1  # all-red time sec

    def __init__(self, num_lanes):
        '''
        we start executing from 0 to end of each list

        - sequence keeps the sequence of phases to be executed from 0
        - green_dur keeps the amount of green allocated to each phase
        - yellow and all-red is a fix amount at the end of all phases (look at class variables)
        - start keeps the absolute time (in sec) when each phase starts

        - schedule keeps the earliest departures at the stop bars of each lane and gets updated when a signal decision goes
         permanent. It is made by a dictionary of arrays (key is lane, value is sorted earliest departures).

        '''
        self.sequence, self.green_dur, self.start = [], [], []
        self.schedule = {l: [] for l in range(num_lanes)}

    def enqueue(self, p, g):
        '''
        makes a signal decision permanent (append a phase and its green to the end of signal array)

        :param p: phase to be added
        :param g: green duration of that phase
        :return:
        '''
        self.sequence += [p]
        self.green_dur += [g]
        self.start += [self.start[-1] + self.green_dur[-1] + self.Y + self.AR]

    def dequeue(self):
        del self.sequence[0]
        del self.green_dur[0]
        del self.start[0]
