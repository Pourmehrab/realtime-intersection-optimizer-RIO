####################################
# File name: inter.py              #
# Author: Mahmoud Pourmehrab       #
# Email: mpourmehrab@ufl.edu       #
# Last Modified: Mar/11/2018       #
####################################

import os

import numpy as np
from src.inter.phs import phenum
from src.inpt.data import *


class Intersection:
    def __init__(self, int_name):
        self.name = int_name

        # d = {"max_speed": '_v', "yellow_duration": '_y', "all_red_duration": '_ar', "opt_range": '_opt_range'}
        # max_speed : m/s, time : sec, distance : meters
        # read_prms(self, int_name, 'int', d)
        self._v, self._nl, self._opt_range = get_general_params(int_name)

    def get_num_lanes(self):
        return self._nl

    def get_max_speed(self):
        return self._v


class Signal:
    SAT = 1  # saturation headway (also in trj.py)

    def __init__(self, int_name, allowable_phases):
        '''
        - sequence keeps the sequence of phases to be executed from 0
        - green_dur keeps the amount of green allocated to each phase
        - yellow and all-red is a fix amount at the end of all phases (look at class variables)
        - start keeps the absolute time (in sec) when each phase starts

        * SPaT starts executing from 0 to end of each list

        - schedule keeps the earliest departures at the stop bars of each lane and gets updated when a signal decision
         goes permanent. It is made by a dictionary of arrays (key is lane, value is sorted earliest departures).

        '''
        self.name = int_name

        self._y, self._ar, self.min_green, self.max_green = get_signal_params(int_name)

        self._set_lli()

        self._set_pli()

        self.SPaT_sequence, self.SPaT_green_dur, self.SPaT_start, self.SPaT_end = [set(allowable_phases).pop()], [0], \
                                                                                  [0], [self._y + self._ar]

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

    def lane_in_phase(self, lane, phase_indx):
        if lane in self._pli[phase_indx]:
            return True
        else:
            return False

    def get_next_phase_indx(self, phase_indx, lane):
        new_phase_index = phase_indx + 1
        while new_phase_index < len(self.SPaT_sequence):
            if self.lane_in_phase(lane, new_phase_index):
                return new_phase_index
            else:
                new_phase_index += 1
        return -1  # this means no next phase serves this lane

    def GA_on_SPaT(self, lanes, num_lanes, allowable_phases, pop_size=20, max_iterations=10):
        '''
        Genetic Algorithms to do SPaT decision
        :param lanes:
        :param num_lanes:
        :param allowable_phases:
        :param pop_size: per loop
        :param max_iterations: per loop
        :return:
        '''
        pass

    def set_optimal_cycle_length(self):
        pass

    def get_optimal_cycle_length(self):
        pass

    def enqueue(self, p, g):
        '''
        makes a signal decision permanent (append a phase and its green to the end of signal array)
        # SPaT decision is the sequence and green duration of phases

        :param p: phase to be added
        :param g: green duration of that phase
        :return:
        '''
        if self.SPaT_sequence[-1] == p:  # extend this phase
            self.SPaT_end[-1] = self.SPaT_start[-1] + g + self._y + self._ar
            print('*** Phase {:d} Extended (ends @ {:2.2f} sec)'.format(self.SPaT_sequence[-1], self.SPaT_end[-1]))
        else:  # append a new phase
            self.SPaT_sequence += [p]
            self.SPaT_green_dur += [g]
            self.SPaT_start += [self.SPaT_end[-1]]
            self.SPaT_end += [self.SPaT_start[-1] + g + self._y + self._ar]
            print('*** Phase {:d} Appended (ends @ {:2.2f} sec)'.format(p, self.SPaT_end[-1]))

    def flush_upcoming_SPaT(self):

        if len(self.SPaT_sequence) > 1:
            self.SPaT_sequence = [self.SPaT_sequence[0]]
            self.SPaT_green_dur = [self.SPaT_green_dur[0]]
            self.SPaT_start = [self.SPaT_start[0]]
            self.SPaT_end = [self.SPaT_end[0]]

        print('*** SPaT Flushed')

    def update_STaT(self, t):
        while len(self.SPaT_end) > 1 and t >= self.SPaT_end[0]:
            print('*** Phase {:d} Purged'.format(self.SPaT_sequence[0]))
            del self.SPaT_sequence[0]
            del self.SPaT_green_dur[0]
            del self.SPaT_start[0]
            del self.SPaT_end[0]
