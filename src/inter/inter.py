####################################
# File name: inter.py              #
# Author: Mahmoud Pourmehrab       #
# Email: mpourmehrab@ufl.edu       #
# Last Modified: Feb/17/2018       #
####################################

import os

import numpy as np
import random

from src.inpt.inpt import read_prms
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

    def get_phs(self):
        return self._pli

    def do_spat_decision(self, lanes, num_lanes, allowable_phases):
        '''
        works based on methodology in IEEE paper
        :return:
        '''
        # the goal is to choose the sequence of SPaT which gives more throughput in less time
        self.flush_upcoming_SPaT()

        # keeps index of last vehicle to be served by progressing SPaT
        served_vehicle_indx = np.array([0 if bool(lanes.vehlist[lane]) else -1 for lane in range(num_lanes)],
                                       dtype=np.int)

        # keeps index of last vehicle to be served by progressing SPaT
        last_vehicle_indx = np.array([len(lanes.vehlist[lane]) - 1 for lane in range(num_lanes)], dtype=np.int)

        def all_not_served(a, b):
            for i in range(len(a)):
                if b[i] > -1 and a[i] <= b[i]:
                    return True
            return False

        while all_not_served(served_vehicle_indx, last_vehicle_indx):  # checks if SPaT did not serve all

            best_phase, best_phase_score, best_phase_green_dur = 0, 0, 0
            best_throughput = np.zeros(num_lanes, dtype=np.int)

            for phase in allowable_phases:  # gives all phases a chance

                time_phase_ends, temp_phase_score = 0, 0
                temp_throughput = np.zeros(num_lanes, dtype=np.int)

                # check the length of phase not to exceed the max green
                start_time = self.SPaT_end[-1] if self.SPaT_sequence[-1] != phase else self.SPaT_start[-1]

                for lane in self._pli[phase]:

                    veh_indx = served_vehicle_indx[lane]
                    # check if the lane is not empty and there are vehicles without trj
                    if last_vehicle_indx[lane] > -1 and veh_indx <= last_vehicle_indx[lane]:

                        while veh_indx <= last_vehicle_indx[lane] and lanes.vehlist[lane][
                                veh_indx].earlst - start_time <= self.max_green:
                            # count and time processing new vehicles
                            temp_throughput[lane] += 1
                            if lanes.vehlist[lane][veh_indx].earlst > time_phase_ends:
                                time_phase_ends = lanes.vehlist[lane][veh_indx].earlst

                            veh_indx += 1

                # make it permanent if it's better than previous temporary SPaT
                temp_phase_score = np.sum(temp_throughput) / (time_phase_ends - start_time)

                if temp_phase_score > best_phase_score:
                    best_phase_score, best_phase = temp_phase_score, phase
                    best_phase_green_dur = time_phase_ends - start_time
                    best_throughput = temp_throughput

            if best_phase_score <= 0:  # vehicles are far away, assign a random phase the max green

                remaining_phases = set(allowable_phases) - {self.SPaT_sequence[-1]}
                self.enqueue(remaining_phases.pop(), self.max_green)  # pop gives a random phase which is new

            else:  # progress SPaT

                for lane in self._pli[best_phase]:
                    if best_throughput[lane] > 0:
                        served_vehicle_indx[lane] += best_throughput[lane]
                self.enqueue(best_phase, max(self.min_green, best_phase_green_dur))

    def enqueue(self, p, g):
        '''
        makes a signal decision permanent (append a phase and its green to the end of signal array)
        # SPaT decision is the sequence and green duration of phases

        :param p: phase to be added
        :param g: green duration of that phase
        :return:
        '''
        if self.SPaT_sequence[-1] == p:  # extend this phase
            self.SPaT_end[-1] = [self.SPaT_start[-1] + g + self._y + self._ar]
        else:  # append a new phase
            self.SPaT_sequence += [p]
            self.SPaT_green_dur += [g]
            self.SPaT_start += [self.SPaT_end[-1]]
            self.SPaT_end += [self.SPaT_start[-1] + g + self._y + self._ar]

    def dequeue(self):
        del self.SPaT_sequence[0]
        del self.SPaT_green_dur[0]
        del self.SPaT_start[0]
        del self.SPaT_end[0]

    def flush_upcoming_SPaT(self):
        if len(self.SPaT_sequence) > 1:
            self.SPaT_sequence = [self.SPaT_sequence[0]]
            self.SPaT_green_dur = [self.SPaT_green_dur[0]]
            self.SPaT_start = [self.SPaT_start[0]]
            self.SPaT_end = [self.SPaT_end[0]]
