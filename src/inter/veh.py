'''
By:     Mahmoud Pourmehrab
E-mail: mpourmehrab@ufl.edu
Date:        Dec 2017
Last update: Dec/08/2017
'''

import numpy as np


# class _DoublyLinkedBase:
#     class _Node:
#         __slots__ = '_element', '_prev', '_next'
#
#         def __init__(self, element, prev, next):
#             self._element = element
#             self._prev = prev
#             self._next = next
#
#     def __init__(self):
#         self._header = self._Node(None, None, None)
#         self._trailer = self._Node(None, None, None)
#         self._header._next = self._trailer
#         self._trailer._prev = self._header
#         self._size = 0
#
#     def __len__(self):
#         return self._size
#
#     def is_empty(self):
#         return self._size == 0
#
#     def _insert_between(self, e, predecessor, successor):
#         newest = self._Node(e, predecessor, successor)
#         predecessor._next = newest
#         successor._prev = newest
#         self._size += 1
#         return newest
#
#     def _delete_node(self, node):
#         predecessor = node._prev
#         successor = node._next
#         predecessor._next = successor
#         successor._prev = predecessor
#         self._size -= 1
#         element = node._element
#         node._prev = node._next = node._element = None
#         return element
#
#
# # class LinkedDeque(_DoublyLinkedBase):
# #     def first(self):
# #         if self.is_empty():
# #             raise Empty('Deque is empty')
# #         return self._header._next._element
# #
# #     def last(self):
# #         if self.is_empty():
# #             raise Empty('Deque is empty')
# #         return self._trailer._prev._element
# #
# #     def insert_first(self, e):
# #         self._insert_between(e, self._header, self._header._next)
# #
# #     def insert_last(self, e):
# #         self._insert_between(e, self._trailer._prev, self._trailer)
# #
# #     def delete_first(self):
# #         if self.is_empty():
# #             raise Empty('Deque is empty')
# #         return self._delete_node(self._header._next)
# #
# #     def delete_last(self):
# #         if self.is_empty():
# #             raise Empty('Deque is empty')
# #         return self._delete_node(self._trailer._prev)
#
#
# class VehiclesList(_DoublyLinkedBase):
#     '''
#     A doubly linked list to keep vehicles in a lane
#     '''
#
#     # -------------------------- nested Position class --------------------------
#     class Position:
#         def __init__(self, container, node):
#             self._container = container
#             self._node = node
#
#         def element(self):
#             return self._node._element
#
#         def __eq__(self, other):
#             return type(other) is type(self) and other._node is self._node
#
#         def __ne__(self, other):
#             return not (self == other)
#
#     # ------------------------------- utility method -------------------------------
#     def _validate(self, p):
#         if not isinstance(p, self.Position):
#             raise TypeError('p must be proper Position type')
#         if p._container is not self:
#             raise ValueError('p is no longer valid')
#         if p._node._next is None:
#             raise Exception('P is no longer valid')
#         return p._node
#
#     def _make_position(self, node):
#         if node is self._header or node is self._trailer:
#             return None
#         else:
#             return self.Position(self, node)
#
#     # ------------------------------- accessors -------------------------------
#     def first(self):
#         return self._make_position(self._header._next)
#
#     def last(self):
#         return self._make_position(self._trailer._prev)
#
#     def before(self, p):
#         node = self._validate(p)
#         return self._make_position(node._prev)
#
#     def after(self, p):
#         node = self._validate(p)
#         return self._make_position(node._next)
#
#     def __iter__(self):
#         cursor = self.first()
#         while cursor is not None:
#             yield cursor.element()
#             cursor = self.after(cursor)
#
#     # ------------------------------- mutators -------------------------------
#     def _insert_between(self, e, predecessor, successor):
#         node = super()._insert_between(e, predecessor, successor)
#         return self._make_position(node)
#
#     def add_first(self, e):
#         return self._insert_between(e, self._header, self._header._next)
#
#     def add_last(self, e):
#         return self._insert_between(e, self._trailer._prev, self._trailer)
#
#     def add_before(self, p, e):
#         original = self._validate(p)
#         return self._insert_between(e, original._prev, original)
#
#     def add_after(self, p, e):
#         original = self._validate(p)
#         return self._insert_between(e, original, original._next)
#
#     def delete(self, p):
#         original = self._validate(p)
#         return self._delete_node(original)
#
#     def replace(self, p, e):
#         original = self._validate(p)
#         old_value = original._element
#         original._element = e
#         return old_value


class Lanes:
    def __init__(self, num_lanes):
        '''
        Data Structure for a dictionary of lanes

        :param num_lanes: number of lanes
        '''
        # todo:(Mahmoud) a lane can have speed limit, opt range and other some other attributes
        # a dictionary of array
        self.vehlist = {l: [] for l in range(num_lanes)}
        self.last_w_trj = [-1 for l in range(num_lanes)]  # this keeps the indx that all up to that vehicle have trj
        # (initialized by -1 meaning no vehicle in this lane has got trajectory)

    def increase_indx(self, lane):
        self.last_w_trj[lane] += 1

    def decrease_indx(self, lane):
        self.last_w_trj[lane] -= 1


class Vehicle:
    def __init__(self, det_id, det_type, det_time, speed, dist, des_speed, dest, length, amin, amax, indx,
                 max_num_trajectory_points=100):
        '''
        Data Structure for an individual vehicle

        :param det_id:          the id assigned to this vehicle by radio
        :param det_type:        0: Conventional, 1: Connected, 2: Automated
        :param det_time:        detection time in seconds from reference time
        :param speed:           detection speed in m/s
        :param dist:            detection distance to stop bar in meter
        :param length:          length of vehicle in meter
        :param amin:            desirable deceleration rate in m/s2
        :param amax:            desired acceleration rate in m/s2
        :param dest:            destination 0: right turn, 1: through, 2: left
        :param des_speed:       desired speed in m/s
        :param trajectory       keeps trajectory of vehicle [time, distance, speed]
        :param last_trj_point_indx       last index points to last row in trajectory
        '''
        self.ID = det_id
        self.veh_type = det_type
        self.init_time = det_time
        self.curr_speed = speed
        self.distance = dist
        self.length = length
        self.max_accel_rate = amin
        self.max_decel_rate = amax
        self.destination = dest
        self.desired_speed = des_speed
        self.trajectory = np.zeros((max_num_trajectory_points, 3))
        self.first_trj_point_indx = 0
        self.last_trj_point_indx = 0
        # time_diff = 3600 * (det_time[0] - ref_time[0]) + 60 * (det_time[1] - ref_time[1]) + (
        #         det_time[2] - ref_time[2])
        self.trajectory[0:] = [det_time, dist, speed, ]
        self.csv_indx = indx  # is used to find vehicle in original csv file
        self.last_trj_point_indx = -1  # changes in set_trj()

    def set_trj(self, t, d, s):
        '''
        Sets trajectory of the vehicle
        '''
        n = len(t)
        self.trajectory[1:n, :] = [t, d, s]
        self.last_trj_point_indx = n

    def set_earlst(self, t):
        '''

        :param t: earliest time vehicle can get to the stop bar if enough green is given
        :return: same
        '''
        self.earlst = t
