'''
By:     Mahmoud Pourmehrab
E-mail: mpourmehrab@ufl.edu
Date:        Dec 2017
Last update: Dec/08/2017
'''


class _DoublyLinkedBase:
    class _Node:
        __slots__ = '_element', '_prev', '_next'

        def __init__(self, element, prev, next):
            self._element = element
            self._prev = prev
            self._next = next

    def __init__(self):
        self._header = self._Node(None, None, None)
        self._trailer = self._Node(None, None, None)
        self._header._next = self._trailer
        self._trailer._prev = self._header
        self._size = 0

    def __len__(self):
        return self._size

    def is_empty(self):
        return self._size == 0

    def _insert_between(self, e, predecessor, successor):
        newest = self._Node(e, predecessor, successor)
        predecessor._next = newest
        successor._prev = newest
        self._size += 1
        return newest

    def _delete_node(self, node):
        predecessor = node._prev
        successor = node._next
        predecessor._next = successor
        successor._prev = predecessor
        self._size -= 1
        element = node._element
        node._prev = node._next = node._element = None
        return element


# class LinkedDeque(_DoublyLinkedBase):
#     def first(self):
#         if self.is_empty():
#             raise Empty('Deque is empty')
#         return self._header._next._element
#
#     def last(self):
#         if self.is_empty():
#             raise Empty('Deque is empty')
#         return self._trailer._prev._element
#
#     def insert_first(self, e):
#         self._insert_between(e, self._header, self._header._next)
#
#     def insert_last(self, e):
#         self._insert_between(e, self._trailer._prev, self._trailer)
#
#     def delete_first(self):
#         if self.is_empty():
#             raise Empty('Deque is empty')
#         return self._delete_node(self._header._next)
#
#     def delete_last(self):
#         if self.is_empty():
#             raise Empty('Deque is empty')
#         return self._delete_node(self._trailer._prev)


class VehiclesList(_DoublyLinkedBase):
    '''
    A doubly linked list to keep vehicles in a lane
    '''

    # -------------------------- nested Position class --------------------------
    class Position:
        def __init__(self, container, node):
            self._container = container
            self._node = node

        def element(self):
            return self._node._element

        def __eq__(self, other):
            return type(other) is type(self) and other._node is self._node

        def __ne__(self, other):
            return not (self == other)

    # ------------------------------- utility method -------------------------------
    def _validate(self, p):
        if not isinstance(p, self.Position):
            raise TypeError('p must be proper Position type')
        if p._container is not self:
            raise ValueError('p is no longer valid')
        return p._node

    # ------------------------------- utility method -------------------------------
    def _make_position(self, node):
        if node is self._header or node is self._trailer:
            return None
        else:
            return self.Position(self, node)

    # ------------------------------- accessors -------------------------------
    def first(self):
        return self._make_position(self._header._next)

    def last(self):
        return self._make_position(self._trailer._prev)

    def before(self, p):
        node = self._validate(p)
        return self._make_position(node._prev)

    def after(self, p):
        node = self._validate(p)
        return self._make_position(node._next)

    def __iter__(self):
        cursor = self.first()
        while cursor is not None:
            yield cursor.element()
            cursor = self.after(cursor)

    # ------------------------------- mutators -------------------------------
    def _insert_between(self, e, predecessor, successor):
        node = super()._insert_between(e, predecessor, successor)
        return self._make_position(node)

    def add_first(self, e):
        return self._insert_between(e, self._header, self._header._next)

    def add_last(self, e):
        return self._insert_between(e, self._trailer._prev, self._trailer)

    def add_before(self, p, e):
        original = self._validate(p)
        return self._insert_between(e, original._prev, original)

    def add_after(self, p, e):
        original = self._validate(p)
        return self._insert_between(e, original, original._next)

    def delete(self, p):
        original = self._validate(p)
        return self._delete_node(original)

    def replace(self, p, e):
        original = self._validate(p)
        old_value = original._element
        original._element = e
        return old_value


class MahmoudLanes:
    def __init__(self, nl):
        '''
        Data Structure for an a dictionary of lanes

        :param nl: number of lanes
        '''
        # todo:(Mahmoud) a lane can have speed limit, opt range and other some other attributes
        self.vehlist = {l: VehiclesList() for l in range(0, nl)}


class MahmoudVehicle:
    def __init__(self, det_id, det_type, det_time, speed, dist, des_speed, dest=1, length=4, amin=-4.5, amax=3):
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
        '''
        self.id = det_id
        self.type = det_type
        self.det_time = det_time
        self.speed = speed
        self.dist = dist
        self.length = length
        self.amin = amin
        self.amax = amax
        self.dest = dest
        self.des_speed = des_speed

    def set_freeflowtt(self):
        '''

        :return: freeflowtt:      free flow travel time in second
        '''
        pass # todo compute free flow travel time

    def set_trj(self):
        '''

        :return: trajectory of the vehicle
        '''
        pass # todo set the trajectory

    def set_earlst(self,t):
        '''

        :param t: earliest time vehicle can get to the stopbar if enouhg green is given
        :return: same
        '''
        self.earlst = t
