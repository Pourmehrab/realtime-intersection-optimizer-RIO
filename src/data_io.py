import threading
import socket
from collections import deque, namedtuple
import datetime
import numpy as np
import os
import time

VehicleMsg = namedtuple('VehicleMsg', 
        ['timestamp', 
         'track_id', # string
         'dsrc_id', # string
         'pos', # List [UTM easting (float), UTM northing (float)]
         'vel', # List [UTM easting dot (float), UTM northing dot (float)]
         'pos_rms', # List [UtM easting (float), UTM northing (float)]
         'vel_rms', # List [UTM easting dot (float), UTM northing dot (float)]
         'veh_type', # 0 or 1
         'veh_len', # float
         'max_accel', # float
         'max_decel', # float
         'road' # strin
         ])

class StoppableThread(threading.Thread):
    """Thread class with a ``stop()`` method. The thread itself has to check
    regularly by calling the `stopped()`` method.
    """

    def __init__(self, name):
        super(StoppableThread, self).__init__(name=name)
        self._stopper = threading.Event()

    def stop(self):
        """Stop the thread."""
        self.stop_thread()
        self.join(1)
        if self.isAlive():
            raise RuntimeError("Thread timeout event")

    def stop_thread(self):
        self._stopper.set()

    def stopped(self):
        return self._stopper.is_set()


class SocketThread(StoppableThread):
    """
    Used for receiving data over a UDP socket. Provides
    a hook, ``handle(msg)``, that should be extended via subclassing.
    """
    def __init__(self, ip_address, port, msg_len, name):
        super(SocketThread, self).__init__(name)
        self._ip_address = ip_address
        self._port = port
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        # open connection to incoming messages
        try:
            self._sock.bind((self._ip_address, self._port))
            self._sock.setblocking(0)
        except socket.error as e:
            print(e)
        self._msg_len = msg_len

    def run(self):
        while not self.stopped():
            try:
                msg, address = self._sock.recvfrom(self._msg_len)
            except socket.error as err:
                continue
            try:
                self.handle(msg)
            except Exception as e:
                continue

        self._sock.close()

    def handle(self, msg):
        """
        Hook method for processing a new msg.
        Needs to be defined by the subclass
        """
        raise NotImplementedError

class TrafficListener(SocketThread):
    """
    Extension of the SocketThread class that defines how
    new traffic messages should be parsed.

    Creates and provides access to synchronized queues for sharing
    the most recent batch of traffic messages. Currently, behavior 
    is to only maintain the most recent batch of messages in the queues
    and to throw away the next oldest.

    Track split/merge messages are not yet supported. 
    """
    def __init__(self, ip_address, port, msg_len=500, name="TrafficListener"):
        super(TrafficListener, self).__init__(ip_address, port, msg_len, name)
        self._vehicle_data_queue = deque(maxlen=1)
        self._track_split_merge_queue = deque(maxlen=1)

    def get_vehicle_data_queue(self):
        """
        Used to access the synchronized vehicle data queue externally.
        """
        return self._vehicle_data_queue

    def get_track_split_merge_queue(self):
        """
        Used to access the synchronized track split/merge queue externally.
        """
        return self._track_split_merge_queue

    def handle(self, msg):
        """
        Parse incoming traffic messages. See TODO for a specification
        of traffic message types.

        :param msg: The incoming message. 
        :type string:
        """
        # parse the msg
        if type(msg) == bytes:
            msg = msg.decode("utf-8")

        msg_chunks = msg.split("^^^^")
        # timestamp is first
        timestamp = msg_chunks[0].split(":")
        hour = int(timestamp[0])
        minute = int(timestamp[1])
        second, microsecond = timestamp[2].split(".") # {second}.{microsecond}
        time_obj = datetime.time(hour, minute, int(second), int(microsecond))
        date_obj = datetime.date.today()
        parsed_vehicle_msgs = []
        parsed_track_split_merge_msgs = []
        def parse_vehicle_msg(msg_):
            data = msg_.split(",")
            vm = VehicleMsg(
                    timestamp=datetime.datetime.combine(date_obj, time_obj),
                    track_id=data[1],
                    dsrc_id=data[2],
                    pos=[float(data[3]), float(data[4])],
                    vel=[float(data[5]), float(data[6])],
                    pos_rms=[float(data[7]), float(data[8])],
                    vel_rms=[float(data[9]), float(data[10])],
                    veh_type=int(data[11]), veh_len=float(data[12]),
                    max_accel=float(data[13]),
                    max_decel=float(data[14]),
                    road=data[15])
            return vm
        for m in msg_chunks[1:]:
            if m.split(",")[0] == "0":
                parsed_vehicle_msgs.append(parse_vehicle_msg(m))
            elif m.split(",")[0] == "1":
                parsed_track_split_merge_msgs.append(parse_track_split_merge_msg(m))

        # Log incoming data + timestamp? 

        if len(parsed_vehicle_msgs) > 0:
            self._vehicle_data_queue.append(parsed_vehicle_msgs)
        if len(parsed_track_split_merge_msgs) > 0:
            self._track_split_merge_queue.append(parsed_track_split_merge_msgs)

class TrafficPublisher(StoppableThread):
    """
    Thread class for sending out intersection approach messages (IAMs) via UDP socket
    to the RSU.

    Operates by maintaining a synchronized queue and sending out queued up trajectories 
    as soon as they are added to the queue.
    """
    def __init__(self, intersection, ip, port, args, name="TrafficPublisher"):
        super(TrafficPublisher, self).__init__(name)
        self._cav_traj_queue = deque()
        self._IAM_publisher = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._IAM_publisher.setblocking(0)
        self._intersection = intersection
        self.ip = ip
        self.port = port
        self.do_logging = args.do_logging
        if self.do_logging:
            self.log_file = open(os.path.join(args.log_dir, "IAM.txt"), "w")

    def get_cav_traj_queue(self):
        """
        Used to access the synchronized cav traje queue externally.
        """
        return self._cav_traj_queue

    def veh_to_IAM(self, veh, lane, timestamp):
        """
        Given a Vehicle object, extracts the trajectory and populates an IAM
        for sending out to the RSU.

        The IAM message is a string consisting of:
            - IAM message ID
            - Hour (added by RSU)
            - Minute (added by RSU)
            - Millisecond (added by RSU)
            - vehicle DSRC ID
            - 4 byte latitutde of first trajectory (unsigned long/int32)
            - 4 byte longitude of first trajectory point (unsigned long/int32)
            - Minute when the first trajectory point occurs
            - Millisecond when first trajectory point occurs
            - Signal color (e.g., red if vehicle trajectory is to stop at stopbar and wait for green)
            - Point count (# of trajectory point - 1)
            - Repeat for point count
                - Latitutde offset (3 bytes), unsigned int16
                - Longitude offset (3 bytes), unsigned int16
                - Time offset (milliseconds)
        """
        max_num_traj_points = self._intersection._inter_config_params["max_num_traj_points"]
        dsrc_ID = int(veh.ID.split(":")[1])
        traj_len = min(veh.last_trj_point_indx - veh.first_trj_point_indx + 1,
            max_num_traj_points) 
        point_count = traj_len - 1
        traj_point_ctr = 0
        traj_point_idx = veh.first_trj_point_indx
        trajectory = []
        while traj_point_ctr < traj_len:
            x = veh.trajectory[:, traj_point_idx]
            trj_t, trj_dist, trj_spd = x[0], x[1], x[2]
            lat, lon = self._intersection.distance_from_stopbar_to_LLA(trj_dist, lane)
            traj_point_idx = 1 + traj_point_idx % max_num_traj_points
            traj_point_ctr += 1
            trajectory.append(np.array([trj_t, lat, lon]))
        x = np.array(trajectory)
        # compute deltas
        deltas = x - np.concatenate([np.array([[0,0,0.]]), x[:-1,:]])
        first_time_offset_sec = datetime.timedelta(seconds=deltas[0,0])
        timestamp += first_time_offset_sec
        first_lat = int(round(deltas[0,1] * 1e7))
        first_lon = int(round(deltas[0,2] * 1e7))
        signal_color = 2 # TODO: green always currently
        first_min = timestamp.minute
        first_sec = int(round(1e3 * (timestamp.second + (1e-6 * timestamp.microsecond))))
        # 18 is the IAM msg ID (fixed), 0 is the MsgCount variable (fixed)
        IAM_blob = "18,0,"
        IAM_blob += "%d," % dsrc_ID
        IAM_blob += "%d," % first_lat
        IAM_blob += "%d," % first_lon
        IAM_blob += "%d," % first_min
        IAM_blob += "%d," % first_sec
        IAM_blob += "%d," % signal_color
        IAM_blob += "%d," % point_count
        for i in range(1,traj_len): 
            # This has to be representable in less than 3 bytes, in (-2^23 to 2^23-1)
            lat_offset = int(round(deltas[i,1] * 1e7))
            lon_offset = int(round(deltas[i,2] * 1e7))
            time_offset = int(round(deltas[i,0] * 1e3))
            IAM_blob += "%d,%d,%d" % (lat_offset,lon_offset,time_offset)
        return IAM_blob

    def close(self):
        self._IAM_publisher.close()
    
    def run(self):
        while not self.stopped():
            while len(self._cav_traj_queue) > 0:
                next_veh, lane, timestamp = self._cav_traj_queue.pop()
                next_IAM = self.veh_to_IAM(next_veh, lane, timestamp)
                next_IAM = next_IAM.encode('utf-8')
                self._IAM_publisher.sendto(next_IAM, (self.ip, self.port))
                if self.do_logging:
                    self.log_file.write(str(timestamp) + " " + next_IAM + "\n")
            #time.sleep(0.1)
        self._IAM_publisher.close()
        if self.do_logging:
            self.log_file.close()
