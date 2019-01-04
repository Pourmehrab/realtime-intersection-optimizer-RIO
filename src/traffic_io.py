import threading
import socket
from collections import deque, namedtuple
import datetime

vehicle_msg = namedtuple('VehicleMsg', 
        ['timestamp', 
         'track_id', # string
         'dsrc_id', # string
         'pos', # List [UTM easting (float), UTM northing (float)]
         'speed', # List [UTM easting dot (float), UTM northing dot (float)]
         'pos_rms', # List [UtM easting (float), UTM northing (float)]
         'speed_rms', # List [UTM easting dot (float), UTM northing dot (float)]
         'veh_type', # 0 or 1
         'veh_len', # float
         'max_accel', # float
         'max_decel', # float
         'road' # strin
         ])

class StoppableThread(threading.Thread):
    """Thread class with a stop() method. The thread itself has to check
    regularly for the stopped() condition.
    
    :Author:
        Patrick Emami <pemami@ufl.edu>
    :Date:
        Dec 2018
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
    """Used for receiving data over a UDP socket."""

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
        """Main method for Stoppable thread"""
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
        Push the msg through the processing pipeline.

        Needs to be defined by the subclass
        """
        raise NotImplementedError

class TrafficListener(SocketThread):

    def __init__(self, ip_address, port, msg_len=500, name="TrafficListener"):
        super(TrafficListener, self).__init__(ip_address, port, msg_len, name)
        self._vehicle_data_queue = deque(maxlen=1)
        self._track_split_merge_queue = deque(maxlen=1)

    def get_vehicle_data_queue(self):
        return self._vehicle_data_queue

    def get_track_split_merge_queue(self):
        return self._track_split_merge_queue

    def handle(self, msg):
        # parse the msg
        msg_chunks = msg.split("-")
        # timestamp is first
        timestamp = msg_chunks[0].split(":")
        hour = int(timestamp[0])
        minute = int(timestamp[1])
        second = float(timestamp[2]) # {second}.{microsecond}
        time_obj = datetime.time(hour, minute, second, microsecond)
        date_obj = datetime.today().date()
        parsed_vehicle_msgs = []
        def parse_vehicle_msg(msg_):
            data = msg_.split(",")
            vm = vehicle_msg(
                    timestamp=datetime.combine(date_obj, time_obj),
                    track_id=data[0],
                    dsrc_id=data[1],
                    pos=[float(data[2]), float(data[3])],
                    speed=[float(data[4]), float(data[5])],
                    pos_rms=[float(data[6]), float(data[7])],
                    speed_rms=[float(data[8]), float(data[9])],
                    veh_type=int(data[10]),
                    max_accel=float(data[11]),
                    max_decel=float(data[12]),
                    road=data[13])
            return vm
        for m in msg_chunks[1:]:
            parsed_vehicle_msgs.append(parse_vehicle_msg(m))

        # Log incoming data + timestamp? 

        # push it onto the queue
        self._vehicle_data_queue.append(parsed_vehicle_msgs)

class TrafficPublisher(StoppableThread):
    """ """
    def __init__(self, intersection, ip, port, name="TrafficPublisher"):
        super(TrafficPublisher, self).__init__(name)
        self._cav_traj_queue = deque()
        self._IAM_publisher = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._IAM_publisher.setblocking(0)
        self._intersection = intersection
        self.ip = ip
        self.port = port
        
    def get_cav_traj_queue(self):
        return self._cav_traj_queue

    def veh_to_IAM(self, veh, lane):
        """
        TODO: Test

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
        https://github.com/pemami4911/FDOT-Intersection-Controller/blob/master/traffic-intersection-comms/interface/EncodeIAMasString.m
        """
        max_num_traj_points = self._intersection._inter_config_params["max_num_traj_points"]
        dsrc_ID = veh.ID.split(":")[1]
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
        deltas = x - np.concatenate(np.array([0,0,0.]), x[1:])
        first_lat = int(round(deltas[0,1] * 1e7))
        first_lon = int(round(deltas[0,2] * 1e7))
        signal_color = 2 # TODO: green always currently
        first_min = deltas[0,0].minute
        first_sec = int(round(deltas[0,0].second * 1e3))
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
            IAM_blob += "%d,%d,%d" % lat_offset,lon_offset,time_offset
        return IAM_blob

    def run(self):
        """Main method for Stoppable thread"""
        while not self.stopped():
            while self._cav_traj_queue.count() > 0:
                next_veh = self._cav_traj_queue.pop()
                next_IAM = self.veh_to_IAM(next_veh)
                self._IAM_publisher.send(next_IAM, (self.ip, self.port))    
        self._IAM_publisher.close()

