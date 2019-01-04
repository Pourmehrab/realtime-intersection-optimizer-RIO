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
    def __init__(self, ip, port, name="TrafficPublisher"):
        super(TrafficPublisher, self).__init__(name)
        self._cav_traj_queue = deque()
        self._IAM_publisher = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._IAM_publisher.setblocking(0)
        self.ip = ip
        self.port = port
        
    def get_cav_traj_queue(self):
        return self._cav_traj_queue

    def veh_to_IAM(self, veh):
        """TODO"""
        raise NotImplementedError

    def run(self):
        """Main method for Stoppable thread"""
        while not self.stopped():
            while self._cav_traj_queue.count() > 0:
                next_veh = self._cav_traj_queue.pop()
                next_IAM = self.veh_to_IAM(next_veh)
                self._IAM_publisher.send(next_IAM, (self.ip, self.port))    
        self._IAM_publisher.close()

