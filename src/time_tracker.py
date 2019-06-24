from datetime import datetime
from src.util import periodic_sleep

class Timer:
    """
    Parent class for real/sim timer classes
    """
    def __init__(self, start_time, resolution):
        self.start_time = start_time
        self.resolution = resolution

    @staticmethod
    def get_timer(mode, resolution):
        if mode == "realtime" or mode == "RealTime":
            return RealTimeTimer(resolution)
        elif mode == "sim" or mode == "Sim":
            return SimTimer(resolution)

    def step(self):
        raise NotImplementedError

    def get_time(self, timestamp):
        raise NotImplementedError


class RealTimeTimer(Timer):

    def __init__(self, resolution):
        """
        :param resolution: the time (in sec) to sleep every step
        """
        start_time = datetime.utcnow()
        super(RealTimeTimer, self).__init__(start_time, resolution)

    def step(self):
        periodic_sleep(self.resolution)

    def get_time(self, timestamp=None):
        """
        Computes elapsed time if timestamp is not None,
        otherwise returns current absolute time.
        """
        if not timestamp:
            timestamp = datetime.utcnow() 
        return (timestamp - self.start_time).total_seconds(), timestamp


class SimTimer(Timer):
    """
    Keeps time by incrementing a counter
    with a fixed delta
    """
    def __init__(self, resolution):
        super(SimTimer, self).__init__(0, resolution)
        self.curr_time = self.start_time

    def step(self):
         self.curr_time += self.resolution

    def get_time(self, timestamp=None):
        if not timestamp:
            timestamp = self.curr_time
        return timestamp - self.start_time, timestamp
