## To run, first copy/move this file into the base directory of the repo
## Author: Patrick Emami

from src.sig_ctrl_interface import snmp_phase_ctrl
from src.intersection import Intersection
from src.data_io import TrafficListener, TrafficPublisher
import time

class Args:
    do_logging = False
    log_dir = "./temp"

phases = [1,2,3,4]

intersection = Intersection("RTS")

tl = TrafficListener("localhost", 24601)
tp = TrafficPublisher(intersection, "localhost", 4201, Args())

tl.start()
tp.start()

calls = 0

try:
    while calls < 10:
        snmp_phase_ctrl(phases[(calls % (len(phases)-1))+1], "RTS")
        time.sleep(5)
        calls += 1
    tl.stop()
    tp.stop()
except KeyboardInterrupt:

    tl.stop()
    tp.stop()
