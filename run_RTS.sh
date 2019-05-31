#!/bin/sh

INTERSECTION="RTS"
MODE="sim"
RUN_DURATION=30
LOOP_FREQ=1
SOLVE_FREQ=1
TRAFFIC_LISTENER_IP="localhost"
TRAFFIC_LISTENER_PORT=24601
TRAFFIC_PUBLISHER_IP="169.254.117.41"
TRAFFIC_PUBLISHER_PORT=4201
DO_LOGGING=True
RUN_WITH_SIGNAL_CONTROL=False
SHOW_VIZ=False
SAVE_VIZ=True

python main.py --intersection $INTERSECTION --mode $MODE --run-duration $RUN_DURATION --loop-freq $LOOP_FREQ --solve-freq $SOLVE_FREQ --traffic-listener-ip $TRAFFIC_LISTENER_IP --traffic-listener-port $TRAFFIC_LISTENER_PORT --traffic-publisher-port $TRAFFIC_PUBLISHER_PORT --do-logging $DO_LOGGING --run-with-signal-control $RUN_WITH_SIGNAL_CONTROL --traffic-publisher-ip $TRAFFIC_PUBLISHER_IP --show-viz $SHOW_VIZ --save-viz $SAVE_VIZ