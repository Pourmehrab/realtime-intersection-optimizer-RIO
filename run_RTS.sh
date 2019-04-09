#!/bin/sh

INTERSECTION="RTS"
MODE="sim"
RUN_DURATION=30
LOOP_FREQ=10
SOLVE_FREQ=0.5
TRAFFIC_LISTENER_IP="localhost"
TRAFFIC_LISTENER_PORT=24601
TRAFFIC_PUBLISHER_IP="169.254.117.41"
TRAFFIC_PUBLISHER_PORT=4201
DO_LOGGING=False
RUN_WITH_SIGNAL_CONTROL=False

python main.py --intersection $INTERSECTION --mode $MODE --run-duration $RUN_DURATION --loop-freq $LOOP_FREQ --solve-freq $SOLVE_FREQ --traffic-listener-ip $TRAFFIC_LISTENER_IP --traffic-listener-port $TRAFFIC_LISTENER_PORT --traffic-publisher-port $TRAFFIC_PUBLISHER_PORT --do-logging $DO_LOGGING --run-with-signal-control $RUN_WITH_SIGNAL_CONTROL --traffic-publisher-ip $TRAFFIC_PUBLISHER_IP
