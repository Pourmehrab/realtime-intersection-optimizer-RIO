#!/bin/sh

INTERSECTION="RTS"
MODE="realtime"
RUN_DURATION=10
LOOP_FREQ=0.5
SOLVE_FREQ=0.5
TRAFFIC_LISTENER_IP="localhost"
TRAFFIC_LISTENER_PORT=24601
TRAFFIC_PUBLISHER_IP="169.254.117.41"
TRAFFIC_PUBLISHER_PORT=4201
DO_LOGGING=True
RUN_WITH_SIGNAL_CONTROL=True

python main.py --intersection $INTERSECTION --mode $MODE --run-duration $RUN_DURATION --loop-freq $LOOP_FREQ --solve-freq $SOLVE_FREQ --traffic-listener-ip $TRAFFIC_LISTENER_IP --traffic-listener-port $TRAFFIC_LISTENER_PORT --traffic-publisher-port $TRAFFIC_PUBLISHER_PORT --do-logging $DO_LOGGING --run-with-signal-control $RUN_WITH_SIGNAL_CONTROL --traffic-publisher-ip $TRAFFIC_PUBLISHER_IP
