#!/bin/sh

INTERSECTION="RTS"
MODE="realtime"
RUN_DURATION=900
LOOP_FREQ=10
SOLVE_FREQ=0.5
TRAFFIC_LISTENER_IP="localhost"
TRAFFIC_LISTENER_PORT=24601
TRAFFIC_PUBLISHER_PORT=7001
DO_LOGGING=True
RUN_WITH_SIGNAL_CONTROL=False

python main.py --intersection $INTERSECTION --mode $MODE --run-duration $RUN_DURATION --loop-freq $LOOP_FREQ --solve-freq $SOLVE_FREQ --traffic-listener-ip $TRAFFIC_LISTENER_IP --traffic-listener-port $TRAFFIC_LISTENER_PORT --traffic-publisher-port $TRAFFIC_PUBLISHER_PORT --do-logging $DO_LOGGING --run-with-signal-control $RUN_WITH_SIGNAL_CONTROL
