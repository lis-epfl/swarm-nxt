#!/bin/bash
PORT=52{{ '%02d' % (ns | regex_search("[0-9]+$") | int) }}
SERVER="192.168.194.165" 
RATE=100M
LEN=1400
DUR=60
NUM_CORE_STRESS=7
sleep {{ ns | regex_search("[0-9]+$") | int }}

stress-ng --cpu ${NUM_CORE_STRESS} --timeout $DUR &
iperf3 -u -c $SERVER -p $PORT -b $RATE -t $DUR -l $LEN -R --json > /tmp/iperf_down_${PORT}_${LEN}_${RATE}_stress${NUM_CORE_STRESS}.json

# iperf3 -u -c $SERVER -p $PORT -b $RATE -t $DUR -l $LEN --json > /tmp/iperf_${PORT}_${LEN}_${RATE}.json

