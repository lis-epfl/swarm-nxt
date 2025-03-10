#!/bin/bash

DEFAULT_HOST_HOSTNAME="nxt_host.local"
DEFAULT_PORT="14550"
DEFAULT_DOMAIN_ID=1
DEFAULT_SERIAL_PORT="/dev/ttyTHS1"
DEFAULT_BAUD_RATE="921600"

# Parse arguments
while [[ "$#" -gt 0 ]]; do
    case $1 in
        --host) HOST="$2"; shift ;;
        --port) PORT="$2"; shift ;;
        --domain-id) DOMAIN_ID="$2"; shift ;;
        --serial-port) SERIAL_PORT="$2"; shift ;;
        --baud-rate) BAUD_RATE="$2"; shift ;;
        *) echo "Unknown parameter passed: $1"; exit 1 ;;
    esac
    shift
done

# Set defaults if not provided
HOST=${HOST:-$DEFAULT_HOST_HOSTNAME}
PORT=${PORT:-$DEFAULT_PORT}
DOMAIN_ID=${DOMAIN_ID:-$DEFAULT_DOMAIN_ID}
SERIAL_PORT=${SERIAL_PORT:-$DEFAULT_SERIAL_PORT}
BAUD_RATE=${BAUD_RATE:-$DEFAULT_BAUD_RATE}

# Run the command
ROS_DOMAIN_ID=$DOMAIN_ID ros2 launch px4.launch fcu_url:="$SERIAL_PORT:$BAUD_RATE" gcs_url:="udp://@$HOST:$PORT"