#!/bin/bash
# Launch zellij panes with docker exec for each running swarm container
# Usage: ./launch_swarm_zellij.sh

SERVICE_NAME="nxt"
STACK_NAME="swarmnxt"

# Get running container names for the service
CONTAINERS=$(docker ps --filter "name=${STACK_NAME}_${SERVICE_NAME}" --format "{{.Names}}" | sort)

if [ -z "$CONTAINERS" ]; then
    echo "No running containers found for ${STACK_NAME}_${SERVICE_NAME}"
    exit 1
fi

# Start zellij session if not already in one
if [ -z "$ZELLIJ" ]; then
    echo "Starting new zellij session..."
    zellij --session swarm-containers
    exit 0
fi

# We're already in zellij, create panes for each container
first=true
for cname in $CONTAINERS; do
    if [ "$first" = true ]; then
        # Use the current pane for the first container
        zellij action write-chars $'\x0c'  # Send Ctrl+L to clear screen
        zellij action write-chars "docker exec -ti $cname bash"
        zellij action write-chars $'\n'
        first=false
    else
        # Create new pane for subsequent containers
        zellij action new-pane
        zellij action write-chars "docker exec -ti $cname bash"
        zellij action write-chars $'\n'
    fi
    echo "Opened pane for container: $cname"
done

echo "All container panes created!"