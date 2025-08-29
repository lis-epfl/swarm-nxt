#!/bin/bash
# Generate an Ansible inventory file for running swarm containers
# Usage: ./generate_swarm_inventory.sh > sim_inventory.ini

SERVICE_NAME="nxt"
STACK_NAME="swarmnxt"

# Get running container names for the service
CONTAINERS=$(docker ps --filter "name=${STACK_NAME}_${SERVICE_NAME}" --format "{{.Names}}" | sort)

# Start inventory file
echo "[drones]"

for cname in $CONTAINERS; do
    # Get container IP address (host networking will not show IP, use DNS name if needed)
    IP=$(docker inspect -f '{{range.NetworkSettings.Networks}}{{.IPAddress}}{{end}}' "$cname")
    # If IP is empty, use container name (for host networking)
    if [ -z "$IP" ]; then
        echo "$cname"
    else
        echo "$cname ansible_host=$IP"
    fi
    # Optionally add more variables per host here
    # echo "$cname ansible_host=$IP drone_id=..."
done

echo ""
echo "[drones:vars]"
echo "ansible_connection=community.docker.docker"
echo "ansible_python_interpreter=python3"

echo ""
echo "[host]"
echo "host_computer"
echo ""
echo "[host:vars]"
echo "ansible_connection=community.docker.docker"



