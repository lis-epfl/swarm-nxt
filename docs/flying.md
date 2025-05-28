# Flying

## Pre-Flight Checklist

1. Ensure that the `inventory.ini` file has all of the hostnames of the drones under `[drones]`. This selects the drones that the operators are done on.
	1. Ensure that the `gcs_url` variable in the preflight ansible (`/path/to/omninxtrepo/ansible/drones_preflight.yml`) is set to the correct hostname or [IP address of the ground station](software-common-tasks.md#get-ip-of-computer). 
2. Run the pre-flight ansible: `cd /path/to/omninxtrepo/ansible && ansible-playbook -i inventory.ini drones_preflight.yml -K`. This will start or check:
	1. The position estimation services on each drone
	2. The mavros service on each drone. This provides information to QGroundControl, and the other services that mavros provides
	3. Checks if chronyc is tracking. This is currently just a boolean check, but can be expanded later to check latency. 
3. Open QGroundControl. Ensure every drone is connected.
4. [Check the EKF Tracking](software-common-tasks.md#check-ekf-tracking) __for each drone__
5. Check the ping between each drone and the ground station, both ways (TODO: Automate)
6. Check the batteries are charged (TODO: Automate)
7. Check that the rotors are not obstructed by any cables, and all cables are well-connected