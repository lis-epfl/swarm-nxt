# Flying

## Pre-Flight Checklist

1. Ensure that the `inventory.ini` file has all of the hostnames of the drones under `[drones]`. This selects the drones that the operators are done on.
	1. Ensure that the `gcs_url` variable in the preflight ansible (`/path/to/swarmnxtrepo/ansible/drones_preflight.yml`) is set to the correct hostname or [IP address of the ground station](software-common-tasks.md#get-ip-of-computer). 
2. Run the pre-flight ansible: `cd /path/to/swarmnxtrepo/ansible && ansible-playbook -i inventory.ini drones_preflight.yml -K`. This will start or check:
	1. The position estimation services on each drone
	2. The mavros service on each drone. This provides information to QGroundControl, and the other services that mavros provides
	3. Checks if chronyc is tracking. This is currently just a boolean check, but can be expanded later to check latency. 
3. Open QGroundControl. Ensure every drone is connected.
4. [Check the EKF Tracking](software-common-tasks.md#check-ekf-tracking) __for each drone__
5. Check the ping between each drone and the ground station, both ways (TODO: Automate)
6. Check the batteries are charged (TODO: Automate)
7. Check that the rotors are not obstructed by any cables, and all cables are well-connected


## Optional: Run an Update

You can run an update by going to the `ansible/` directory and running the following command: 

```bash
ansible-playbook -i inventory.ini drones_update.yml -K 
```

This will update apt repositories, pull the latest version of ros packages, and build them. 

```asciinema-player
{
    "file": "/swarm-nxt/demos/drones_update.cast"
}
```

## Post-Flight Procedures

### Log Collection

After completing flight operations, collect all flight data and logs using:

```bash
cd /path/to/swarmnxtrepo/ansible
ansible-playbook -i inventory.ini drones_postflight.yml -K
```

This will automatically:
- Download MAVLink logs from each drone's flight controller
- Collect ROS bag files and system logs from all drones  
- Consolidate logs into timestamped directories on the host computer
- Create convenient symlinks for accessing the latest flight data

Collected data includes:
- **ROS bag files**: High-level flight data, sensor readings, commands
- **MAVLink logs**: Low-level flight controller data, parameters, system status
- **System logs**: Service status, error messages, performance metrics

For more details on log analysis and data handling, see [Data Logging with ROS Bag](software-common-tasks.md#data-logging-with-ros-bag).
