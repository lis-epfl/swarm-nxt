
This documentation describes how to build and install the OmniNXT drone. 

For hardware setup, follow [Hardware Setup](hardware-setup.md)

For software setup, most users can follow the following steps: 

1. [Software Setup](software-setup.md)
2. [Flight Controller Setup](nxt-setup.md)
3. [Remote Control Setup](remote-control-setup.md)
4. [Position Estimation Setup](position-estimate-setup.md)


# Pre-Flight Checklist

1. Ensure that the `inventory.ini` file has all of the hostnames of the drones under `[drones]`.
	1. Ensure that the `gcs_url` variable in the preflight ansible (`/path/to/omninxtrepo/ansible/drones_preflight.yml`) is set to the correct hostname or [IP address of the ground station](software-common-tasks#get-ip-of-computer). 
2. Run the pre-flight ansible: `cd /path/to/omninxtrepo/ansible && ansible-playbook -i inventory.ini drones_preflight.yml -K`. This will start:
	1. The position estimation services on each drone
	2. The mavros service on each drone. This provides