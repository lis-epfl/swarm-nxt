# Position Estimate Setup

The drones need to get their position from some source. There are two options: 

- OptiTrack 
- Visual Inertial Odometry (VIO)

## OptiTrack

Assumptions: 

- You have a space with an OptiTrack system setup
- The computer that is running Motive is on the same network as the host computer and the drones


### Optitrack Setup Process

1. Place 4-5 markers on each drone such that the marker placement is asymmetrical. Each drone should have a unique placement of the markers to avoid confusion. See the [docs](https://docs.optitrack.com/motive/rigid-body-tracking) for more information
2. On the bottom right corner of Motive, the frequency should be >200 Hz <!-- check -->
3. Place each drone in view of the OptiTrack cameras, and ensure that the drone is facing the positive x direction of the coordinate system. 
4. Select each of the drones separately on the screen, right click, and click on create rigid body. Give each drone the same name as the hostname. This allows the automated tooling to work properly.

!!! warning
	It is very important that the drone is facing the positive x direction (orin towards the negative x direction). 

### Host Computer Setup

1. On the host computer, open the [ROS workspace](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Creating-A-Workspace/Creating-A-Workspace.html) and clone this [repository](https://github.com/lis-epfl/optitrack_packages_ros2/) into the src folder
2. Edit the file in `src/optitrack_packages_ros2/optitrack_multiplexer_ros2/config/optitrack_multiplexer_config.yaml`, and add all of the rigid bodies that you want to track under "rigid_body_names"
3. Run `colcon build --symlink-install`
4. Source the ROS Workspace: `source install/setup.bash`
5. Run the wrapper and the multiplexer: `ros2 launch optitrack_multiplexer_ros2 wrapper_and_multiplexer.launch.py`

### Drone Setup

You can run the `drone_preflight.yml` ansible with the `position_estimate` variable set to "optitrack" in `group_vars/all`: 

Ensure that the `inventory.ini` file has all of the drones you wish to target under the `[drones]` group. Also, make sure you put the domain name or IP of the ground control station in `drones_preflight.yml`.


```bash
cd ansible/
ansible-playbook -i inventory.ini drones_preflight.yml -K
```




