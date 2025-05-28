# Manual Setup

These instructions are for documentation purposes, normal users do not need to follow them. They show how the images or parameter files used in the other sections were made. This section will be useful if you want to understand how to make those assets and change something in the default settings that we've assumed. 

## Image Creation

!!! important
    You do not need to follow all of these steps if you are simply flashing a pre-made image. 

### Raw Image Setup 

These steps describe how to build the intial image for the NVIDIA Orin board. These steps are only necessary if you want to rebuild the raw image. 

1. Ensure that the devkit is turned off and the power unplugged, but ready to be plugged in
2. Ensure that the devkit is connected to the host computer via the Recovery Port
3. Press the button labelled "REC" on the Devboard
4. While pressing the REC button, press the RES button
5. Release the RES button while still pressing the REC button
6. Connect the power to the Jetson, and release the REC button after the power is connected
7. On the host computer, run `lsusb`. This sequence was a success if an entry with NVIDIA Corp. APX is visible (`ID 0955:7323 NVIDIA Corp. APX`)


Download the SDK Manager from the [NVIDIA Website](https://developer.nvidia.com/sdk-manager). Your computer should be running the same OS version as you would like to be installed on the Orin. If not, you can run it in docker. These instructions can be found on the NVIDIA SDK Manager page.  Then, run the following steps: 

1. Install the SDK Manager: `sudo dpkg -i /path/to/sdkmanager.deb`. If required, install any missing dependencies with `sudo apt --fix-broken install`
2. Run the sdkmanager: `sdkmanager --cli`
3. Login, and then select the following options: install -> jetson -> target hardare
4. The Orin NX should already be detected, select it. 
5. Reply Y to showing all Jetson versions. Select JetPack 6.2
6. Additional SDKs: <!-- TODO: which ones? -->
7. Customize install settings: Y
8. Select all the options by using the arrow keys and space to select, enter to continue
9. Reply Y to "Do you want to flash Jetson Orin NX module?". Select Pre-Config for OEM Config. 
10. Username: lis, Password: orin
11. Choose NVME for the storage device
12. Wait for the installation to finish. 
13. Select proxy mode
14. Select the IP that you want the device to have. This documentation assumes `192.168.55.1`


!!! warning
    If you get an error like "cannot stat /dev/nvme0n1", ensure that the flash storage is fully seated.

!!! note
    The raw image was created after these steps.


### Password-Free Access

You must set this up before running the playbook below. See [Password-Free Access](software-common-tasks.md#password-free-access)

### Run Ansible Playbook

We use [Ansible](https://www.redhat.com/en/ansible-collaborative) to facilitate idempotent configuration. 

Make sure Ansible is installed following the [setup instructions](infra-setup.md#ansible-installation)

Then, clone the [repo](https://github.com/lis-epfl/onix-nxt), and open a terminal in the ansible directory. To make the following process faster, download the patched-kernel tarball from the [Google Drive](https://drive.google.com/drive/u/1/folders/1XL-hTVf6IsB96XvfQLSesLO4FHOVjW6y). Place this file in a folder called `ansible/patched-kernel` inside the repo. 

Ensure you are in the `ansible/` folder of the repository. Run `ansible-playbook -i inventory.ini drone_setup.yml -K`. It will ask you for a BECOME password. It is the root password of the orin (`orin`). There should be no failed steps.

!!! note
    The post install image was created after this step, with the hostname set to `ubuntu`


### Backups and Restores

Common: 

-  On the host computer with the NVIDIA SDK installed, navigate to `~/nvidia/nvidia_sdk/JetPack_6.2_Linux_JETSON_ORIN_NX_TARGETS/Linux_for_Tegra`
#### Create Image Procedure

1. Prepare the device by installing what is required
2. Cleanup any log files and unnecessary files to make the image compact
3. Restart the device in recovery mode (follow the steps in [Raw Image Setup](#raw-image-setup))
4. Run `sudo ./tools/backup_restore/l4t_backup_restore.sh -e nvme0n1 -b jetson-orin-nano-devkit-nvme`
5. Compress the image folder: `cd tools/backup_restore && sudo tar -cvzf <name>.tar.gz images`
6. Save the compressed image for restore later. 

#### Restore Procedure
1. Ensure that the image you want to restore is unzipped in tools/backup_restore/ (There should be a directory called `images` with various .img files inside)
2. Ensure you are in the `Linux_for_Tegra` folder. 
3. Restart the device in recovery mode (follow the steps in [Raw Image Setup](#raw-image-setup))
4. Run `sudo ./tools/backup_restore/l4t_backup_restore.sh -e nvme0n1 -r jetson-orin-nano-devkit-nvme`


## NXT Flight Controller Setup

These steps were followed to create the parameter file described in [Drone Setup](drone-setup.md#post-flash-setup). This file only replaces the application of the parameter file part. You still must complete the rest of the setup described in the [NXT Configuration and Calibration Section](drone-setup.md#nxt-configuration-and-calibration)
### Airframe

!!! important
	You must do this first, since it will reset the other values.

Under airframe, select Quadcopter X, Generic Quadcopter. Click on apply and restart. 


### General Settings

- If indoors, no GPS (assumed default)
	- `COM_ARM_WO_GPS`: Warning 
	- `COM_ARM_MAG_STR`: Warning
	- `SYS_HAS_MAG`, `SYS_HAS_GPS`, `SYS_HAS_BARO`: Set to 0/Disabled. 
- `RC_PORT_CONFIG`: Radio Controller 
- `IMU_GYRO_RATEMAX`: 2000Hz
- `IMU_INTEG_RATE`: 400Hz
- `MAV_0_MODE`: External Vision
- `MAV_0_RATE`: 92160 B/s

### Power

Navigate to the power tab and fill in these settings: 

- Source: Power Module

Set the following items to match the battery you're using. We have provided the settings for the battery that we used: 

- Set the number cells:  6 
- Empty Voltage: 2.56V 
- Full Voltage: 4.05V

### Actuators
Open the MAVLink console (in the Analyze Tools menu accessible from the Q button on the main screen). Input the coordinates of the actuators. It is done this way since the Actuators tab truncates values to two decimal places. 

```
param set CA_ROTOR0_PX 0.0535
param set CA_ROTOR0_PY 0.0535

param set CA_ROTOR1_PX -0.0535
param set CA_ROTOR1_PY -0.0535

param set CA_ROTOR2_PX 0.0535
param set CA_ROTOR2_PY -0.0535

param set CA_ROTOR3_PX -0.0535
param set CA_ROTOR3_PY 0.0535
```


These positions are relative to the center of gravity of the drone, so ensure that the battery is placed such that the CoG is truly at the middle of the drone. Then, go back to the Vehicle Setup menu and open the Actuators tab. __DO NOT TOUCH THE POSITION TEXTBOXES__ since the values will truncate to two decimal places. 

### Safety Setup

On the Safety tab: 

1. Set the battery failsafe to land mode
2. Set the RC loss failsafe to land mode
3. Set the geofence failsafe to land mode
4. Set the land mode speed to 0.3m/s and disarm after 7s. You will have to force save.

On the parameters tab: 

1. Set `COM_OBL_RC_ACT` to Land Mode
2. Set `CBRK_SUPPLY_CHK` to 0. 

!!! important
	If you do not set the `CBRK_SUPPLY_CHK` to zero, none of the battery related checks will execute!



### EKF Setup

Set the following parameters in the Parameters screen: 

| **Parameter**       | **Value**        | **Notes**                                       |
|---------------------|------------------|-------------------------------------------------|
| `EKF2_ACC_NOISE`   | max_value (1.00)  |                                                 |
| `EKF2_ACC_B_NOISE` | max_value (0.01)  |                                                 |
| `EKF2_GYR_NOISE`   | max_value (0.1)   |                                                 |
| `EKF2_GYR_B_NOISE` | max_value (0.01)  |                                                 |
| `EKF2_EV_NOISE_MD` | 0.0               |                                                 |
| `EKF2_EVP_NOISE`   | 0                 | Need to force save                              |
| `EKF2_EVA_NOISE`   | 0                 |                                                 |
| `EKF2_EV_CTRL`     | 11                | Horizontal, Vertical, and Yaw should be checked |
| `EKF2_HGT_REF`     | Vision            |                                                 |
| `EKF2_GPS_CTRL`    | 0                 |                                                 |
| `EKF2_BARO_CTRL`   | 0                 |                                                 |
| `EKF2_RNG_CTRL`    | 0                 |                                                 |

Restart the drone by going to Tools > Restart.
