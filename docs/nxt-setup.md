# Flight Controller Setup

## Host Computer Dependencies

### Automatable Dependencies

For easily running through the automatable steps, ensure Ansible is installed on the host computer by following the steps [here](software-common-tasks.md#ansible-installation). 

Clone the [repository](https://github.com/lis-epfl/onix-nxt). Then, change to the ansible directory and run: 

```bash
ansible-playbook -i inventory.ini flight_controller.yml -K
``` 

Enter your password for sudo as the BECOME password. Next, it will ask you for a path to store the downloaded assets. You can specify one, or use the default by pressing enter. We will refer to this path as `/path/to/assets`

You can do these steps manually by looking through the `flight_controller.yml` file.

### Non-Automatable Dependencies

1. Download [STM32CubeProgrammer](https://www.st.com/en/development-tools/stm32cubeprog.html?cache=nocache#get-software) if you do not have it. 
2. Extract the folder, open it in terminal, and run `sudo ./SetupSTM32CubeProgrammer-<version>.linux` (replace `<version>` with the version you've downloaded)
3. Walk through the wizard and install the program. 

## Flashing Firmware

!!! important  
    All the steps above must be finished prior to this section

### Bootloader

1. Ensure the flight controller is powered off, unplug any power from the drone. 
2. While pressing the BOOT button beside the MicroSD card, connect the USB-C cable to the PC and release the boot button. 
3. Open the STM32CubeProgrammer by navigating to where it was installed in a terminal (`/usr/local/STMicroelectronics/STM32Cube/STM32CubeProgrammer/bin` by default), and running `sudo ./STM32CubeProgrammer`
4. On the blue dropdown next to the green connect button, select USB. 
5. Click the refresh button until a port is automatically selected under port. If it doesn't show up, ensure the board is in DFU mode by unplugging the USB-C cable, and holding the BOOT button while plugging it in again. 
6. Open the menu on the left side and select "Erasing & Programming"
7. Under file path, click browse and find the bootloader bin file. It was built in the automatable dependencies playbook. It will be in `/path/to/assets/PX4-Autopilot/build/hkust_nxt-dual_bootloader/hkust_nxt-dual_bootloader.bin`
8. Check Verify programming, and Run after programming. Ensure the start address is `0x08000000`.
9. Click on start Start Programming. 

### Main Program

1. Navigate to `/path/to/assets` in a terminal 
2. Open QGroundControl: `./QGroundControl.AppImage`. 
3. Click on the Q symbol on the top left, and click on vehicle setup. 
4. Click on the firmware tab, and unplug and replug the USB cable to the flight controller. 
5. A pop-up should come up named "Firmware Setup". Select PX4 Pro, and check advanced settings. Select "Custom firmware file..." and click Ok
6. In the file picker, navigate to `/path/to/assets/PX4-Autopilot/build/hkust_nxt-dual_default` and select the `hkust_nxt-dual_default.px4` file. 
7. You should see tabs such as "Airframe" and "Sensors" once the flash is complete. 

## Post-Flash Setup

First, ensure that the units in QGroundControl are set to SI Units: 

1. Click on the Q on the top left
2. Click on application settings
3. In the general tab under units, ensure that they are Meters, Meters, SquareMeters, Meters/second, Celsius. 


Then, follow one of the steps below: 

### Using a Pre-Saved Parameter File

1. Click on the Q button on the top left, click on Vehicle Setup, and parameters. 
2. Click the tools button and select "Load from file". 
3. Select the parameter file that can be found on the Google Drive. 

!!! warning
    This pre-saved file assumes the same settings as the Manual Setup below. If anything has changed, make sure to change it before starting. 

### Manual Setup

You can also set the settings manually. Navigate to the parameters tab, and you can search for these using the search box. Double click to edit.

#### General Settings

- If indoors, no GPS (assumed default)
    - `COM_ARM_WO_GPS`: Warning
    - `COM_ARM_MAG`: Warning
    - `SYS_HAS_MAG`, `SYS_HAS_GPS`, `SYS_HAS_BARO`: Set to 0. 
- `RC_PORT_CONFIG`: Radio Controller 
- `IMU_GYRO_RATEMAX`: 2000Hz
- `IMU_INTEG_RATE`: 400Hz
- `MAV_0_MODE`: External Vision
- `MAV_0_RATE`: 92160 B/s

#### Power

Navigate to the power tab and fill in these settings: 

- Source: Power Module
- Set the number of cells and voltage per cell to match the battery that is used. 
- Max Volt: 4.05V

#### Airframe

Under airframe, select Quadcopter X, Generic Quadcopter. Click on apply and restart. 

#### Actuators
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


These positions are relative to the center of gravity of the drone, so ensure that the battery is placed such that the CoG is truly at the middle of the drone. Then, go back to the Vehicle Setup menu and open the Actuators tab. __DO NOT TOUCH THE POSITION TEXTBOXES__ since the values will truncate to two decimal places. Ensure that the photo under "Actuator Testing" looks correct: 

![](images/actuator_setting.png)

Under PWM MAIN, select DShot600 for MAIN 1-4. Click on "Identify & Assign Motors" and follow the process. For collision safety, ensure that the Onix is located closer to the rear rotors (rotors 2 and 4 in the image). 

!!! important
    Ensure the motors are unmounted before the next step

Set the spin direction for each motor so that it matches the image. Enable the slider, slide it up a bit and ensure the rotation direction is correct. 

!!! important
    Manually verify the spin direction individually for each motor, since the spin direction may not match QGroundControl

#### Telemetry

You can choose to use either the TELEM1 or TELEM2 port for providing telemetry to the Orin. 

__TELEM2 Port (Preferred)__:

In the parameters tab, set the following settings: 

- `MAV_0_CONFIG`: TELEM 2
- `SER_TEL2_BAUD`: 921600 8N1

Go back to the home screen of QGroundControl, and click on the Q button in the top left again. Click on analyze tools, and MAVLink Console. 

In the console, run the following commands: 
```
cd fs/microsd
mkdir etc/
cd etc
echo "mavlink stream -d /dev/ttyS3 -s HIGHRES_IMU -r 1000" > extras.txt
```

If you run `cat extras.txt`, the mavlink stream line should be present. 



__TELEM1 Port__:

If the TELEM2 port is broken, you can use the TELEM1 port instead. On the parameter screen, change `MAV_0_CONFIG` to TELEM 1.


Then, open the MAVLink console under "Analyze Tools", and run the following commands: 

``` 
param set SER_TEL1_BAUD 921600
cd fs/microsd
mkdir etc/
cd etc
echo "mavlink stream -d /dev/ttyS1 -s HIGHRES_IMU -r 1000" > extras.txt
```

!!! note
    The HKUST GitHub suggests that the correct port is /dev/ttyS2 for TELEM1, but /dev/ttyS1 is tested and works. An issue has been filed [here](https://github.com/HKUST-Aerial-Robotics/Nxt-FC/issues/22)

If you run `cat extras.txt`, the mavlink stream line should be present. 



You can reboot the flight controller with `reboot` in the console. Connect the appropriate port to the THS1 port on the Onix, and SSH in. To test if the telemetry is streaming, run `python3 docs/examples/mavsdk_imu.py`. The rate should be roughly 250Hz. 


