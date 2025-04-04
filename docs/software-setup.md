# Software Setup

This section describes how to setup the software on the host computer and the target. 

## Hardware Prerequisites 

1. Install the empty storage card, WiFi card, and the Jetson compute unit to the carrier board. Ensure that each device is correctly seated. 
2. Connect the host computer to the USB C port called "Recovery Port" on the carrier board. 
3. Provide the carrier board with an internet connection via an Ethernet dongle on one of the three host USB-C ports on the long side of the carrier board


## NVIDIA SDK 

The NVIDIA SDK is required on the host computer to flash the Jetson. First, prepare the target Jetson by running the following steps. 

1. Ensure that the devkit is turned off and the power unplugged, but ready to be plugged in
2. Ensure that the devkit is connected to the host computer via the Recovery Port
3. Press the button labelled "REC" on the Devboard
4. While pressing the REC button, press the RES button
5. Release the RES button while still pressing the REC button
6. Connect the power to the Jetson, and release the REC button after the power is connected
7. On the host computer, run `lsusb`. This sequence was a success if an entry with NVIDIA Corp. APX is visible (`ID 0955:7323 NVIDIA Corp. APX`)


Download the SDK Manager from the [NVIDIA Website](https://developer.nvidia.com/sdk-manager). Then, run the following steps: 

1. Install the SDK Manager: `sudo dpkg -i /path/to/sdkmanager.deb`. If required, install any missing dependencies with `sudo apt --fix-broken install`
2. Run the sdkmanager: `sdkmanager --cli`
3. Login, and then select the following options: install -> jetson -> target hardare
4. Select Jetson Orin NX, it should already be pre-selected
5. Reply Y to showing all Jetson versions. Select JetPack 6.2
6. Select both additional SDKs
7. Do not customize install settings
8. Reply N to flashing the Jetson Orin NX Module


### Flashing Process 

Wait until the SDK is installed, then run the following steps: 

1. Open the `~/nvidia/nvidia_sdk/JetPack_6.2_Linux_JETSON_ORIN_NX_TARGETS/Linux_for_Tegra` folder. Open a terminal in this folder. 
2. Download the post-install image from the [Google Drive](https://drive.google.com/drive/folders/1IpKJmJZyAb2P-46V7JcgBnGyn-WECAMc). Run the following command: `tar -xvf /path/to/tarball -C /home/nvidia/nvidia-sdk/JetPack_6.2_Linux_JETSON_ORIN_NX_TARGETS/Linux_for_Tegra/tools/backup_restore`. This will take a while.
3. Once this is done, run this command to flash the Jetson: `sudo ./tools/backup_restore/l4t_backup_restore.sh -e nvme0n1 -r jetson-orin-nano-devkit-nvme`. This will also take a while.
5. Press the button labelled RES on the Jetson once the command has completed. 

!!! important
    Ensure you are in the `Linux_for_Tegra` folder

# Post-Flash Setup

On the host computer, you should be able to SSH into the Orin now: `ssh lis@192.168.55.1`. This connection is provided through the USB Recovery port. These are the default credentials: 

```
username: lis
password: orin
```

[Run the ansible playbook](image-creation.md#run-ansible-playbook) to update the software and set the hostname. 

!!! important
    It is very important that you set a unique hostname. We recommend following a structured naming pattern, and recording the assigned names somewhere to avoid duplicate naming.

## Password-Free Access

See [Password-Free Access](software-common-tasks.md#password-free-access)
