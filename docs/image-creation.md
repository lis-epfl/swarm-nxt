# Software Setup

This section describes how to setup the software on the host computer and the target. We use [Ansible](https://www.redhat.com/en/ansible-collaborative) to facilitate idempotent configuration. 

## Hardware Setup 

1. Install the empty storage card, WiFi card, and the Jetson compute unit to the carrier board. Ensure that each device is correctly seated. 
2. Connect the host computer to the USB C port called "Recovery Port" on the carrier board. 
3. Provide the carrier board with an internet connection via an Ethernet dongle on one of the three host USB-C ports on the long side of the carrier board

## Raw Image Setup 

These steps describe how to build the intial image for the NVIDIA Orin board. These steps are only necessary if you want to rebuild the raw image. 

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
4. The Orin NX should already be detected, select it. 
5. Reply Y to showing all Jetson versions. Select JetPack 6.2
6. Additional SDKs: <!-- TODO: which ones? -->
7. Customize install settings: Y
6. Select all the options by using the arrow keys and space to select, enter to continue
7. Reply Y to "Do you want to flash Jetson Orin NX module?". Select Pre-Config for OEM Config. 
10. Username: lis, Password: orin
8. Choose NVME for the storage device
11. Wait for the installation to finish. 
12. Proxy mode
9. Select the IP that you want the device to have. This documentation assumes `192.168.55.1`


!!! warning
    If you get an error like "cannot stat /dev/nvme0n1", ensure that the flash storage is fully seated.

The raw image was created after these steps. 1


## Password-Free Access

For password free access to the Orin, follow these steps: 

1. Ensure that you have an SSH key in ~/.ssh 
2. With the USB connected, run `ssh-copy-id lis@192.168.55.1`
3. Make sure that the ssh-agent is on: ``eval `ssh-agent` `` 
4. Add your key to the agent `ssh-add`
5. Ensure that you can access the device without a password: `ssh lis@192.168.55.1`

## Set Hostname

For easier reference to the device, you can set the hostname per-device by ssh'ing in and running `sudo hostnamectl hostname nxt<num>`

We track the hostnames that have been assigned in [this](https://docs.google.com/spreadsheets/d/1uuXMoGTHYXsHLR0sF3Sd2kVL-pUlbTp2RO5qc1V2LAo/edit) spreadsheet. 


## Ansible Installation 

## Backups and Restores

Common: 

-  On the host computer with the NVIDIA SDK installed, navigate to `~/nvidia/nvidia_sdk/JetPack_6.2_Linux_JETSON_ORIN_NX_TARGETS/Linux_for_Tegra`
### Create Image Procedure

1. Prepare the device by installing what is required
2. Cleanup any log files and unnecessary files to make the image compact
3. Restart the device in recovery mode (follow the steps in [Raw Image Setup](#raw-image-setup))
5. Run `sudo ./tools/backup_restore/l4t_backup_restore.sh -e nvme0n1 -b jetson-orin-nano-devkit-nvme`
6. Compress the image folder: `cd tools/backup_restore && sudo tar -cvzf <name>.tar.gz images`
7. Save the compressed image for restore later. 

### Restore Procedure
1. Ensure that the image you want to restore is unzipped in tools/backup_restore/ (There should be a directory called `images` with various .img files inside)
2. Ensure you are in the `Linux_for_Tegra` folder. 
3. Restart the device in recovery mode (follow the steps in [Raw Image Setup](#raw-image-setup))
4. Run `sudo ./tools/backup_restore/l4t_backup_restore.sh -e nvme0n1 -r jetson-orin-nano-devkit-nvme`


