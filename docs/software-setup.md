#Software Setup

This section describes how to setup the software on the host computer and the target. We use [Ansible](https://www.redhat.com/en/ansible-collaborative) to facilitate idempotent configuration. 

## Hardware Setup 

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
2. Download the post-install image from the [Google Drive](https://drive.google.com/drive/u/1/folders/1IpKJmJZyAb2P-46V7JcgBnGyn-WECAMc). Run the following command: `tar -xvf /path/to/tarball -C /home/nvidia/nvidia-sdk/JetPack_6.2_Linux_JETSON_ORIN_NX_TARGETS/Linux_for_Tegra/tools/backup_restore`. This will take a while.
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

You can [run the ansible playbook](image-creation.md#ansible-installation) to update the software if needed. 

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

## Wi-Fi Setup

SSH into the Orin. The following is a reference for commands that you can use with `nmcli`: 

To see available Wi-Fi networks: `nmcli device wifi list`

To connect to a network with simple auth (open or WPA2/WPA3 password): `sudo nmtui` -> Activate Connection

To connect to a more complicated network (eduroam/epfl): 
```bash
sudo nmcli connection add type wifi \
con-name "epfl" \
ssid "epfl" \
wifi-sec.key-mgmt wpa-eap \
802-1x.eap ttls \
802-1x.phase2-auth mschapv2 \
802-1x.identity "YourUsername" \
802-1x.password "YourPassword" \
802-11-wireless.bssid "AA:BB:CC:DD:EE:FF"
```

Replace YourUsername, YourPassword with your GASPAR credentials. You can find the bssid of your local router by `nmcli device wifi list`. Use that instead of `AA:BB:CC:DD:EE:FF`.
