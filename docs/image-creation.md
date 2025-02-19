# Image Creation

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


## Password-Free Access

You must set this up before running the playbook below. See [Password-Free Access](software-common-tasks.md#password-free-access)

## Run Ansible Playbook

We use [Ansible](https://www.redhat.com/en/ansible-collaborative) to facilitate idempotent configuration. 

Make sure Ansible is installed following the [setup instructions](software-common-tasks.md#ansible-installation)

Then, clone the [repo](https://github.com/lis-epfl/onix-nxt), and open a terminal in the ansible directory. To make the following process faster, download the patched-kernel tarball from the [Google Drive](https://drive.google.com/drive/u/1/folders/1XL-hTVf6IsB96XvfQLSesLO4FHOVjW6y). Place this file in a folder called `ansible/patched-kernel` inside the repo. 

Ensure you are in the `ansible/` folder of the repository. Run `ansible-playbook -i inventory.ini drone-setup.yml -K`. It will ask you for a BECOME password. It is the root password of the orin (`orin`). There should be no failed steps.

!!! note
    The post install image was created after this step. 


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


