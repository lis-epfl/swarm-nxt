# Common Software Tasks

The various subheadings of this document are linked in other pages. This page is meant as a reference for commonly performed tasks.

## Password-Free Access

For password free access to the Orin, follow these steps: 

1. Ensure that you have an SSH key in ~/.ssh
    1. If not, run `ssh-keygen -t ed25519 -C <yourCommentHere>`
2. With the USB connected, run `ssh-copy-id lis@192.168.55.1`
3. Make sure that the ssh-agent is on: ``eval `ssh-agent` `` 
4. Add your key to the agent: `ssh-add`
5. Ensure that you can access the device without a password: `ssh lis@192.168.55.1`

## Wi-Fi Setup

SSH into the Orin. The following is a reference for commands that you can use with `nmcli`: 

To see available Wi-Fi networks: `nmcli device wifi list`

To connect to a network with simple auth (open or WPA2/WPA3 password): `sudo nmtui` -> Activate Connection, or: 

```shell
sudo nmcli connection add type wifi \ 
con-name "NetworkName" \
ssid "SSID" \
wifi-sec.psk "password" \
wifi-sec.key-mgmt "wpa-psk"
```

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

To activate a connection, you can either use `sudo nmtui`, or: 

`nmcli con up <Name>
## Wireless Connection to QGroundControl

This is done with mavros. This should be done with the preflight Ansible. You can inspect the script in `ansible/scripts/mavros_start.sh` and the lines related to Mavros in `ansible/drones_preflight.yml`. 

## Get IP of Computer

You can get the internet protocol address of a Linux computer by running `ifconfig` and looking for the four numbers after `inet` under the interface that you care about. Wireless interfaces typically begin with `w`, and ethernet interfaces typically begin with `e`. 

On most networks, you can usually use `hostname.local` in place of the IP address if the two computers are on the same network. Replace `hostname` with the hostname of the computer. You can get the hostname by running: `hostname` or `hostnamectl` on the computer. 

### Rescue Unreachable Device

If you do not know the IP address of a drone, you can try these steps to find it's IP: 

1. Try restarting everything first. 
2. Try running `avahi-resolve -n hostname.local` while on the same network. 
3. Try running `avahi-resolve -n hostname-2.local`. Sometimes, the original hostname gets reserved and this becomes a fallback. If this works, you can run `avahi-publish` in the background with the original hostname to force it to work. 
4. Connect to the recovery port using a USB-C cable. You should be able to SSH into the device at `192.168.55.1` and use `ifconfig` to find the IP.
	1. Then, you can use `avahi-resolve -a <ip>` to find the mDNS hostname. 

## VNC/Remote Desktop setup

On the host computer, go to terminal and type `gvncviewer lis@nxt1.local` where nxt1 is the hostname of the Orin on which is on the same network as the host computer. Then input the password 'orin'. This will allow you do remote desktop into the Orin. You can also replace `nxt1.local` with the full IP address of the orin e.g. `gvncviewer lis@192.168.55.1`.

## Check EKF Tracking

To check EKF tracking, perform the following steps:

1. On the host computer, make sure ros is sourced: `source /opt/ros/humble/setup.bash`
2. Ensure the `ROS_DOMAIN_ID` is set to the same value as the drones. By default, this is 1. You can check in `ansible/group_vars/all`: `export ROS_DOMAIN_ID=<domain id>`
3. Run plotjuggler: `ros2 run plotjuggler plotjuggler`
4. In plotjuggler, add:
	1. `/mavros/local_position/pose`  xyz, orientation quaternions (EKF Output)
	2. `/mavros/vision_pose/pose_cov` xyz, orientation quaternions (optitrack)
5. Move the drone around in all directions and ensure there are no discontinuities, and the values are tracking each other. 
6. Rotate the drone in different directions and ensure the values are tracking each other
