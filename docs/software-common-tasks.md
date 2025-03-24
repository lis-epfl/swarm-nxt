# Common Software Tasks

## Ansible Installation 

Install ansible on the host computer: `python3 -m pip install --user ansible`. For more information, see the [Ansible Documentation](https://docs.ansible.com/ansible/latest/installation_guide/intro_installation.html#installing-and-upgrading-ansible-with-pip)

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
password "password"
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

`nmcli con up <Name>`
## Wireless Connection to QGroundControl

This is done with mavros. This should be done with the preflight Ansible. You can inspect the script in `ansible/scripts/mavros_start.sh` and the lines related to Mavros in `ansible/drones_preflight.yml`. 

## Get IP of Computer

You can get the internet protocol address of a Linux computer by running `ifconfig` and looking for the four numbers after `inet` under the interface that you care about. Wireless interfaces typically begin with `w`, and ethernet interfaces typically begin with `e`. 

On most networks, you can usually use `hostname.local` in place of the IP address if the two computers are on the same network. Replace `hostname` with the hostname of the computer. You can get the hostname by running: `hostname` or `hostnamectl` on the computer. 
