# Infrastructure Setup 

These instructions need to be followed once to setup the infrastructure to setup and operate the drones. 

## Networking

We recommend using the Ubiquiti Amplifi configured with the following settings (*verify these*): 

- Dual Band Enabled
- Band optimization enabled
- WPA2/WPA3 PSK Authentication

Ensure you edit the ssid and password variables in the `ansible/group_vars/all` file as described in [Host Computer Setup](#host-computer-setup)

## Host Computer

We recommend having a consistent and dedicated host computer that is used to run the central controller and manage the drone swarm. The requirements of this computer are as follows: 

- A reliable Wi-Fi connection 
- CPU: Performance of i7-5600U or better 
- RAM: 8GB minimum
- Ubuntu 22.04 installed

### Ansible Installation 

On the computer, run the host computer ansible by running the following commands (if $HOME/.local/bin is already in PATH (`echo $PATH`), no need for last command):

```
sudo apt update && sudo apt install python3 python3-pip 
python3 -m pip install --user ansible
echo 'export PATH="$HOME/.local/bin:$PATH"' >> ~/.bashrc
```

For troubleshooting or more information, see the [Ansible Documentation](https://docs.ansible.com/ansible/latest/installation_guide/intro_installation.html#installing-and-upgrading-ansible-with-pip). 


### Host Computer Setup

This procedure will install required packages on the host computer and configure some networking. 

First, clone the swarm nxt [repository](https://github.com/lis-epfl/omni-nxt) in an easy to access place on the host computer, and install the requirements. 
```
sudo apt install git
git clone https://github.com/lis-epfl/swarm-nxt
```
1. Navigate to the `ansible/` folder inside the repository 
2. Edit the variables in the `ansible/group_vars/all` file
    1. `wifi_ssid`: Enter the name of the Unifi SSID
    2. `wifi_password`: Enter the name of the Unifi password
    3. `host_hostname`: Enter the string you get when you type `hostname` in the terminal
    4. `host_base_path`: Enter the base path to where all the downloaded packages/files will be added.
3. Run the host computer ansible: `ansible-playbook -i inventory.ini host_computer.yml -K`. 
4. Enter the sudo password of the host computer when prompted. 
5. Restart the PC, then check that `dialout` is there when you type `groups` in the terminal.
6. Download [STM32CubeProgrammer](https://www.st.com/en/development-tools/stm32cubeprog.html?cache=nocache#get-software) if you do not have it. 
7. Extract the folder, open it in terminal, and run `sudo ./SetupSTM32CubeProgrammer-<version>.linux` (replace `<version>` with the version you've downloaded)
8. Walk through the wizard and install the program. 
9. Download the SDK Manager from the [NVIDIA Website](https://developer.nvidia.com/sdk-manager). Install it by running `sudo dpkg -i /path/to/sdkmanager.deb`. If required, install any missing dependencies with `sudo apt --fix-broken install`


## Visual Odometry (Optitrack)

If using Optitrack as the odometry source, you must setup and configure an instance of Motive that is connected to the Optitrack system. This computer must also connect to the network described in the [networking](#networking) section via ethernet. 

This setup is out of scope for this guide. 

## Safety Bounds

!!! danger
	Take care in defining the bounds in this step to avoid drones flying where they should not fly

This package has a safety module `bounds_checker_ros2` that ensures two things: 

1. Generated trajectories stay within a polyhedron
2. If the drone exits the polyhedron, executes a land command

These bounds are defined by a `bounds.json` file that is in `ansible/templates`. Since this file is dependent on your particular environment, this file is NOT provided by default. An example file exists called `bounds.json.example` in `ansible/templates`, and is repeated here for demonstration: 

```json
[
	[ 1.0,  0.0,  0.0,  3.0],  
	[ -1.0,  0.0,  0.0,  3.0],  
	[ 0.0,   1.0,  0.0,  3.0],  
	[ 0.0,  -1.0,  0.0,  3.0],  
	[ 0.0,   0.0,  1.0,  3.6],  
	[ 0.0,   0.0,  1.0,  0.0]   
]

```
This is a 6x6x3.6m rectangular prism with the x and y axes centered on the origin and the z axis bounded from 0.0m to 3.6m.  

The format is an array of four dimensional arrays, with the first three elements defining the array normal vector, and the last element defining the offset from the origin: 

$$
\begin{bmatrix} \begin{bmatrix} \vec{n_0} & c_0 \end{bmatrix} \\ \vdots \\ \begin{bmatrix} \vec{n}_{N-1} & c_{N-1} \end{bmatrix} \end{bmatrix}
$$

Points are safe when the system of inequalities $\vec{p} \cdot \vec{n}_i - c_i \leq 0$ holds for all $i \in \{0, \dots, N-1\}$.
