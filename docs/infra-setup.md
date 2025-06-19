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

On the computer, run the host computer ansible by running the following commands:

```
sudo apt update && sudo apt install python3 
python3 -m pip install --user ansible
```

For troubleshooting or more information, see the [Ansible Documentation](https://docs.ansible.com/ansible/latest/installation_guide/intro_installation.html#installing-and-upgrading-ansible-with-pip). 


### Host Computer Setup

This procedure will install required packages on the host computer and configure some networking. 

1. Clone the swarm nxt [repository](https://github.com/lis-epfl/omni-nxt) in an easy to access place on the host computer. 
2. Navigate to the `ansible/` folder inside the repository
3. Edit the variables in the `ansible/group_vars/all` file
	1. `wifi_ssid`: Enter the name of the Unifi SSID
	2. `wifi_password`: Enter the name of the Unifi password
4. Run the host computer ansible: `ansible-playbook -i inventory.ini host_computer.yml -K`. 
5. Enter the sudo password of the host computer when prompted. 
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
	[ 0.0838,  -0.9965,  0.0,  3.0029],  
	[ 0.9990,  -0.0439,  0.0,  2.4217],  
	[-0.0164,   0.9999,  0.0,  3.9568],  
	[-0.9999,  -0.0138,  0.0,  4.0459],  
	[ 0.0,      0.0,     1.0,  3.6   ],  
	[ 0.0,      0.0,     1.0,  0.0   ]   
]
```


The format is an array of four dimensional arrays, with the first three elements defining the array normal vector, and the last element defining the offset from the origin: 

$$
\begin{bmatrix} \begin{bmatrix} \vec{n_1} & c_1 \end{bmatrix} \\ \vdots \\ \begin{bmatrix} \vec{n_N} & c_N \end{bmatrix} \end{bmatrix}
$$

Points are safe when the system of inequalities $\vec{p} \cdot \vec{n}_i + c_i \geq 0$ holds for all $i \in \{1, \dots, N\}$.
