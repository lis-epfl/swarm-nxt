# Hardware Setup 

This file takes you through the hardware setup process for the OmniNXT Drone. The bill of materials can be found [here](https://docs.google.com/spreadsheets/d/1R3wPALCepJXdboiaurltGrkgvVYNo0OVzrUW1RJAQ7c/edit?gid=0#gid=0). 

3D print all of the necessary items that are labelled with the manufacturer "Self". 

## Flight Controller and ESC

### Soldering 

1. Insert the antivibration rubber pieces into the four holes of both the ESC and the flight controller. On the ESC, the shorter side should be on the side with the eight pin connection port.  On the flight controller, the shorter side of the rubber piece should be on the SD card side. 
2. Cut each of the motor wires to 55mm. Measure from the foot of the heatshrink.
3. Cut a orin power cable from the base of the female side: 
    1. 10cm on the red wire
    2. 9.5cm on the black wire
4. Take the XT60 Cable (CAB-004) and cut it to the following lengths: 
	1. Red: 11cm from the edge of the connector
	2. Black: 10cm from the edge of the connector 
5. Strip around 5mm from each wire of CAB-004 and add solder to each wire
6. Strip around 5mm from each wire of the orin power cable and add solder to each wire
7. Solder the red wires of CAB-004 and the orin power cable together on the positive pad of the ESC
8. Solder the black wires of CAB-004 and the orin power cable together on the negative pad of the ESC
9. Solder each of the motors to one triad of pins on the ESC. 

!!! warning
    Ensure that each motor's wires are connected to three consecutive pads on the ESC, with one of them in the corner. It does not matter which motor wire goes on which pin other than this. 


![](images/hw/motor_lengths.jpg)
![](images/hw/battery_length.jpg)



### Fastening Assembly to Frame

1. Stack the ESC and flight controller such that the big side of the rubber stoppers are touching. The MicroSD Card slot should be above the ESC power connection. 
2. Prepare the assembly: <!--TODOx2 -->
	1. With the flight controller on the bottom, insert two M2X? bolts on the side with the USB-C port. The head of the bolt should be on the same side as the flight controller
	2. Insert two M2x? bolts on the opposite side of the USB-C port. 
	3. Place the 6mm nylon spacers over all of the bolts. 
3. Orient the center of the frame over the assembly. The USB-C port should be facing towards the side if the frame with the cutout. The side with the divets for the nuts should be on the top. 
4. Loosely thread the M2 nuts onto each of the bolts 
5. For each of the bolts, hold the nut with your finger in the divet, and use a screwdriver to tighten the bolt. It should be snug, and equally tight across the four bolts. Do not overtighten, the ESC can contact the flight controller!
6. Fasten the motors to the frame with the shorter bolts that were included in the packaging. Ensure that the cables are close to the frame, you can twist them together to ensure this. 
7. Connect the RC receiver to the appropriate port on the flight controller, and tape it down to the frame as shown
   <!-- TODO: Insert picture -->


## Orin Connection 

1. Insert the NVME SSD to the corresponding port, and bolt it down with an M2x6 bolt. 
2. Attach the Wi-Fi antenna to the adapter. It does not matter which port goes into which antenna. [YouTube Video](https://youtu.be/8tzWKIt1v1E?t=40)
3. Insert the Wi-Fi adapter to the corresponding port, and bolt it down using an M2x4 bolt. 
4. Attach the carrier board and Orin to the other side of the frame as the flight controller. Use 5mm nylon spacers between the carrier board and the frame, and M3x12 bolts on the top. 
5. Remove the orange covers off of the bolt holes on the carrier board.
6. Bolt the orin down to the carrier board with 2 M2x6 Nylon bolts 
7. Remove the protective film from the double sided tape on the Wi-Fi antenna, and attach it to the front of the drone. 

## Frame Preparation 

1. Bolt (M3x8) the five oddity RC standoffs at the marked locations in the image below. They should be protruding on the side with the flight controller. 
   <!--TODO: Insert image-->
2. Bolt four M3 40mm standoffs with M3x10 bolts. These should be on the Orin side.
   <!--TODO: Insert image-->
3. Bolt the prop guard onto the Oddity RC standoffs with M3x8 bolts. 

!!! warning
	Do not install the propellers yet! They will be installed during the software setup section.  

## Camera Assembly

### Cameras

For each of the cameras, perform the following tasks: 

#### Cable Preparation

1. Insert the camera cable into the camera. The side with the pads labelled FSIN and STROBE should be on the side of the camera. 
	1. Use a flathead screwdriver to pull out the retaining clip from the connector. 
	2. Insert the cable with the exposed contacts facing the PCB, push until it hits the back of the connector
	3. Secure the cable by pushing the retaining clip back in
2. Cut the microwire to a length of 30mm. 
3. Scratch around 4mm of coating off of both ends. The colour should change, and you can check with a magnifying glass or microscope. 
4. Apply some solder to both sides of the microwire
5. Solder one side of the microwire to the port labelled `F` on the camera board. The wire should come out on the opposite side of the camera.
6. Solder the other side to the pad lablled `FSIN` on the cable. 
7. Use the Kapton tape to secure the microwire, ensuring that the tape is providing strain relief for the solder connection

#### Lens Switch

1. Unscrew the lens that the camera ships with, and discard.
2. Unbolt the existing lens mount. 
3. Bolt the lens mount that came with the fisheye lens. 
4. Screw in the fisheye lens into the mount. 

#### Test and Focus

!!! note
	This step can be done anytime. However, it is suggested to test the camera setup now before things are bolted in more permanently. 

1. Insert all of the camera cables into the OAK camera board.
2. Plug in the OAK camera board via USB-C to a laptop computer with ROS installed
3. Download and install the driver package: 
4. Check for focus on each of the cameras, and change the focus by turning the lenses on their holders. 
<!--5. If continuing in this order, disconnect the cameras from the OAK camera board.-->

### Camera Mounts

!!! warning
	Be careful not to tug on the cables too much. They can tear.

1. Use a soldering iron at 215℃ to insert the M2 threaded inserts into the six holes of each camera mount
2. Bolt each camera to a mount by using four M2x? bolts. The cables should be exiting towards the flange on the mount. <!--TODO-->
3. Bolt the camera mounts to the camera holder frame. 

### OAK Board

1. Connect the four camera cables to the OAK board. Ensure that the orientation of the board is as it would be in the final assembly, and do not allow any camera cable to be twisted. 
2. Put the two halves of the cover around the OAK board, and bolt the assembly to the frame with 4 M2x14 bolts. 
3. Put the camera frame on the metal standoffs above the orin, and bolt down with four M3x6 bolts.
