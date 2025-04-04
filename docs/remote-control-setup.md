# Remote Control Setup

We use the Taranis Remote Control. Download the latest version of OpenTX for your RC from [their website](https://www.open-tx.org/downloads). To install it, unzip the files into the root directory of the MicroSD card, and plug the MicroSD card into the slot in the RC. The slot is above the battery. 

Ensure the TBX Nano RX is plugged into the RC slot on the flight controller. Plug in the TBS Micro TX into the controller.

## Profile Creation

Once the Taranis has booted up, click on Menu. Then, follow the steps: 

1. Use the scroll wheel to the right of the screen to select an empty spot. Click and hold the scroll wheel until a menu comes up.
2. Select Create model
3. Scroll to the Quadcopter/Multi and select
4. Click page repeatedly, keeping the default channels. 
5. On the main menu, ensure you're hovering over the newly created model. Click the Page button. 
6. Press the scroll wheel to enter the name field. Use the scroll wheel to select a character. Press the scroll wheel to move to the next character. Once the name is input, press and hold the scroll wheel over a blank character to save the name. 
7. Use the scroll wheel to scroll down to "Internal RF", and ensure it is set to "None"
8. Use the scroll wheel to scroll down to "External RF", and ensure it is set to "CSRF".

## Crossfire Binding

This is only required if you're binding a new pair of transmitters and receivers. 

1. Turn the drone on, and press the small button on the Nano RX. The light should turn turn blinking green. 
2. On the Taranis, ensure you're on the home page for the desired profile. The name of the profile should be present in a large font. 
3. Press and hold menu, and use the scroll wheel to navigate down to "Crossfire config". Click on it. 
4. Click on "XF Micro TX"
5. Click on bind. It should turn to "Binding OK", and the light on the Nano RX should turn to solid green. 
6. Click Exit to go back, and then click on the "XF Nano RX V3"
7. Ensure channels is set to 12. 
8. Ensure failsafe mode is set to the desired value: "Cut" if operating a single drone, disabled if swarm <!-- FC pos? -->
9. Scroll down to "Output map" and set Output 1 to SBUS
10. Click exit until you're back at the main screen for your profile. There should be a radio link strength indicator beside the battery indicator. 

## Mixes Setting

1. Move back one level to the profile selection screen by pressing menu. Move the cursor over the profile you want to edit. 
2. Keep clicking page until mixes is seen. 
3. Keep CH1-4 as-is
4. Change the source of CH5-CH8 by clicking and holding the scroll wheel, clicking edit, navigating to source, and clicking the scroll wheel. Then, flip the appropriate switch. Click exit twice to go back to the mixes screen.

| **Channel** | **Source** | **Name**   |
|-------------|------------|------------|
| CH5         | SF         | Arm/Disarm |
| CH6         | SC         | Offboard   |
| CH7         | SD         | Modes      |
| CH8         | SA         | Kill       |


