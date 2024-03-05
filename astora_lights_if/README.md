# dmx_light_interface
Python interface for controlling one or multiple ASTORA soft box lights via DMX.

## Target Hardware
One or multiple ASTORA SF 120 SoftPanels

## Prerequisites & Installation
- Install [QLC+](https://www.qlcplus.org/) to control the lights. On Ubuntu run `sudo apt install qlcplus`. Tested on v4.12.2 on Ubuntu 20.04 and v4.12.4 on Ubuntu 22.04
- Install Python packages `pip install -r requirements.txt'

## Connecting the lights
1. Plug in the power cable to the back of the light panel(s).
2. Turn on the light using the power switch on the back of the light panel. Using the knobs on the back, adjust brightness and color temperature manually and check that everything works as expected.
3. Connect the first light panel to the PC using a **DMX-to-USB** cable. Make sure to connect this using the **DMX in** port at the back of the light panel. (We use [this](https://www.amazon.co.uk/DSD-TECH-Lighting-Equipment-Controller/dp/B07WV6P5W6?ref_=ast_sto_dp) interface from DSD TECH)
4. If you are using more than one panel, chain any additional lights to the first one with regular 3-pin DMX cables (out>in).
4. Attach a **DMX terminator** to the output DMX port of the final light panel in your chain.
6. If you want to control the chained panels inividually, use the channel dial buttons on the back of each light panel to specify separate channels. 
Every panel uses 2 channels to communicate, so set the first light to 1, the next to 3 etc...
7. If you are connecting your lights for the first time, in a terminal, navigate to **this** folder and run "python3 -c 'from astora_if import ASTORA_Lights; lights = ASTORA_Lights(num_lights=1)'". Now take a look at the QLC+ window that pops up. In the bottom menu switch to the Inputs/Outputs screen and make sure there is a device detected under 'DMX USB' and the 'output' tick box behind the entry is ticked. If no device is detected please refer to your USB dongle manual.

## Quick start
- Take a look at the usage_example.py