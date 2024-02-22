from astora_if import ASTORA_Lights

# Connect to the lights
    # Make sure they are connected and switched on
    # The QLC+ server will be launched automatically, make sure not to close that window
    # Specify how many lights you want to control independently of each other. 
    # If you have multiple lights, you can also control them all together with the same settings by specifying num_lights=1
lights = ASTORA_Lights(num_lights=1)

# Get the current light parameters
print(lights.get_light_parameters())

# Set the light parameters
    # specify two arrays, one for intensity and one for colour temparature
    # the arrays must have the same length, one value per light 
    # the values must be between 0 and 255
    # if you are unsure about the input shape, look at the output of get_light_parameters()
lights.set_light_parameters([0],[0]) # lights switched off
lights.set_light_parameters([255],[0]) # lights switched on at full intensity and low colour temperature
lights.set_light_parameters([255],[255]) # lights switched on at full intensity and high colour temperature

# Fade
    # specify a duration in seconds
    # the lights will fade to the new settings over the specified duration
    # the smoothness of the fade is limited by the frequency of the websocket connection and the connected hardware
lights.set_light_parameters([0],[0],duration=5) # lights fading to off over 5 seconds

# Disconnect from the lights
lights.disconnect()
