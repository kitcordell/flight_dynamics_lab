import numpy as np
import constants

def speed_of_sound(T):
    a = np.sqrt(constants.gamma * constants.R * T) # speed of sound, [ft/s]
    return a