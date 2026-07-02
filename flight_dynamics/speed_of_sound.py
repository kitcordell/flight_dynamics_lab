import numpy as np
from flight_dynamics import constants

def speed_of_sound(T):
    a = np.sqrt(constants.gamma * constants.R * T) # speed of sound, [ft/s]
    return a
