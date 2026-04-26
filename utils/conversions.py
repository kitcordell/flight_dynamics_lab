import numpy as np
from config import constants
from utils.standard_atmosphere import standard_atmosphere
from utils.speed_of_sound import speed_of_sound

def kts2fps (kts):
    fps = kts * 1.68781 # convert knots to feet per second
    return fps

def fps2kts (fps):
    kts = fps / 1.68781 # convert feet per second to knots
    return kts

def hp2ftlbfps (hp):
    ftlbfps = hp * 550.0 # convert horsepower to foot-pounds per second
    return ftlbfps

#%% Indicated airspeed to true airspeed conversion

def ias2tas(ias, press_alt, T):
    standard_atmosphere(press_alt)
    rho, T, p = standard_atmosphere(press_alt)
       # Static pressure from pressure altitude (troposphere ISA)
    

    # Impact pressure qc from CAS at sea-level standard conditions
    # Derived from compressible pitot relation
    a0 = speed_of_sound(constants.T_0) # speed of sound at sea level standard conditions
    term0 = 1 + (constants.gamma - 1) / 2 * (ias / a0) ** 2
    qc = constants.p_0 * (term0 ** (constants.gamma / (constants.gamma - 1)) - 1)

    # Mach number from qc/p at altitude
    M = np.sqrt(
        (2 / (constants.gamma - 1)) *
        ((qc / p + 1) ** ((constants.gamma - 1) / constants.gamma) - 1)
    )

    # Local speed of sound from actual temperature
    a = speed_of_sound(T)

    # TAS
    V_tas = M * a

    return V_tas
