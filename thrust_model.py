import numpy as np
from standard_atmosphere import standard_atmosphere
from constants import rho_0

#%% Naturally aspirated piston engine thrust approximation
    # Inputs:
    #   throttle: throttle setting (0 to 1)
    #   V: velocity of the aircraft (ft/s)
    #   alt: altitude (ft)
    #   params: dictionary containing engine parameters
    # Thrust decreases proportionally with altitude and propellor efficiency coefficient
def thrust_piston_na(throttle, V, alt, params):

    if throttle < 0.0 or throttle > 1.0:
        raise ValueError("Throttle must be between 0 and 1")
    else:
        P_max_SL = params["P_max_SL"]  # maximum power at sea level [lb*ft/s]
        eta_p = params["eta_p"]            # propeller efficiency
        rho, _, _ = standard_atmosphere(alt)

        
        P_A = eta_p * P_max_SL * throttle * rho / rho_0 # available power at altitude [lb*ft/s]
        T = P_A / V



    

    return T


def power_available(throttle, alt, params):
    if throttle < 0.0 or throttle > 1.0:
        raise ValueError("Throttle must be between 0 and 1")

    P_max_SL = params["P_max_SL"]
    eta_p = params["eta_p"]

    rho, _, _ = standard_atmosphere(alt)

    P_A = eta_p * P_max_SL * throttle * rho / rho_0

    return P_A