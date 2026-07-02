import numpy as np


#%% Elevator Deflection Function
def elevator_deflection(t, delta_e):
    if t > 5.6 and t < 6.6:
        delta_e = delta_e + np.deg2rad(-2.164)

    else:

        delta_e = delta_e

    return delta_e

def neutral_elevator_deflection(t, delta_e):
    return delta_e