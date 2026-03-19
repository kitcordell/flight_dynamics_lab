import numpy as np
import matplotlib.pyplot as plt

# Drag Polar Function

# Assumptions:
    # Steady, unaccelerated, level flight
    # 


def drag_polar(alt_0, params,):
    h = 0.01
    C_D_0 = params["C_D_0"]
    e = params["e"]
    
    rho = params["rho"]
    V_S = params["V_S"]
    V_ne = params["V_ne"]
    W = params["W"]
    S = params["S"]
    

    N = int(np.round((V_ne - V_S) / h))
    V = np.linspace(V_S, V_ne, N)

    AR = params["bw"] / params["cbar"]

    qbar = 0.5 * rho * V**2

    C_L = W / ( qbar * S ) # lift coefficient where lift equals weight
    
    C_D_i = C_L**2 / (np.pi * e * AR)
    C_D = C_D_0 + C_D_i

    D_p = qbar * S * C_D_0 
    D_i = qbar * S * C_D_i
    D = qbar * S * C_D


    

    
    return V, D, D_i, D_p




