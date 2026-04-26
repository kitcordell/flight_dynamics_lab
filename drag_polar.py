import numpy as np
import matplotlib.pyplot as plt
from standard_atmosphere import standard_atmosphere
from thrust_model import power_available
from scipy.optimize import least_squares
# Drag Polar Function

# Assumptions:
    # Steady, unaccelerated, level flight
    # 


def drag_polar(alt_0, params,):
    h = 0.01
    C_D_0 = params["C_D_0"]
    e = params["e"]
    
    V_S = params["V_S"]
    V_ne = params["V_ne"]
    W = params["W"]
    S = params["S"]
    AR = params["AR"]

    rho, _, _ = standard_atmosphere(alt_0)
    N = int(np.round((V_ne - V_S) / h))
    V = np.linspace(V_S, V_ne, N)

    # AR = params["bw"] / params["cbar"]

    qbar = 0.5 * rho * V**2


    C_L = W / ( qbar * S ) # lift coefficient where lift equals weight
    
    C_D_i = C_L**2 / (np.pi * e * AR)
    C_D = C_D_0 + C_D_i

    D_p = qbar * S * C_D_0 
    D_i = qbar * S * C_D_i
    D = qbar * S * C_D


    

    
    return V, D, D_i, D_p


def power_required(alt_0, params):
    V, D, D_i, D_p = drag_polar(alt_0, params)

    P_req = D * V
    P_i = D_i * V
    P_p = D_p * V

    return V, P_req, P_i, P_p



def power_curves(alt, throttle, params):
    V, P_req, P_i, P_p = power_required(alt, params)

    P_A = power_available(throttle, alt, params)

    # Make it same shape as V for plotting
    P_A_curve = np.full_like(V, P_A)

    return V, P_req, P_i, P_p, P_A_curve


def velocity_max(alt, throttle, params, x0):

    def residual(V):
        _, P_req, _, _ = power_required(alt, params)
        P_A = power_available(1 ,alt, params)

        residual = P_req - P_A
        return residual
    
    sol = least_squares(residual,x0)
    return sol

from c172_params import params
