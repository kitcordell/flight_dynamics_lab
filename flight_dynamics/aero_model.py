import numpy as np
from flight_dynamics import atmosphere


def lift_coefficient(alpha, delta_e, params):
    C_L_alpha = params["C_L_alpha"]
    C_L_0 = params["C_L_0"]
    C_L_delta_e = params["C_L_delta_e"]
    
    C_L = C_L_0 + C_L_alpha * alpha + C_L_delta_e * delta_e
    return C_L

def induced_drag_coefficient(C_L, params):
    e = params["e"]
    S = params["S"]
    bw = params["bw"]
    AR = bw**2 / S
    C_D_i = C_L**2 / (np.pi * e * AR)
    return C_D_i

def drag_coefficient(C_L, params):
    C_D_0 = params["C_D_0"]
    C_D_i = induced_drag_coefficient(C_L, params)
    return C_D_0 + C_D_i, C_D_i

def moment_coefficient(alpha, delta_e, Q, V, params):
    cbar = params["cbar"]
    C_m_0 = params["C_m_0"]
    C_m_alpha = params["C_m_alpha"]
    C_m_delta_e = params["C_m_delta_e"]
    C_mq = params["C_mq"]

    C_m = C_m_0 + C_m_alpha * alpha + C_m_delta_e * delta_e + C_mq * ((Q * cbar) / (2 * V))
    return C_m

def aero_coefficients(alpha, delta_e, Q, V, params):
    C_L = lift_coefficient(alpha, delta_e, params)
    C_D, C_D_i = drag_coefficient(C_L, params)
    C_m = moment_coefficient(alpha, delta_e, Q, V, params)
    return C_L, C_D, C_D_i, C_m

def aero_loads(qbar, params, C_L, C_D, C_m):


    S = params["S"]
    cbar = params["cbar"]

    L = qbar * S * C_L
    D = qbar * S * C_D
    M = qbar * S * cbar * C_m

    return L, D, M

def air_data_from_state(x, params):
    U, W, Q, theta, alt = x

    rho = params["atmosphere"](alt)
    V = np.sqrt(U**2 + W**2)
    alpha = np.arctan2(W, U)
    qbar = 0.5 * rho * V**2

    return {
        "U": U,
        "W": W,
        "Q": Q,
        "theta": theta,
        "alt": alt,
        "rho": rho,
        "V": V,
        "alpha": alpha,
        "qbar": qbar,
    }
