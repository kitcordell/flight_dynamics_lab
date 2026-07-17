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


def lateral_aero_coefficients(beta, P, R, V_tas, delta_a, delta_r, params):
    """Calculate side-force, rolling-moment, and yawing-moment coefficients."""
    # The nondimensional angular-rate terms divide by true airspeed
    if V_tas <= 0.0:
        raise ValueError("True airspeed must be greater than zero")

    bw = params["bw"]                         # wing span, [ft]
    p_hat = P * bw / (2.0 * V_tas)            # nondimensional roll rate
    r_hat = R * bw / (2.0 * V_tas)            # nondimensional yaw rate

    # Side-force coefficient from sideslip, angular rates, aileron, and rudder
    C_Y = (
        params["C_Y_0"]
        + params["C_Y_beta"] * beta
        + params["C_Y_p"] * p_hat
        + params["C_Y_r"] * r_hat
        + params["C_Y_delta_a"] * delta_a
        + params["C_Y_delta_r"] * delta_r
    )

    # Rolling-moment coefficient from sideslip, angular rates, and controls
    C_l = (
        params["C_l_0"]
        + params["C_l_beta"] * beta
        + params["C_l_p"] * p_hat
        + params["C_l_r"] * r_hat
        + params["C_l_delta_a"] * delta_a
        + params["C_l_delta_r"] * delta_r
    )

    # Yawing-moment coefficient from sideslip, angular rates, and controls
    C_n = (
        params["C_n_0"]
        + params["C_n_beta"] * beta
        + params["C_n_p"] * p_hat
        + params["C_n_r"] * r_hat
        + params["C_n_delta_a"] * delta_a
        + params["C_n_delta_r"] * delta_r
    )

    # Return the three lateral-directional aerodynamic coefficients
    return C_Y, C_l, C_n


def lateral_aero_loads(qbar, params, C_Y, C_l, C_n):
    """Convert lateral aerodynamic coefficients into dimensional loads."""
    S = params["S"]                           # wing surface area, [ft^2]
    bw = params["bw"]                         # wing span, [ft]

    # The side-force coefficient uses wing area as its reference area
    side_force = qbar * S * C_Y                # body y-axis force, [lbf]

    # The roll and yaw coefficients use wing area and span as their references
    roll_moment = qbar * S * bw * C_l          # body x-axis moment, [lbf*ft]
    yaw_moment = qbar * S * bw * C_n           # body z-axis moment, [lbf*ft]

    return side_force, roll_moment, yaw_moment

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
