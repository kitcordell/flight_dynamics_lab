import numpy as np
from aircraft_longitudinal_dynamics import aircraft_longitudinal_dynamics
from c172_params import alt_0

#%% Longitudinal trim solver
# ADD LATER: function of throttle_trim, gamma_trim, alt, and V_trim

def aircraft_longitudinal_trim_solver(x, params):
    throttle, delta_e, theta = x

    V = params["V_trim"]          # fixed trim speed
    gamma_trim = params["gamma_trim"]
    # fixed flight-path angle
    alt = alt_0      # fixed altitude

    # Kinematic relationship
    alpha = theta - gamma_trim

    # Build body-axis velocity components
    U = V * np.cos(alpha)
    W = V * np.sin(alpha)
    Q = 0.0

    # Build state vector for the dynamics model
    x_state = np.array([U, W, Q, theta, alt])

    # Copy params and insert trim controls
    params_trim = params.copy()
    params_trim["throttle"] = throttle
    params_trim["delta_e"] = delta_e

    # Evaluate dynamics
    xdot = aircraft_longitudinal_dynamics(0.0, x_state, params_trim)

    # Trim residuals: require steady longitudinal dynamics
    return np.array([
        xdot[0],   # u_dot = 0
        xdot[1],   # w_dot = 0
        xdot[2],   # q_dot = 0
    ])



