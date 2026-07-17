import numpy as np
from flight_dynamics import aero_model, control_inputs, thrust_model
from flight_dynamics.atmosphere import standard_atmosphere
from flight_dynamics.constants import g

# Calculates derivatives of the equations of motion
# t: timespan array
# x: aircraft state
# u: control input
# params: aircraft parameters
# control_input: selected elevator input function from control_inputs.py
def aircraft_longitudinal_dynamics(
    t,
    x,
    u,
    params,
    control_input=control_inputs.elevator_deflection,
):
    U, W, Q, theta, alt = x     # forward and vertical body axis velocities, pitch angle, altitude states
    throttle, delta_e = u       # throttle and elevator deflection input states



    bw = params["bw"]                         # wing span, [ft]
    cbar = params["cbar"]                     # average chord, [ft]
    S = params["S"]                           # wing surface area [ft^2]
    AR = bw**2 / S
    # delta_e = params["delta_e"]               # elevator deflection, [rad]
    # throttle = params["throttle"]                 # total thrust in, [lb]
    # throttle = 0.45


    I_yy = params["I_yy"]                     # moment of inertia about pitch axis, [slug*ft^2]
    m = params["W"] / g                       # aircraft mass, [slugs]

    rho, _, _ = standard_atmosphere(alt)   # air density calculation

#%% Calculate States in the freestream axis
    V = np.sqrt(U**2+W**2)                  #  TAS relative to the freestream [ft/s]
    qbar = 0.5 * rho * V**2                 # dynamic pressure, [lb/ft^2]
    alpha = np.arctan2(W,U)                 # AOA

#%% Aicraft Control Inputs

    # Apply the elevator input function selected by the calling script
    delta_e = control_input(t, delta_e)
    thrust = thrust_model.thrust_piston_na(throttle, V, alt, params)
#%% Calculate forces and moments
    C_L, C_D,_ , C_m = aero_model.aero_coefficients(alpha, delta_e, Q, V, params)
    
    L = qbar * S * C_L # total lift
    D = qbar * S * C_D  # total drag
    M = qbar * S * cbar * C_m   # total pitching moment

    
    X = -D * np.cos(alpha) + L * np.sin(alpha) + thrust # X forces
    Z = -D * np.sin(alpha) - L * np.cos(alpha)  # Z forces
    

#%% Equations of motion
    xdot = np.zeros_like(x)     # initialize derivative array

    xdot[0] = X/m - g*np.sin(theta) - Q*W   # # U_dot
    xdot[1] = Z/m + g*np.cos(theta) + Q*U   # W_dot
    xdot[2] = M/I_yy    # Q_dot
    xdot[3] = Q # theta_dot
    xdot[4] = U * np.sin(theta) - W * np.cos(theta) # h_dot

    

    return xdot
