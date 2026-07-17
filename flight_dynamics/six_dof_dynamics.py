import numpy as np

from flight_dynamics import aero_model, control_inputs, thrust_model
from flight_dynamics.atmosphere import standard_atmosphere
from flight_dynamics.constants import g


# Calculates derivatives of the six-degree-of-freedom equations of motion
# t: time
# x: aircraft state [U, V, W, P, Q, R, phi, theta, psi, north, east, altitude]
# u: control input [throttle, elevator, aileron, rudder]
# params: aircraft parameters
# surface input functions: selected functions from control_inputs.py
def aircraft_six_dof_dynamics(
    t,
    x,
    u,
    params,
    elevator_input=control_inputs.elevator_deflection,
    aileron_input=control_inputs.neutral_aileron_deflection,
    rudder_input=control_inputs.neutral_rudder_deflection,
):
    # Body-axis velocities, angular rates, Euler angles, and inertial position states
    U, V, W, P, Q, R, phi, theta, psi, north, east, alt = x

    # Throttle and control-surface inputs
    throttle, delta_e, delta_a, delta_r = u

    # Aircraft geometry
    bw = params["bw"]                         # wing span, [ft]
    cbar = params["cbar"]                     # average chord, [ft]
    S = params["S"]                           # wing surface area, [ft^2]

    # Aircraft mass properties
    I_xx = params["I_xx"]                     # roll moment of inertia, [slug*ft^2]
    I_yy = params["I_yy"]                     # pitch moment of inertia, [slug*ft^2]
    I_zz = params["I_zz"]                     # yaw moment of inertia, [slug*ft^2]
    I_xz = params["I_xz"]                     # roll-yaw product of inertia, [slug*ft^2]
    m = params["W"] / g                       # aircraft mass, [slugs]

    rho, _, _ = standard_atmosphere(alt)       # air density at the current altitude

#%% Calculate states in the freestream axis
    V_tas = np.sqrt(U**2 + V**2 + W**2)        # total true airspeed, [ft/s]
    qbar = 0.5 * rho * V_tas**2                # dynamic pressure, [lb/ft^2]
    alpha = np.arctan2(W, U)                   # angle of attack, [rad]
    beta = np.arctan2(V, np.sqrt(U**2 + W**2)) # sideslip angle, [rad]

#%% Aircraft control inputs
    # Apply the elevator, aileron, and rudder functions selected by the caller
    delta_e = elevator_input(t, delta_e)
    delta_a = aileron_input(t, delta_a)
    delta_r = rudder_input(t, delta_r)

    # Thrust acts along the positive body x-axis
    thrust = thrust_model.thrust_piston_na(throttle, V_tas, alt, params)

#%% Calculate aerodynamic forces and moments
    # Longitudinal lift, drag, and pitching-moment coefficients
    C_L, C_D, _, C_m = aero_model.aero_coefficients(
        alpha,
        delta_e,
        Q,
        V_tas,
        params,
    )

    # Lateral side-force, rolling-moment, and yawing-moment coefficients
    C_Y, C_l, C_n = aero_model.lateral_aero_coefficients(
        beta,
        P,
        R,
        V_tas,
        delta_a,
        delta_r,
        params,
    )

    # Convert aerodynamic coefficients into dimensional forces
    L = qbar * S * C_L                  # total lift, [lbf]
    D = qbar * S * C_D                  # total drag, [lbf]
    Y = qbar * S * C_Y                  # body-axis side force, [lbf]

    # Convert moment coefficients into dimensional moments
    L_roll = qbar * S * bw * C_l        # rolling moment, [lbf*ft]
    M_pitch = qbar * S * cbar * C_m     # pitching moment, [lbf*ft]
    N_yaw = qbar * S * bw * C_n         # yawing moment, [lbf*ft]

    # Resolve lift and drag into the body x-z axes and add thrust
    X = -D * np.cos(alpha) + L * np.sin(alpha) + thrust # body x-axis force, [lbf]
    Z = -D * np.sin(alpha) - L * np.cos(alpha)          # body z-axis force, [lbf]

#%% Equations of motion
    # I_xz couples the roll and yaw accelerations, so they are solved together
    roll_yaw_inertia = np.array([
        [I_xx, -I_xz],
        [-I_xz, I_zz],
    ])

    # Applied roll and yaw moments after removing gyroscopic coupling terms
    roll_yaw_rhs = np.array([
        L_roll - (I_zz - I_yy) * Q * R + I_xz * P * Q,
        N_yaw - (I_yy - I_xx) * P * Q - I_xz * Q * R,
    ])

    P_dot, R_dot = np.linalg.solve(roll_yaw_inertia, roll_yaw_rhs) # roll and yaw acceleration

    # Pitch acceleration including roll-yaw inertial coupling
    Q_dot = (
        M_pitch
        - (I_xx - I_zz) * P * R
        - I_xz * (P**2 - R**2)
    ) / I_yy                                      # pitch acceleration, [rad/s^2]

    xdot = np.zeros_like(x, dtype=float)           # initialize derivative array

    # Body-axis translational equations
    xdot[0] = X / m - g * np.sin(theta) + R * V - Q * W                 # U_dot
    xdot[1] = Y / m + g * np.sin(phi) * np.cos(theta) + P * W - R * U # V_dot
    xdot[2] = Z / m + g * np.cos(phi) * np.cos(theta) + Q * U - P * V # W_dot

    # Body-axis angular accelerations
    xdot[3] = P_dot                         # P_dot
    xdot[4] = Q_dot                         # Q_dot
    xdot[5] = R_dot                         # R_dot

    # Euler-angle kinematics
    xdot[6] = P + np.tan(theta) * (Q * np.sin(phi) + R * np.cos(phi)) # phi_dot
    xdot[7] = Q * np.cos(phi) - R * np.sin(phi)                       # theta_dot
    xdot[8] = (Q * np.sin(phi) + R * np.cos(phi)) / np.cos(theta)     # psi_dot

    # North velocity from the body-to-inertial rotation
    xdot[9] = (
        U * np.cos(theta) * np.cos(psi)
        + V * (
            np.sin(phi) * np.sin(theta) * np.cos(psi)
            - np.cos(phi) * np.sin(psi)
        )
        + W * (
            np.cos(phi) * np.sin(theta) * np.cos(psi)
            + np.sin(phi) * np.sin(psi)
        )
    )                                           # north_dot

    # East velocity from the body-to-inertial rotation
    xdot[10] = (
        U * np.cos(theta) * np.sin(psi)
        + V * (
            np.sin(phi) * np.sin(theta) * np.sin(psi)
            + np.cos(phi) * np.cos(psi)
        )
        + W * (
            np.cos(phi) * np.sin(theta) * np.sin(psi)
            - np.sin(phi) * np.cos(psi)
        )
    )                                           # east_dot

    # Altitude rate from the vertical part of the body-to-inertial rotation
    xdot[11] = (
        U * np.sin(theta)
        - V * np.sin(phi) * np.cos(theta)
        - W * np.cos(phi) * np.cos(theta)
    )                                           # altitude_dot, positive upward

    return xdot
