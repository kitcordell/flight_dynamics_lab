import numpy as np

from flight_dynamics import aero_model, control_inputs
from flight_dynamics.atmosphere import standard_atmosphere
from flight_dynamics.constants import g


# Calculates derivatives of the lateral-directional equations of motion
# t: time
# x: lateral state [V, P, R, phi, theta, psi, altitude]
# u: control input [aileron deflection, rudder deflection]
# params: aircraft parameters
# longitudinal_state: trimmed longitudinal state [U, W, Q, theta, altitude]
# aileron_input and rudder_input: selected functions from control_inputs.py
def aircraft_lateral_dynamics(
    t,
    x,
    u,
    params,
    longitudinal_state,
    aileron_input=control_inputs.neutral_aileron_deflection,
    rudder_input=control_inputs.neutral_rudder_deflection,
):
    # Lateral states
    V, P, R, phi, theta, psi, alt = x     # side velocity, body rates, Euler angles, altitude

    # Lateral control inputs
    delta_a, delta_r = u                  # aileron and rudder deflection, [rad]

    # Apply the aileron and rudder input functions selected by the caller
    delta_a = aileron_input(t, delta_a)
    delta_r = rudder_input(t, delta_r)

    # The reduced lateral model holds U, W, and Q at their longitudinal trim values
    # Theta and altitude remain in the lateral state so their kinematics can be integrated
    U, W, Q, _, _ = longitudinal_state

    # Aircraft mass properties
    I_xx = params["I_xx"]                 # roll moment of inertia, [slug*ft^2]
    I_yy = params["I_yy"]                 # pitch moment of inertia, [slug*ft^2]
    I_zz = params["I_zz"]                 # yaw moment of inertia, [slug*ft^2]
    I_xz = params["I_xz"]                 # roll-yaw product of inertia, [slug*ft^2]
    m = params["W"] / g                   # aircraft mass, [slugs]

    rho, _, _ = standard_atmosphere(alt)   # air density at the current altitude

#%% Calculate lateral aerodynamic states
    V_tas = np.sqrt(U**2 + V**2 + W**2)        # total true airspeed, [ft/s]
    qbar = 0.5 * rho * V_tas**2                # dynamic pressure, [lb/ft^2]
    beta = np.arctan2(V, np.sqrt(U**2 + W**2)) # sideslip angle, [rad]

#%% Calculate lateral forces and moments
    # Calculate the nondimensional aerodynamic coefficients
    C_Y, C_l, C_n = aero_model.lateral_aero_coefficients(
        beta,
        P,
        R,
        V_tas,
        delta_a,
        delta_r,
        params,
    )

    # Convert the lateral coefficients into dimensional aerodynamic loads
    side_force, roll_moment, yaw_moment = aero_model.lateral_aero_loads(
        qbar,
        params,
        C_Y,
        C_l,
        C_n,
    )

#%% Equations of motion
    # I_xz couples the roll and yaw accelerations, so they are solved together
    roll_yaw_inertia = np.array([
        [I_xx, -I_xz],
        [-I_xz, I_zz],
    ])

    # Remove the gyroscopic coupling terms from the applied moments
    roll_yaw_rhs = np.array([
        roll_moment - (I_zz - I_yy) * Q * R + I_xz * P * Q,
        yaw_moment - (I_yy - I_xx) * P * Q - I_xz * Q * R,
    ])
    P_dot, R_dot = np.linalg.solve(roll_yaw_inertia, roll_yaw_rhs) # roll and yaw acceleration

    xdot = np.zeros_like(x, dtype=float)       # initialize derivative array

    # Body-axis lateral translation
    xdot[0] = side_force / m + g * np.sin(phi) * np.cos(theta) + P * W - R * U # V_dot

    # Body-axis angular accelerations
    xdot[1] = P_dot                           # P_dot
    xdot[2] = R_dot                           # R_dot

    # Euler-angle kinematics
    xdot[3] = P + np.tan(theta) * (Q * np.sin(phi) + R * np.cos(phi)) # phi_dot
    xdot[4] = Q * np.cos(phi) - R * np.sin(phi)                       # theta_dot
    xdot[5] = (Q * np.sin(phi) + R * np.cos(phi)) / np.cos(theta)     # psi_dot

    # Altitude rate, positive upward
    xdot[6] = (
        U * np.sin(theta)
        - V * np.sin(phi) * np.cos(theta)
        - W * np.cos(phi) * np.cos(theta)
    )                                           # altitude_dot

    return xdot
