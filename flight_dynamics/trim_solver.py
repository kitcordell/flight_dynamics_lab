
#%%
import numpy as np
from scipy.optimize import least_squares
from flight_dynamics.c172_params import params
from flight_dynamics.lateral_dynamics import aircraft_lateral_dynamics
from flight_dynamics.longitudinal_dynamics import aircraft_longitudinal_dynamics
from flight_dynamics.six_dof_dynamics import aircraft_six_dof_dynamics
from flight_dynamics.axis_transformations import velocity_to_body


#%%
## Build Trim States
    # Solves for trim conditions given a fixed velocity, flight path angle, and altitude.
    # Builds the desired state vector for trim conditions from flight path axis to body axis
def build_trim_states(V, gamma, alt, theta):


    U, W, alpha = velocity_to_body(V,gamma,theta)
    Q = 0.0  # pitch rate, [rad/s]

    x = np.array([U, W, Q, theta, alt])
    return x

## Trim Residuals
    # Separates what values are inputted and what is solved for into target and unknown values to build the trim states
    # Also separates them into inputs and outputs for the dynamics function
def level_trim_residuals(unknown, trim_target):

    # Build States
    V, gamma, alt = trim_target                 # desired states to be trimmed for (inputs)
    throttle, delta_e, theta = unknown    # unknown states to solve for (guesses)
        
    # Calculate Dynamics
    x = build_trim_states(V, gamma, alt, theta) # state vector
    u = np.array([throttle, delta_e])   # control vector

    xdot_desired = np.zeros_like(x)     # set desired derivatives to zero
    xdot_desired[4] = V * np.sin(gamma) # set desired hdot as a function of gamma



        

    xdot_actual = aircraft_longitudinal_dynamics(0.0, x, u, params)    
    residual = xdot_actual - xdot_desired
    return residual

# x0: initial guesses for [throttle [%], elevator deflection [rad], theta [rad]]
# trim_target: target values for [velocity [ft/s], flight path angle [rad], altitude [ft]]
def longitudinal_trim(x0, trim_target):
    
    sol = least_squares(level_trim_residuals, x0, bounds=([0.0, -np.inf, -np.inf], [1.0, np.inf, np.inf]), args=(trim_target,)) # Solve Non-linear equations
    
    # unpacking for printing
    
    V_trim, gamma_trim, alt_trim = trim_target # unpack trim conditions
    throttle_trim, delta_e_trim, theta_trim = sol.x  # unpack solution
    
    x = build_trim_states(V_trim, gamma_trim, alt_trim, theta_trim)  # convert back to body axis 
    u = np.array([throttle_trim, delta_e_trim])
    


    U_trim, W_trim, Q_trim , theta_trim, alt = x # unpack state vector


    # U, W, Q, theta, alt = x     # forward and vertical body axis velocities, pitch angle, altitude states
    # throttle, delta_e = u       # throttle and elevator deflection input states
    
    # Print inputs and outputs
    print("\nTrim Target:")
    print("Velocity:", V_trim, "[ft/s]")
    print("Flight Path angle:", gamma_trim, "[rad]")
    print("Altitude:", alt_trim, "[ft]")

    print("\nTrim Solutions:")
    print("Throttle:", throttle_trim, "[%]")
    print("Elevator Deflection", delta_e_trim, "[rad]")

   
    print("Body Angle:", theta_trim, "[rad]")
    print("Forward Body Velocity (U):", U_trim, "[ft/s]")
    print("Vertical Body Velocity (W):", W_trim, "[ft/s]")
    print("Pitch Rate (Q):", Q_trim, "[rad/s]")

    print("\nLongitudinal Trim Residuals:")
    print("U Acceleration Residual:", sol.fun[0], "[ft/s^2]")
    print("W Acceleration Residual:", sol.fun[1], "[ft/s^2]")
    print("Pitch Acceleration Residual:", sol.fun[2], "[rad/s^2]")
    print("Pitch Angle Rate Residual:", sol.fun[3], "[rad/s]")
    print("Altitude Rate Residual:", sol.fun[4], "[ft/s]")

  
    return x, u


#%%
## Build Lateral Trim States
    # Builds the seven-state vector used by the reduced lateral-directional model
    # The longitudinal trim solution supplies theta and altitude for this state
def build_lateral_trim_state(V_side, phi, heading, longitudinal_state):

    # Read the longitudinal attitude and altitude used by the lateral model
    _, _, _, theta, alt = longitudinal_state

    # A steady lateral trim condition has no roll or yaw rate
    P = 0.0 # roll rate, [rad/s]
    R = 0.0 # yaw rate, [rad/s]

    x = np.array([
        V_side, # side body velocity, [ft/s]
        P,
        R,
        phi,     # bank angle, [rad]
        theta,   # pitch angle from longitudinal trim, [rad]
        heading, # heading angle, [rad]
        alt,     # altitude from longitudinal trim, [ft]
    ])

    return x


## Lateral Trim Residuals
    # Solves for steady side force, roll moment, yaw moment, and target sideslip
    # Unknown values are [aileron, rudder, side velocity, bank angle]
def lateral_trim_residuals(unknown, longitudinal_state, trim_target, aircraft_params):

    # Separate the lateral controls and states being solved for
    delta_a, delta_r, V_side, phi = unknown

    # The caller selects the desired sideslip and heading
    beta_target, heading = trim_target

    # Build the reduced lateral state and control vectors
    x = build_lateral_trim_state(
        V_side,
        phi,
        heading,
        longitudinal_state,
    )
    u = np.array([
        delta_a,
        delta_r,
    ])

    # Calculate the lateral state derivatives at the proposed trim condition
    xdot = aircraft_lateral_dynamics(
        0.0,
        x,
        u,
        aircraft_params,
        longitudinal_state,
    )

    # Convert the requested sideslip angle into the matching side body velocity
    U_trim, W_trim, _, _, _ = longitudinal_state
    longitudinal_speed = np.sqrt(U_trim**2 + W_trim**2)
    desired_side_velocity = longitudinal_speed * np.tan(beta_target)

    # Four equations are returned for the four lateral trim unknowns
    residual = np.array([
        xdot[0],                                  # V acceleration, [ft/s^2]
        xdot[1],                                  # roll acceleration, [rad/s^2]
        xdot[2],                                  # yaw acceleration, [rad/s^2]
        V_side - desired_side_velocity,           # side velocity error, [ft/s]
    ])

    return residual


# x0: initial guesses for [aileron, rudder, side velocity, bank angle]
# longitudinal_state: trimmed state [U, W, Q, theta, altitude]
# trim_target: target values for [sideslip angle, heading]
def lateral_trim(x0, longitudinal_state, trim_target, aircraft_params, verbose=True):

    # Solve all four lateral trim equations together
    sol = least_squares(
        lateral_trim_residuals,
        x0,
        args=(longitudinal_state, trim_target, aircraft_params),
        method="dogbox",
    )

    # Separate the solved controls and lateral states
    delta_a, delta_r, V_side, phi = sol.x
    beta_target, heading = trim_target

    # Rebuild the complete lateral state and control vectors
    x = build_lateral_trim_state(
        V_side,
        phi,
        heading,
        longitudinal_state,
    )
    u = np.array([
        delta_a,
        delta_r,
    ])

    # Print the lateral trim solution and every residual when requested
    if verbose:
        print("\nLateral Trim Target:")
        print("Sideslip Angle:", beta_target, "[rad]")
        print("Heading:", heading, "[rad]")

        print("\nLateral Trim Solutions:")
        print("Aileron Deflection:", delta_a, "[rad]")
        print("Rudder Deflection:", delta_r, "[rad]")
        print("Side Body Velocity (V):", V_side, "[ft/s]")
        print("Bank Angle:", phi, "[rad]")

        print("\nLateral Trim Residuals:")
        print("V Acceleration Residual:", sol.fun[0], "[ft/s^2]")
        print("Roll Acceleration Residual:", sol.fun[1], "[rad/s^2]")
        print("Yaw Acceleration Residual:", sol.fun[2], "[rad/s^2]")
        print("Side Velocity Residual:", sol.fun[3], "[ft/s]")

    return x, u, sol


#%%
## Build Six-DOF Trim States
    # Builds the complete 12-state vector used by the six-DOF model
    # The body velocities are calculated from true airspeed, angle of attack, and sideslip
def build_six_dof_trim_state(V_tas, alt, alpha, beta, phi, theta, psi):

    # Resolve the desired true airspeed into the aircraft body axes
    U = V_tas * np.cos(alpha) * np.cos(beta) # forward body velocity, [ft/s]
    V = V_tas * np.sin(beta)                 # side body velocity, [ft/s]
    W = V_tas * np.sin(alpha) * np.cos(beta) # vertical body velocity, [ft/s]

    # A steady trim condition has no body-axis angular rates
    P = 0.0 # roll rate, [rad/s]
    Q = 0.0 # pitch rate, [rad/s]
    R = 0.0 # yaw rate, [rad/s]

    # Position does not affect the trim forces, so start at the local origin
    north = 0.0 # north position, [ft]
    east = 0.0  # east position, [ft]

    x = np.array([
        U,
        V,
        W,
        P,
        Q,
        R,
        phi,
        theta,
        psi,
        north,
        east,
        alt,
    ])

    return x


## Six-DOF Trim Residuals
    # Solves the force, moment, climb-rate, and cross-track equations together
    # This lets the lateral states and controls be trimmed instead of assumed to be zero
def six_dof_trim_residuals(unknown, trim_target, aircraft_params):

    # Desired flight condition held fixed by the trim target
    V_tas, gamma, alt, heading = trim_target

    # States and controls that the nonlinear solver is allowed to change
    throttle, delta_e, delta_a, delta_r, alpha, beta, phi, theta = unknown

    # Build the 12-state vector and the complete four-control vector
    x = build_six_dof_trim_state(
        V_tas,
        alt,
        alpha,
        beta,
        phi,
        theta,
        heading,
    )
    u = np.array([
        throttle,
        delta_e,
        delta_a,
        delta_r,
    ])

    # Calculate the actual derivatives at the proposed trim condition
    xdot = aircraft_six_dof_dynamics(
        0.0,
        x,
        u,
        aircraft_params,
    )

    # The first six derivatives are the body accelerations and angular accelerations
    acceleration_residuals = xdot[:6]

    # The requested flight-path angle sets the desired positive-up altitude rate
    desired_altitude_rate = V_tas * np.sin(gamma)
    altitude_rate_residual = xdot[11] - desired_altitude_rate

    # Steady straight flight cannot have velocity perpendicular to the requested heading
    cross_track_velocity = (
        -xdot[9] * np.sin(heading)
        + xdot[10] * np.cos(heading)
    )

    # Eight equations are returned for the eight unknown trim values
    residual = np.concatenate((
        acceleration_residuals,
        np.array([
            altitude_rate_residual,
            cross_track_velocity,
        ]),
    ))

    return residual


# x0: initial guesses for [throttle, elevator, aileron, rudder, alpha, beta, phi, theta]
# trim_target: target values for [velocity, flight path angle, altitude, heading]
def six_dof_trim(x0, trim_target, aircraft_params, verbose=True):

    # Keep throttle inside its valid zero-to-one range while solving the trim equations
    lower_bounds = np.array([
        0.0,
        -np.inf,
        -np.inf,
        -np.inf,
        -np.pi / 2.0,
        -np.pi / 2.0,
        -np.pi / 2.0,
        -np.pi / 2.0,
    ])
    upper_bounds = np.array([
        1.0,
        np.inf,
        np.inf,
        np.inf,
        np.pi / 2.0,
        np.pi / 2.0,
        np.pi / 2.0,
        np.pi / 2.0,
    ])

    # Solve the nonlinear six-DOF trim equations
    sol = least_squares(
        six_dof_trim_residuals,
        x0,
        bounds=(lower_bounds, upper_bounds),
        args=(trim_target, aircraft_params),
        method="dogbox",
    )



    # Unpack the requested trim condition and the solved trim values
    V_tas, gamma, alt, heading = trim_target
    throttle, delta_e, delta_a, delta_r, alpha, beta, phi, theta = sol.x

    # Rebuild the complete state and control vectors for the dynamics model
    x = build_six_dof_trim_state(
        V_tas,
        alt,
        alpha,
        beta,
        phi,
        theta,
        heading,
    )
    u = np.array([
        throttle,
        delta_e,
        delta_a,
        delta_r,
    ])

    # Print the trim condition and solution when the caller wants solver output
    if verbose:
        print("\nSix-DOF Trim Target:")
        print("Velocity:", V_tas, "[ft/s]")
        print("Flight Path Angle:", gamma, "[rad]")
        print("Altitude:", alt, "[ft]")
        print("Heading:", heading, "[rad]")

        print("\nSix-DOF Trim Solutions:")
        print("Throttle:", throttle, "[%]")
        print("Elevator Deflection:", delta_e, "[rad]")
        print("Aileron Deflection:", delta_a, "[rad]")
        print("Rudder Deflection:", delta_r, "[rad]")
        print("Angle of Attack:", alpha, "[rad]")
        print("Sideslip Angle:", beta, "[rad]")
        print("Bank Angle:", phi, "[rad]")
        print("Pitch Angle:", theta, "[rad]")

        print("\nSix-DOF Trim Residuals:")
        print("U Acceleration Residual:", sol.fun[0], "[ft/s^2]")
        print("V Acceleration Residual:", sol.fun[1], "[ft/s^2]")
        print("W Acceleration Residual:", sol.fun[2], "[ft/s^2]")
        print("Roll Acceleration Residual:", sol.fun[3], "[rad/s^2]")
        print("Pitch Acceleration Residual:", sol.fun[4], "[rad/s^2]")
        print("Yaw Acceleration Residual:", sol.fun[5], "[rad/s^2]")
        print("Altitude Rate Residual:", sol.fun[6], "[ft/s]")
        print("Cross-Track Velocity Residual:", sol.fun[7], "[ft/s]")

    return x, u, sol


#%% 
# Example: Level Flight
# import conversions
# from c172_params import params

# # Trim Conditions
# V_trim = conversions.kts2fps(90)
# gamma_trim = np.deg2rad(0.0)
# alt_trim = 4000
# trim_target = np.array([V_trim, gamma_trim, alt_trim])

# # Guessed unknown states
# throttle_guess = 0.45
# delta_e_guess = np.deg2rad(-2.0)
# theta_guess = 0.0
# x0 = np.array([throttle_guess, delta_e_guess, theta_guess])

# sol = longitudinal_trim(x0, trim_target)
