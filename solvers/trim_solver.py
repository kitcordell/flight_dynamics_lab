
#%%
import numpy as np
from scipy.optimize import least_squares
from aircraft.c172_params import params, alt_0
from models.aircraft_longitudinal_dynamics import aircraft_longitudinal_dynamics
from utils.axis_transformations import body_to_velocity, velocity_to_body


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

  
    return x, u

import numpy as np
from scipy.optimize import least_squares


def climb_trim_residuals(unknown, trim_target):
    # unknown = [theta, delta_e, gamma]
    theta, delta_e, gamma = unknown

    # trim_target = [throttle, V, alt]
    throttle, V, alt = trim_target

    x = build_trim_states(V, gamma, alt, theta)
    u = np.array([throttle, delta_e])

    xdot_actual = aircraft_longitudinal_dynamics(0.0, x, u, params)

    # Steady climb trim conditions
    residual = np.array([
        xdot_actual[0],   # U_dot
        xdot_actual[1],   # W_dot
        xdot_actual[2]    # Q_dot
    ])

    return residual


# x0 = [theta_guess, delta_e_guess, gamma_guess]
# trim_target = [throttle, V, alt]
def climb_trim_at_speed(x0, trim_target):
    throttle_trim, V_trim, alt_trim = trim_target

    sol = least_squares(
        climb_trim_residuals,
        x0,
        args=(trim_target,)
    )

    theta_trim, delta_e_trim, gamma_trim = sol.x

    x = build_trim_states(V_trim, gamma_trim, alt_trim, theta_trim)
    u = np.array([throttle_trim, delta_e_trim])

    U_trim, W_trim, Q_trim, theta_state, alt_state = x

    ROC_fps = V_trim * np.sin(gamma_trim)
    ROC_fpm = ROC_fps * 60.0

    print("\nTrim Target:")
    print("Throttle:", throttle_trim * 100, "[%]")
    print("Velocity:", V_trim, "[ft/s]")
    print("Altitude:", alt_trim, "[ft]")

    print("\nTrim Solution:")
    print("Theta:", theta_trim, "[rad]")
    print("Elevator Deflection:", delta_e_trim, "[rad]")
    print("Flight Path Angle:", gamma_trim, "[rad]")

    print("\nTrimmed State:")
    print("Forward Body Velocity (U):", U_trim, "[ft/s]")
    print("Vertical Body Velocity (W):", W_trim, "[ft/s]")
    print("Pitch Rate (Q):", Q_trim, "[rad/s]")
    print("Body Angle (theta):", theta_state, "[rad]")
    print("Altitude:", alt_state, "[ft]")

    print("\nClimb Performance:")
    print("ROC:", ROC_fps, "[ft/s]")
    print("ROC:", ROC_fpm, "[ft/min]")

    return x, u, gamma_trim, ROC_fps, ROC_fpm, sol

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


#%% 
# # Example: ROC max

# V_array = np.linspace(80, 180, 30)   # ft/s, example only
# throttle = 1.0
# alt = 4000.0

# # initial guess = [theta, delta_e, gamma]
# x0 = np.array([
#     np.deg2rad(5.0),
#     np.deg2rad(-2.0),
#     np.deg2rad(3.0)
# ])

# trim_target = [1, 90, 4000]
# climb_trim_at_speed(x0, trim_target)
