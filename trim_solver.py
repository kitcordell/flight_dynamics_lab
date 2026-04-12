
#%%
import numpy as np
from aircraft_longitudinal_dynamics import aircraft_longitudinal_dynamics
from c172_params import params, alt_0
from scipy.optimize import least_squares
from axis_transformations import body_to_velocity, velocity_to_body


#%%
## Build Trim States
    # Solves for trim conditions given a fixed velocity, flight path angle, and altitude.
    # Builds the desired state vector for trim conditions from flight path axis to body axis
def build_trim_states( unknown, trim_target):
    V, gamma, alt = trim_target
    throttle, delta_e, theta  = unknown

    U, W, alpha = velocity_to_body(V,gamma,theta)
    Q = 0.0  # pitch rate, [rad/s]

    x = np.array([U, W, Q, theta, alt])
    return x

## Trim Residuals
    # Separates what values are inputted and what is solved for into target and unknown values to build the trim states
    # Also separates them into inputs and outputs for the dynamics function
def trim_residuals(unknown, trim_target):
    
    # Build States
    V, gamma, alt = trim_target                 # desired states to be trimmed for (inputs)
    throttle, delta_e, theta = unknown    # unknown states to solve for (guesses)
    
    # Calculate Dynamics
    x = build_trim_states(unknown, trim_target) # state vector
    u = np.array([throttle, delta_e])   # control vector

    xdot_desired = np.zeros_like(x)     # set desired derivatives to zero
    xdot_desired[4] = V * np.sin(gamma) # set desired hdot as a function of gamma

    xdot_actual = aircraft_longitudinal_dynamics(0.0, x, u, params)
    
    residual = xdot_actual - xdot_desired
   
    return residual

# x0: initial guesses for [throttle [%], elevator deflection [rad], theta [rad]]
# trim_target: target values for [velocity [ft/s], flight path angle [rad], altitude [ft]]
def longitudinal_trim(x0, trim_target):
    
    sol = least_squares(trim_residuals, x0, bounds=([0.0, -np.inf, -np.inf], [1.0, np.inf, np.inf]), args=(trim_target,)) # Solve Non-linear equations
    
   
    throttle_trim, delta_e_trim, theta_trim = sol.x  # unpack solution
    
    x = build_trim_states(sol.x, trim_target)  # convert back to body axis 
    u = np.array([throttle_trim, delta_e_trim])
    

    # unpacking for printing
    V_trim, gamma_trim, alt_trim = x0 # unpack trim conditions
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

  
    print(x)
    print(u)
    return x, u

#%% 
# # Example
# import conversions

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



