import numpy as np
from flight_dynamics import aero_model, control_inputs, thrust_model
from flight_dynamics.axis_transformations import aero_to_body
from flight_dynamics.c172_params import params



def central_difference(f,x0):
    
    dx = 1e-5 * max(abs(x0), 1.0)                   # step size is determined how large the function values are
    slope = (f(x + dx) - f(x - dx)) / (2 * dx)      # numerically differentiate
    
    
    return slope

def longitudinal_EOM(t, x, u, control_input, params, aero_model, thrust_model):
# States
    # Angles & Velocities
    U = x[0]
    W = x[1]
    Q = x[2]
    theta_0 = x[3]

    # Inputs
    Throttle = u[0]
    delta_e = u[1]


    # Calculate aerodynamic forces based on selected model and control inputs
    air_data = aero_model.airdata(x,params) # load relevant air data (U, W, alpha, etc.) for aerodynamics
    Thrust, delta_e = control_inputs.control_input() # load selected control input (neutral elevator, elevator up at specific time, etc.)
    L, D, M = aero_model.aero_model((air_data, control_input, params)) # calculate lift, drag and moment using the selected aerodynamics and thrust model

    T = thrust_model.thrust_model(throttle, V, alt, params)



    X = aero_to_body(L,D,T,alpha)         # calculate forces in the X direction
    X_u = central_difference()

    # Initialize A and B arrays
    A = np.arange(16)
    A = np.reshape((4,4))
    
    B = np.arange(4)
    B = np.reshape((1,4))
    

    A = np.array([
            [X_u,  X_w,       X_q,       -g * np.cos(theta_0)],
            [Z_u,  Z_w,       Z_q + U0,  -g * np.sin(theta_0)],
            [M_u,  M_w,       M_q,        0.0],
            [0.0, 0.0,      1.0,       0.0],
        ])

    B = np.array([
            [X_de],
            [Z_de],
            [M_de],
            [0.0],
        ])

# Initial Conditions
U_0 = 151.75    # forward velocity [ft/s]
W_0 = 6.76      # vertical velocity [ft/s]
Q_0 = 0         # pitch rate [rad/s]
theta_0 = 0.0445 # body angle [rad]

x0 = [U_0, W_0, Q_0, theta_0] # U, W, Q, theta

t = np.linspace(0,20, 1000)

xdot = longitudinal_EOM(t, x, u, params, aero_loads, thrust_piston_na)
