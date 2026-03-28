import numpy as np
from standard_atmosphere import standard_atmosphere
import aero_model


def aircraft_longitudinal_dynamics(t,x, params):
    U, W, Q, theta, alt = x
    


    g = params["g"]
    bw = params["bw"]                         # wing span, [ft]
    cbar = params["cbar"]                     # average chord, [ft]
    S = bw * cbar                             # wing surface area [ft^2]
    AR = bw**2 / S
    delta_e = params["delta_e"]               # elevator deflection, [rad]
    thrust = params["thrust"]                 # total thrust in, [lb]

    I_yy = params["I_yy"]                     # moment of inertia about pitch axis, [slug*ft^2]
    m = params["W"] / g                       # aircraft mass, [slugs]

    rho, _, _ = standard_atmosphere(alt)   # air density calculatino
    
    V = np.sqrt(U**2+W**2)                  #  resolved aircraft velocity
    qbar = 0.5 * rho * V**2 # dynamic pressure, [lb/ft^2]
    alpha = np.arctan2(W,U)
    
    # Lift Calculations
    C_L_alpha   = params["C_L_alpha"]
    C_L_0       = params["C_L_0"]
    C_L_delta_e = params["C_L_delta_e"]

    C_D_0       = params["C_D_0"]

    C_m_delta_e = params["C_m_delta_e"]
    C_m_alpha   = params["C_m_alpha"]
    C_mq        = params["C_mq"]
    C_m_0       = params["C_m_0"]
  
   ## Elevator Input Function
    delta_e = elevator_deflection(t, delta_e)

#%% Calculate forces and moments
    C_L, C_D,_ , C_m = aero_model.aero_coefficients(alpha, delta_e, Q, V, params)
    
    L = qbar * S * C_L # total lift
    D = qbar * S * C_D  # total drag
    M = qbar * S * cbar * C_m   # total pitching moment

    
    X = -D * np.cos(alpha) + L * np.sin(alpha) + thrust # X forces
    Z = -D * np.sin(alpha) - L * np.cos(alpha)  # Z forces
    

    xdot = np.zeros_like(x)     # initialize derivative array


    xdot[0] = X/m - g*np.sin(theta) - Q*W   # # U_dot
    xdot[1] = Z/m + g*np.cos(theta) + Q*U   # W_dot
    xdot[2] = M/I_yy    # Q_dot
    xdot[3] = Q # theta_dot
    xdot[4] = U * np.sin(theta) - W * np.cos(theta) # h_dot

    

    return xdot



def elevator_deflection(t, delta_e):
    if t > 5.6 and t < 6.6:
        delta_e = delta_e + np.deg2rad(-2.164)

    else:

        delta_e = delta_e

    return delta_e