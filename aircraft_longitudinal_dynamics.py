import numpy as np



def aircraft_longitudinal_dynamics(t,x, params):
    u, w, q, theta = x
    


    g = params["g"]
    rho = params["rho"]                       # air density, [slug/ft^3]
    bw = params["bw"]                         # wing span, [ft]
    cbar = params["cbar"]                     # average chord, [ft]
    S = bw * cbar                             # wing surface area [ft^2]
    AR = bw / cbar
    delta_e = params["delta_e"]               # elevator deflection, [rad]
    thrust = params["thrust"]                 # total thrust in, [lb]
    K = params["K"]

    I_yy = params["I_yy"]                     # moment of inertia about pitch axis, [slug*ft^2]
    m = params["W"] / g                       # aircraft mass, [slugs]
    
    V = np.sqrt(u**2+w**2)
    qbar = 0.5 * rho * V**2 # dynamic pressure, [lb/ft^2]
    alpha = np.arctan2(w,u)
    
    # Lift Calculations
    C_L_alpha   = params["C_L_alpha"]
    C_L_0       = params["C_L_0"]
    C_L_delta_e = params["C_L_delta_e"]

    C_D_0       = params["C_D_0"]

    C_m_delta_e = params["C_m_delta_e"]
    C_m_alpha   = params["C_m_alpha"]
    C_mq        = params["C_mq"]
    C_m_0       = params["C_m_0"]
  
    
    C_L = C_L_0 + C_L_alpha * alpha +C_L_delta_e * delta_e  # coefficient of lift
    L = qbar * S * C_L # total lift

    # Drag Calculations
    C_D = C_D_0 + C_L**2 * K
    D = qbar * S * C_D
    
    # Pitching Moment Calculation


    C_m = C_m_0 + C_m_alpha * alpha + C_m_delta_e * delta_e + C_mq * ((q*cbar)/(2*V))
    M = qbar * S * cbar * C_m

    
    X = -D * np.cos(alpha) + L * np.sin(alpha) + thrust
    Z = -D * np.sin(alpha) - L * np.cos(alpha)
    

    xdot = np.zeros_like(x)


    xdot[0] = X/m - g*np.sin(theta) - q*w
    xdot[1] = Z/m + g*np.cos(theta) + q*u
    xdot[2] = M/I_yy
    xdot[3] = q

    

    return xdot



