import numpy as np

t0 = 0
tf = 200
dt = 0.01
U_0 = 110 * 1.6 # initial velocity, [ft/s]
alt_0 = 5000


params = {
    "g": 32,                                  # Acceleration of gravity
    "rho": 0.002378,                          # air density, [slug/ft^3]
    "bw": 35.8,                               # wing span, [ft]
    "cbar": 4.9,                              # average chord, [ft]
    "K": 0.054,
    "I_yy": 1346,                             # moment of inertia about pitch axis, [slug*ft^2]
    "e" : 0.7,                                # oswald efficiency factor
    "S" : 175,

    "V_S" : 40,
    "V_ne" : 400,
    
    "W": 2500,                                # aircraft weight, [lb]
    "delta_e": np.deg2rad(-5),                 # elevator deflection, [rad]
    "thrust": 0,                              # total thrust in, [lb]

    # Lift coefficients
    "C_L_alpha": 5.143,
    "C_L_0": 0.31,
    "C_L_delta_e": 0.43,

    # Drag
    "C_D_0": 0.031,

    # Pitching moment
    "C_m_delta_e": -1.28,
    "C_m_alpha": -0.89,
    "C_mq": -12.4,
    "C_m_0": -0.015
}