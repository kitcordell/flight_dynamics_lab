import numpy as np


t0 = 0
tf = 115
dt = 0.01
alt_0 = 4000



params = {
    "g": 32,                                  # acceleration of gravity [ft/s^2]
    "rho": 0.002378,                          # air density, [slug/ft^3]
    "bw": 35.8,                               # wing span, [ft]
    "cbar": 4.9,                              # average chord, [ft]
    "K": 0.054,
    "I_yy": 1346,                             # moment of inertia about pitch axis, [slug*ft^2]
    "e" : 0.7,                                # oswald efficiency factor
    "S" : 175,                                # wing area, [ft^2]

    "V_S" : 45,
    "V_ne" : 170,
    
    "V_trim": 150.0,   # trimmed free stream velocity, [ft/s]  
    "gamma_trim": np.deg2rad(0.0), # flight path angle, [deg]
    
    "W": 2300,                                # aircraft weight, [lb]
    # "delta_e_trim": np.deg2rad(-2.5268517650491225),                 # elevator deflection, [rad]
    # "thrust": 217.05256710180734,                              # total thrust in, [lb]

    # Lift coefficients
    "C_L_alpha": 5.143,                                         # lift curve slope
    "C_L_0": 0.31,                                              # lift at zero aoa
    "C_L_delta_e": 0.43,                                        # lift per rad of elevator deflection

    # Drag
    "C_D_0": 0.031,                                            # zero lift drag coefficient
    "C_D_u": 0,

    # Pitching moment
    "C_m_delta_e": -1.28,                                      # pitching moment per rad of elevator deflection
    "C_m_alpha": -0.89,                                        #
    "C_mq": -12.4,                                             #
    "C_m_0": -0.015                                            # 
}
