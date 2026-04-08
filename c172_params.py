import numpy as np
import conversions
from standard_atmosphere import standard_atmosphere
t0 = 0
tf = 115
dt = 0.01
alt_0 = 4000  # ft

params = {
    "g": 32.174,                  # ft/s^2
    "bw": 35.8,                   # ft
    "cbar": 4.9,                  # ft
    "S": 175.0,                   # ft^2
    "e": 0.7,

    "I_yy": 1346.0,               # slug*ft^2
    "W": 2300.0,                  # lbf

    "V_S": 45.0,                  # decide units and stay consistent
    "V_ne": 170.0,                # decide units and stay consistent
    "V_trim": 150,              # ft/s
    "gamma_trim": np.deg2rad(0.0),# rad

    # Propulsion
    "P_max_SL": 180.0,           # hp at sea level, convert to ft*lbf/s in thrust_model.py
    "eta_p": 0.8,                # propeller efficiency

    # Lift coefficients
    "C_L_alpha": 5.143,           
    "C_L_0": 0.31,
    "C_L_delta_e": 0.43,

    # Drag coefficients
    "C_D_0": 0.031,
    "C_D_u": 0.0,

    # Pitching moment coefficients
    "C_m_0": -0.015,
    "C_m_alpha": -0.89,
    "C_m_delta_e": -1.28,
    "C_mq": -12.4,
}

_, T, _ = standard_atmosphere(alt_0)
params["AR"] = params["bw"]**2 / params["S"]
params["V_ne"] = conversions.ias2tas(params["V_ne"], alt_0, T)
params["V_S"] = conversions.ias2tas(params["V_S"], alt_0, T)
params["P_max_SL"] = conversions.hp2ftlbfps(params["P_max_SL"])