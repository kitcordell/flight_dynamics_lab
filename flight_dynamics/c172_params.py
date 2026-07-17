import numpy as np
from flight_dynamics import conversions
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

    # Aircraft moments of inertia
    "I_xx": 948.0,                # roll moment of inertia, [slug*ft^2]
    "I_yy": 1346.0,               # pitch moment of inertia, [slug*ft^2]
    "I_zz": 1967.0,               # yaw moment of inertia, [slug*ft^2]
    "I_xz": 0.0,                  # roll-yaw product of inertia, [slug*ft^2]
    "W": 2300.0,                  # lbf

    "V_S": 45.0,          # stall speed in indicated knots
    "V_ne": 170.0,        # never exceed speed in indicated knots
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
    "C_m_0": 0.035,
    "C_m_alpha": -0.89,
    "C_m_delta_e": -1.28,
    "C_mq": -12.4,

    # Side-force coefficients
    "C_Y_0": 0.0,                 # side force at zero sideslip
    "C_Y_beta": -0.31,            # side-force change with sideslip angle
    "C_Y_p": -0.037,              # side-force change with roll rate
    "C_Y_r": 0.21,                # side-force change with yaw rate
    "C_Y_delta_a": 0.0,           # side-force change with aileron deflection
    "C_Y_delta_r": 0.187,         # side-force change with rudder deflection

    # Rolling-moment coefficients
    "C_l_0": 0.0,                 # rolling moment at zero sideslip
    "C_l_beta": -0.089,           # rolling-moment change with sideslip angle
    "C_l_p": -0.47,               # roll damping derivative
    "C_l_r": 0.096,               # rolling-moment change with yaw rate
    "C_l_delta_a": -0.178,        # aileron control derivative
    "C_l_delta_r": 0.0147,        # rudder contribution to rolling moment

    # Yawing-moment coefficients
    "C_n_0": 0.0,                 # yawing moment at zero sideslip
    "C_n_beta": 0.065,            # directional stability derivative
    "C_n_p": -0.030,              # yawing-moment change with roll rate
    "C_n_r": -0.099,              # yaw damping derivative
    "C_n_delta_a": 0.053,         # aileron contribution to yawing moment
    "C_n_delta_r": -0.0657,       # rudder control derivative
}

params["AR"] = params["bw"]**2 / params["S"]
params["P_max_SL"] = conversions.hp2ftlbfps(params["P_max_SL"])
