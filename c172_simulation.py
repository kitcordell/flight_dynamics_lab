#%% Imports
import numpy as np
import matplotlib.pyplot as plt
from integrators import RK4
from scipy.integrate import solve_ivp
from scipy.optimize import least_squares
import pandas as pd
import conversions

from aircraft_longitudinal_dynamics import aircraft_longitudinal_dynamics, elevator_deflection
from drag_polar import drag_polar
from trim_solver import longitudinal_trim
from c172_params import params, t0, tf, dt, alt_0
from thrust_model import thrust_piston_na


#%% Drag Polar Plots
V, D, D_i, D_p = drag_polar(alt_0, params,)
plt.plot(V[:], D[:], label = "Total Drag", linewidth=1)
plt.plot(V, D_i[:], label = "Induced Drag", linewidth=1)
plt.plot(V,D_p[:], label = "Parasite Drag", linewidth=1)
plt.legend()
plt.title("Drag Polar Curve, C172")
plt.ylabel("Drag (lbf)")
plt.xlabel("Velocity (ft/s)")


#%% Solve for trim conditions
# xdot_trim, theta_trim, U_0, W_0, Q_0 = trim_solver(0.45, -2.0, 3.0)  #  return x_trim, alpha_trim, U_0, W_0, Q_0

# Trim Conditions
V_trim = conversions.kts2fps(90)
gamma_trim = np.deg2rad(0.0)
alt_trim = 4000
trim_target = np.array([V_trim, gamma_trim, alt_trim])

# Guessed unknown states
throttle_guess = 0.45
delta_e_guess = np.deg2rad(-2.0)
theta_guess = 0.0
x0 = np.array([throttle_guess, delta_e_guess, theta_guess])

x_trim, u_trim = longitudinal_trim(x0, trim_target)



#%% Dynamics Calculations
    # Uses RK4 script for numerical integration and aircraft_longitudinal_dynamics EOM script
t_rk4, x_rk4 = RK4(aircraft_longitudinal_dynamics, (0.0, tf), [x_trim], dt, args=(u_trim,params,))   # integrate equations of motion
alpha = np.arctan2(x_rk4[:,1], x_rk4[:,0]) # angle of attack calculated from forward and vertical velocity, [rad]


## Log Elevator Deflection
_, delta_e = u_trim
elevator_deflection_history = np.zeros_like(t_rk4) # initialize elevator deflection array with same dimensions as time vector
for i in range(len(t_rk4)):
    elevator_deflection_history[i] = elevator_deflection(t_rk4[i], delta_e) # plugs in all values of "t" into the elevator function and stores then in an array

#%% Rate of Climb Plot



#%% POH Data
roc_poh = pd.read_csv(         # read data
    "c172_roc.csv",
    sep=",",
    engine="python",
    skipinitialspace=True
)

print(roc_poh)

poh_roc_alt = roc_poh["press_alt_ft"]
poh_roc_0C = roc_poh["fpm_0C"]
poh_roc_M20C = roc_poh["fpm_M20C"]
poh_roc_20C = roc_poh["fpm_20C"]
poh_roc_40C = roc_poh["fpm_40C"]

plt.figure(2)
plt.plot(poh_roc_alt, poh_roc_M20C, label="-20C", linewidth=1)
plt.plot(poh_roc_alt, poh_roc_0C, label="0C", linewidth=1)
plt.plot(poh_roc_alt, poh_roc_20C, label="20C", linewidth=1)
plt.plot(poh_roc_alt, poh_roc_40C, label="40C", linewidth=1)

plt.xlabel("Pressure Altitude (ft)")
plt.ylabel("Rate of Climb (fpm)")
plt.title("C172 Rate of Climb Performance (POH)")
plt.legend()
plt.grid(True)

#%% X-Plane Data

data_xplane = pd.read_csv(         # read data
    "Data.txt",
    sep="|",
    engine="python",
    skipinitialspace=True
)

print(data_xplane.columns.tolist()) # print column names
data_xplane = data_xplane.iloc[6000:9000]     # select data set
print(data_xplane.head())              # preview data
time_xplane = data_xplane["_totl,_time "]
print(time_xplane.iloc[1])
time_xplane = time_xplane - time_xplane.iloc[1]     # Subtract total sim time to start the time span at zero

xplane_pitch = data_xplane["pitch,__deg "]  
xplane_elevator_deflection = data_xplane["elev1,__deg .1"]
xplane_alpha = data_xplane["alpha,__deg "]
xplane_alt = data_xplane["p-alt,ftMSL "]
xplane_U = data_xplane["Vtrue,_ktas "] * np.cos(np.deg2rad(xplane_alpha)) * 1.68781
xplane_W = data_xplane["Vtrue,_ktas "] * np.sin(np.deg2rad(xplane_alpha)) * 1.68781
xplane_Q = data_xplane["____Q,deg/s "] 




#%% Comparison Plots
fig, axs = plt.subplots(7, 1, figsize=(9,10), sharex=True)

axs[0].plot(t_rk4, x_rk4[:,0], color="red", linewidth=1)
axs[0].plot(time_xplane, xplane_U, color="royalblue", linewidth=1, linestyle='dashed' )
axs[0].set_ylabel("u (ft/s)")
axs[0].set_title("Nonlinear Longitudinal Aircraft States Comparison (RK4)")
axs[0].legend(["Sim U", "X-Plane U"], loc="center right")
axs[0].grid(True)

axs[1].plot(t_rk4, x_rk4[:,1], color="red", linewidth=1)
axs[1].plot(time_xplane, xplane_W, color="royalblue", linewidth=1, linestyle='dashed' )
axs[1].set_ylabel("w (ft/s)")
axs[1].legend(["Sim W", "X-Plane W"], loc="center right")
axs[1].grid(True)

axs[2].plot(t_rk4, np.rad2deg(x_rk4[:,2]), color="red", linewidth=1) 
axs[2].plot(time_xplane, xplane_Q, color="royalblue", linewidth=1, linestyle='dashed' )       # Pitch Rate
axs[2].set_ylabel("Q (deg/s)")
axs[2].legend(["Sim Q", "X-Plane Q"], loc="center right")
axs[2].grid(True)

axs[3].plot(t_rk4, np.rad2deg(x_rk4[:,3]), color="red", linewidth=1)                           # Pitch
axs[3].plot(time_xplane, xplane_pitch, color="royalblue", linewidth=1, linestyle='dashed' )
axs[3].set_ylabel("θ (deg)")
axs[3].set_xlabel("Time (s)")
axs[3].legend(["Sim θ", "X-Plane θ"], loc="center right")
axs[3].grid(True)

axs[4].plot(t_rk4, np.rad2deg(alpha), color="red", linewidth=1)                             # Alpha
axs[4].plot(time_xplane, xplane_alpha, color="royalblue", linewidth=1, linestyle='dashed')
axs[4].set_ylabel("α (deg)")
axs[4].set_xlabel("Time (s)")
axs[4].legend(["Sim α", "X-Plane α"], loc="center right")
axs[4].grid(True)

axs[5].plot(t_rk4,x_rk4[:,4], color="red", linewidth=1)                                 # Altitude
axs[5].plot(time_xplane, xplane_alt, color="royalblue", linewidth=1, linestyle='dashed')
axs[5].set_ylabel("Altitude (ft)")
axs[5].set_xlabel("Time (s)")
axs[5].legend(["Sim alt", "X-Plane alt"], loc="center right")
axs[5].grid(True)

axs[6].plot(t_rk4,np.rad2deg(elevator_deflection_history), color="red", linewidth=1)
axs[6].plot(time_xplane,xplane_elevator_deflection, color="royalblue", linewidth=1, linestyle ='dashed')
axs[6].set_ylabel("Elevator Deflection (deg)")
axs[6].set_xlabel("Time (s)")
axs[6].legend(["SimElevator Deflection", "X-Plane Elevator Deflection"], loc="center right")
axs[6].grid(True)


plt.tight_layout()
plt.show()

