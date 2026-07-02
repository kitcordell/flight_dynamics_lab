#%% Imports
from pathlib import Path
import sys

import numpy as np
import matplotlib.pyplot as plt
import pandas as pd

ROOT_DIR = Path(__file__).resolve().parents[1]
if str(ROOT_DIR) not in sys.path:
    sys.path.insert(0, str(ROOT_DIR))

from flight_dynamics.c172_params import params, tf, dt, alt_0
from flight_dynamics.control_inputs import elevator_deflection
from flight_dynamics.longitudinal_dynamics import aircraft_longitudinal_dynamics
from flight_dynamics.drag_polar import drag_polar, power_curves, velocity_max
from flight_dynamics.trim_solver import longitudinal_trim, max_ROC
from flight_dynamics import conversions as conv
from flight_dynamics.integrators import RK4
from flight_dynamics.plot_theme import AERO_COLORS, set_aerospace_theme, style_axes

DATA_DIR = ROOT_DIR / "data"

set_aerospace_theme()

#%% Max Airspeed
max_speed_throttle = 0.75
max_speed_min_alt = 0
max_speed_max_alt = 12000
max_speed_alt_array = np.arange(
    max_speed_min_alt,
    max_speed_max_alt + 1000,
    1000,
)
max_speed_array = np.zeros_like(max_speed_alt_array, dtype=float)

V_max_guess = 300.0
for i, altitude in enumerate(max_speed_alt_array):
    V_max = velocity_max(altitude, max_speed_throttle, params, V_max_guess)
    max_speed_array[i] = V_max
    V_max_guess = V_max

cruise_poh = pd.read_csv(DATA_DIR / "c172_cruise_performance_visible_rows.csv")
target_mcp_percent = max_speed_throttle * 100.0

poh_cruise_rows = []
for _, group in cruise_poh.groupby("pressure_altitude_ft"):
    closest_index = (group["mcp_percent"] - target_mcp_percent).abs().idxmin()
    poh_cruise_rows.append(cruise_poh.loc[closest_index])

poh_cruise = pd.DataFrame(poh_cruise_rows).sort_values("pressure_altitude_ft")
poh_altitudes = poh_cruise["pressure_altitude_ft"].to_numpy(dtype=float)
poh_tas = poh_cruise["ktas"].to_numpy(dtype=float)
poh_mcp = poh_cruise["mcp_percent"].to_numpy(dtype=float)
estimated_tas_at_poh_altitudes = np.interp(
    poh_altitudes,
    max_speed_alt_array,
    conv.fps2kts(max_speed_array),
)

print(
    "Max Velocity @ 75% throttle, sea level: "
    f"{max_speed_array[0]:.2f} ft/s | {conv.fps2kts(max_speed_array[0]):.2f} kts"
)
print("\nCruise TAS comparison using nearest POH MCP to 75%:")
for altitude, mcp, poh_ktas, estimated_ktas in zip(
    poh_altitudes,
    poh_mcp,
    poh_tas,
    estimated_tas_at_poh_altitudes,
):
    print(
        f"{altitude:5.0f} ft | POH {mcp:4.0f}% MCP: {poh_ktas:6.1f} kt TAS | "
        f"Estimated 75%: {estimated_ktas:6.1f} kt TAS"
    )

fig_speed, ax_speed_alt = plt.subplots(1, 1, figsize=(7, 5))
fig_speed.suptitle("C172 Maximum Airspeed", fontsize=13, fontweight="bold")
ax_speed_alt.plot(
    max_speed_alt_array,
    conv.fps2kts(max_speed_array),
    color=AERO_COLORS["cyan"],
    marker="o",
    markeredgecolor=AERO_COLORS["text"],
    label="Estimated 75% Throttle",
)
ax_speed_alt.plot(poh_altitudes,poh_tas,color=AERO_COLORS["amber"],markeredgecolor=AERO_COLORS["text"],linestyle="--",label="POH Closest to 75% MCP",)
ax_speed_alt.set_title("TAS vs Altitude")
ax_speed_alt.set_xlabel("Pressure Altitude (ft)")
ax_speed_alt.set_ylabel("Maximum Airspeed (kt TAS)")
ax_speed_alt.legend()
style_axes(ax_speed_alt)
fig_speed.tight_layout(rect=[0, 0, 1, 0.92])

#%% Rate of Climb Calculations
# initial guess = [theta, delta_e, gamma]
theta_guess = np.deg2rad(5.0)  # 5 degrees
delta_e_guess = np.deg2rad(-2.0)  # -2 degrees
gamma_guess = np.deg2rad(3.0)  # 3 degrees climb

x0 = np.array([theta_guess, delta_e_guess, gamma_guess])


throttle = 1.0
V = 90.0
alt_0 = 0
trim_target = [throttle, V, alt_0]

min_alt = 0
max_alt = 12000

alt_array = np.arange(min_alt, max_alt + 1000, 1000)  # Altitude array from 0 to 12,000 ft in 1,000 ft increments
ROC_alt_array = np.zeros_like(alt_array, dtype=float)  # To store max ROC at each altitude

#%% Compute max ROC across altitudes
for i, altitude in enumerate(alt_array):
    trim_target[2] = altitude
    V_array, ROC_array, V_max_ROC, ROC_max = max_ROC(x0, trim_target, params, num_points=500, verbose=False)
    ROC_alt_array[i] = ROC_max


#%% POH Data
roc_poh = pd.read_csv(DATA_DIR / "c172_roc.csv",sep=",",engine="python",skipinitialspace=True) # read POH data

print("\n", roc_poh)

poh_roc_alt = roc_poh["press_alt_ft"]
poh_roc_0C = roc_poh["fpm_0C"]
poh_roc_M20C = roc_poh["fpm_M20C"]
poh_roc_20C = roc_poh["fpm_20C"]
poh_roc_40C = roc_poh["fpm_40C"]

# Plot POH ROC vs simulated ROC and speed sweep together
fig_roc, (ax_roc_alt, ax_roc_speed) = plt.subplots(1, 2, figsize=(12, 5))
fig_roc.suptitle("C172 Climb Performance", fontsize=13, fontweight="bold")

ax_roc_alt.plot(poh_roc_alt, poh_roc_20C, label="POH ROC", color=AERO_COLORS["amber"])
ax_roc_alt.plot(alt_array, ROC_alt_array * 60, label="Sim ROC", color=AERO_COLORS["cyan"])
ax_roc_alt.set_title("Rate of Climb vs Altitude @ 20C")
ax_roc_alt.set_xlabel("Altitude (ft)")
ax_roc_alt.set_ylabel("Rate of Climb (ft/min)")
ax_roc_alt.legend()
style_axes(ax_roc_alt)

ax_roc_speed.plot(V_array, ROC_array * 60, label="Sim Rate of Climb", color=AERO_COLORS["green"])
ax_roc_speed.plot(
    V_max_ROC,
    ROC_max * 60,
    "o",
    color=AERO_COLORS["red"],
    markeredgecolor=AERO_COLORS["text"],
    label="Max ROC",
)
ax_roc_speed.set_title("Rate of Climb vs Airspeed")
ax_roc_speed.set_xlabel("Airspeed (ft/s)")
ax_roc_speed.set_ylabel("Rate of Climb (ft/min)")
ax_roc_speed.legend()
style_axes(ax_roc_speed)

fig_roc.tight_layout(rect=[0, 0, 1, 0.94])


#%% Drag Polar Plots
fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 5))
fig.suptitle("C172 Aerodynamic Performance", fontsize=13, fontweight="bold")

# Drag Polar
V, D, D_i, D_p = drag_polar(alt_0, params)
ax1.plot(V, D, label="Total Drag", color=AERO_COLORS["cyan"])
ax1.plot(V, D_i, label="Induced Drag", color=AERO_COLORS["amber"])
ax1.plot(V, D_p, label="Parasite Drag", color=AERO_COLORS["magenta"])
ax1.set_xlabel("Velocity (ft/s)")
ax1.set_ylabel("Drag (lbf)")
ax1.set_title("Drag Polar - C172")
ax1.legend()
style_axes(ax1)

# Power Curves
V, P_req, P_i, P_p, P_A = power_curves(4000, 1.0, params)
ax2.plot(V, P_req, label="Power Required", color=AERO_COLORS["cyan"])
ax2.plot(V, P_A, label="Power Available", color=AERO_COLORS["green"])
ax2.set_xlabel("Velocity (ft/s)")
ax2.set_ylabel("Power (lbÂ·ft/s)")
ax2.set_title("Power Curves - C172")
ax2.legend()
style_axes(ax2)

plt.tight_layout(rect=[0, 0, 1, 0.94])





#%% Solve for trim conditions
# xdot_trim, theta_trim, U_0, W_0, Q_0 = trim_solver(0.45, -2.0, 3.0)  #  return x_trim, alpha_trim, U_0, W_0, Q_0

# Trim Conditions
V_trim = conv.kts2fps(90)
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
t_rk4, x_rk4 = RK4(aircraft_longitudinal_dynamics, (0.0, tf), x_trim, dt, args=(u_trim,params,))   # integrate equations of motion
alpha = np.arctan2(x_rk4[:,1], x_rk4[:,0]) # angle of attack calculated from forward and vertical velocity, [rad]


## Log Elevator Deflection
_, delta_e = u_trim
elevator_deflection_history = np.zeros_like(t_rk4) # initialize elevator deflection array with same dimensions as time vector
for i in range(len(t_rk4)):
    elevator_deflection_history[i] = elevator_deflection(t_rk4[i], delta_e) # plugs in all values of "t" into the elevator function and stores then in an array

#%% Rate of Climb Plot




#%% X-Plane Data

data_xplane = pd.read_csv(         # read data
    DATA_DIR / "Data.txt",
    sep="|",
    engine="python",
    skipinitialspace=True
)

data_xplane = data_xplane.iloc[6000:9000]     # select data set
time_xplane = data_xplane["_totl,_time "]
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
fig.suptitle("Nonlinear Longitudinal States - RK4 vs X-Plane", fontsize=13, fontweight="bold")

axs[0].plot(t_rk4, x_rk4[:,0], color=AERO_COLORS["cyan"])
axs[0].plot(time_xplane, xplane_U, color=AERO_COLORS["amber"], linestyle='dashed' )
axs[0].set_ylabel("u (ft/s)")
axs[0].legend(["Sim U", "X-Plane U"], loc="center right")
style_axes(axs[0])

axs[1].plot(t_rk4, x_rk4[:,1], color=AERO_COLORS["cyan"])
axs[1].plot(time_xplane, xplane_W, color=AERO_COLORS["amber"], linestyle='dashed' )
axs[1].set_ylabel("w (ft/s)")
axs[1].legend(["Sim W", "X-Plane W"], loc="center right")
style_axes(axs[1])

axs[2].plot(t_rk4, np.rad2deg(x_rk4[:,2]), color=AERO_COLORS["cyan"]) 
axs[2].plot(time_xplane, xplane_Q, color=AERO_COLORS["amber"], linestyle='dashed' )       # Pitch Rate
axs[2].set_ylabel("Q (deg/s)")
axs[2].legend(["Sim Q", "X-Plane Q"], loc="center right")
style_axes(axs[2])

axs[3].plot(t_rk4, np.rad2deg(x_rk4[:,3]), color=AERO_COLORS["cyan"])                           # Pitch
axs[3].plot(time_xplane, xplane_pitch, color=AERO_COLORS["amber"], linestyle='dashed' )
axs[3].set_ylabel("Î¸ (deg)")
axs[3].set_xlabel("Time (s)")
axs[3].legend(["Sim Î¸", "X-Plane Î¸"], loc="center right")
style_axes(axs[3])

axs[4].plot(t_rk4, np.rad2deg(alpha), color=AERO_COLORS["cyan"])                             # Alpha
axs[4].plot(time_xplane, xplane_alpha, color=AERO_COLORS["amber"], linestyle='dashed')
axs[4].set_ylabel("Î± (deg)")
axs[4].set_xlabel("Time (s)")
axs[4].legend(["Sim Î±", "X-Plane Î±"], loc="center right")
style_axes(axs[4])

axs[5].plot(t_rk4,x_rk4[:,4], color=AERO_COLORS["cyan"])                                 # Altitude
axs[5].plot(time_xplane, xplane_alt, color=AERO_COLORS["amber"], linestyle='dashed')
axs[5].set_ylabel("Altitude (ft)")
axs[5].set_xlabel("Time (s)")
axs[5].legend(["Sim alt", "X-Plane alt"], loc="center right")
style_axes(axs[5])

axs[6].plot(t_rk4,np.rad2deg(elevator_deflection_history), color=AERO_COLORS["cyan"])
axs[6].plot(time_xplane,xplane_elevator_deflection, color=AERO_COLORS["amber"], linestyle ='dashed')
axs[6].set_ylabel("Elevator Deflection (deg)")
axs[6].set_xlabel("Time (s)")
axs[6].legend(["SimElevator Deflection", "X-Plane Elevator Deflection"], loc="center right")
style_axes(axs[6])


plt.tight_layout(rect=[0, 0, 1, 0.97])
plt.show()
