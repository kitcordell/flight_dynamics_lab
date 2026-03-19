import numpy as np
import matplotlib.pyplot as plt
from integrators import RK4
from scipy.integrate import solve_ivp
from scipy.optimize import root
import pandas as pd

from aircraft_longitudinal_dynamics import aircraft_longitudinal_dynamics, elevator_deflection
from drag_polar import drag_polar
from elevator_trim_solver import aircraft_longitudinal_trim_solver
from c172_params import params, t0, tf, dt, alt_0






## Drag Polar Plots
V, D, D_i, D_p = drag_polar(alt_0, params,)
plt.plot(V[:], D[:], label = "Total Drag")
plt.plot(V, D_i[:], label = "Induced Drag")
plt.plot(V,D_p[:], label = "Parasite Drag")
plt.legend()
plt.title("Drag Polar Curve, C172")
plt.ylabel("Drag (lbf)")
plt.xlabel("Velocity (kts)")



## Trim Solver

# initial guess
x0 = np.array([
    300.0,               # thrust guess [lb]
    np.deg2rad(-2.0),    # delta_e guess [rad]
    np.deg2rad(3.0),     # theta guess [rad]
])

sol = root(aircraft_longitudinal_trim_solver, x0, args=(params,), method="hybr")
thrust_trim, delta_e_trim, theta_trim = sol.x

print("success:", sol.success)
print("message:", sol.message)
print("thrust trim [lb]:", thrust_trim)
print("delta_e trim [deg]:", np.rad2deg(delta_e_trim))
print("theta trim [deg]:", np.rad2deg(theta_trim))
print("trim residuals:", aircraft_longitudinal_trim_solver(sol.x, params))

V = params["V_trim"]


alpha_trim = theta_trim - params["gamma_trim"]

U_0 = V * np.cos(alpha_trim)
W_0 = V * np.sin(alpha_trim)
Q_0 = 0.0


x_trim = np.array([U_0, W_0, Q_0, theta_trim, alt_0])

params["thrust"] = thrust_trim
params["delta_e"] = delta_e_trim


xdot_trim = aircraft_longitudinal_dynamics(0.0, x_trim, params)

print("u_dot     =", xdot_trim[0])
print("w_dot     =", xdot_trim[1])
print("q_dot     =", xdot_trim[2])
print("theta_dot =", xdot_trim[3])
print("h_dot     =", xdot_trim[4])







## Dynamics Calculations
    # Uses RK4 script for numerical integration and aircraft_longitudinal_dynamics EOM script

t_rk4, x_rk4 = RK4(aircraft_longitudinal_dynamics, (0.0, tf), [U_0, W_0, Q_0, theta_trim, alt_0], dt, args=(params,))

alpha = np.arctan2(x_rk4[:,1], x_rk4[:,0]) # angle of attack calculated from forward and vertical velocity, [rad]


## Log Elevator Deflection
elevator_deflection_history = np.zeros_like(t_rk4) # initialize elevator deflection array with same dimensions as time vector
for i in range(len(t_rk4)):
    elevator_deflection_history[i] = elevator_deflection(t_rk4[i], params["delta_e"]) # plugs in all values of "t" into the elevator function and stores then in an array



## Dynamics Plots

# fig, axs = plt.subplots(7, 1, figsize=(9,10), sharex=True)

# axs[0].plot(t_rk4, x_rk4[:,0], color="royalblue", linewidth=2)
# axs[0].set_ylabel("u (ft/s)")
# axs[0].set_title("Nonlinear Longitudinal Aircraft States (RK4)")
# axs[0].legend(["u — forward body velocity"])
# axs[0].grid(True)

# axs[1].plot(t_rk4, x_rk4[:,1], color="darkorange", linewidth=2)
# axs[1].set_ylabel("w (ft/s)")
# axs[1].legend(["w — vertical body velocity (z-axis, positive down)"])
# axs[1].grid(True)

# axs[2].plot(t_rk4, np.rad2deg(x_rk4[:,2]), color="forestgreen", linewidth=2)
# axs[2].set_ylabel("q (deg/s)")
# axs[2].legend(["q — pitch rate"])
# axs[2].grid(True)

# axs[3].plot(t_rk4, np.rad2deg(x_rk4[:,3]), color="crimson", linewidth=2)
# axs[3].set_ylabel("θ (deg)")
# axs[3].set_xlabel("Time (s)")
# axs[3].legend(["θ — pitch angle (aircraft attitude)"])
# axs[3].grid(True)

# axs[4].plot(t_rk4, np.rad2deg(alpha), color="purple", linewidth=2)
# axs[4].set_ylabel("α (deg)")
# axs[4].set_xlabel("Time (s)")
# axs[4].legend(["α — angle of attack"])
# axs[4].grid(True)

# axs[5].plot(t_rk4,x_rk4[:,4], color="purple", linewidth=2)
# axs[5].set_ylabel("Altitude (ft)")
# axs[5].set_xlabel("Time (s)")
# axs[5].legend(["Altitude"])
# axs[5].grid(True)

# axs[6].plot(t_rk4,np.rad2deg(elevator_deflection_history), color="purple", linewidth=2)
# axs[6].set_ylabel("Elevator Deflection (deg)")
# axs[6].set_xlabel("Time (s)")
# axs[6].legend(["Elevator Deflection"])
# axs[6].grid(True)

## X-Plane Data

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


# Comparison Plots
fig, axs = plt.subplots(7, 1, figsize=(9,10), sharex=True)

axs[0].plot(t_rk4, x_rk4[:,0], color="royalblue", linewidth=2)
axs[0].plot(time_xplane, xplane_U, color="royalblue", linewidth=2, linestyle='dashed' )
axs[0].set_ylabel("u (ft/s)")
axs[0].set_title("Nonlinear Longitudinal Aircraft States (RK4)")
axs[0].legend(["Sim U", "X-Plane U"], loc="center right")
axs[0].grid(True)

axs[1].plot(t_rk4, x_rk4[:,1], color="darkorange", linewidth=2)
axs[1].plot(time_xplane, xplane_W, color="royalblue", linewidth=2, linestyle='dashed' )
axs[1].set_ylabel("w (ft/s)")
axs[1].legend(["Sim W", "X-Plane W"], loc="center right")
axs[1].grid(True)

axs[2].plot(t_rk4, np.rad2deg(x_rk4[:,2]), color="forestgreen", linewidth=2) 
axs[2].plot(time_xplane, xplane_Q, color="royalblue", linewidth=2, linestyle='dashed' )       # Pitch Rate
axs[2].set_ylabel("Q (deg/s)")
axs[2].legend(["Sim Q", "X-Plane Q"], loc="center right")
axs[2].grid(True)

axs[3].plot(t_rk4, np.rad2deg(x_rk4[:,3]), color="crimson", linewidth=2)                           # Pitch
axs[3].plot(time_xplane, xplane_pitch, color="royalblue", linewidth=2, label = "X-Plane" )
axs[3].set_ylabel("θ (deg)")
axs[3].set_xlabel("Time (s)")
axs[3].legend(["Sim θ", "X-Plane θ"], loc="center right")
axs[3].grid(True)

axs[4].plot(t_rk4, np.rad2deg(alpha), color="purple", linewidth=2)                             # Alpha
axs[4].plot(time_xplane, xplane_alpha, color="blue", linewidth=2, linestyle='dashed')
axs[4].set_ylabel("α (deg)")
axs[4].set_xlabel("Time (s)")
axs[4].legend(["Sim α", "X-Plane α"], loc="center right")
axs[4].grid(True)

axs[5].plot(t_rk4,x_rk4[:,4], color="purple", linewidth=2)                                 # Altitude
axs[5].plot(time_xplane, xplane_alt, color="blue", linewidth=2, linestyle='dashed')
axs[5].set_ylabel("Altitude (ft)")
axs[5].set_xlabel("Time (s)")
axs[5].legend(["Sim alt", "X-Plane alt"], loc="center right")
axs[5].grid(True)

axs[6].plot(t_rk4,np.rad2deg(elevator_deflection_history), color="purple", linewidth=2)
axs[6].plot(time_xplane,xplane_elevator_deflection, color="blue", linewidth=2, linestyle ='dashed')
axs[6].set_ylabel("Elevator Deflection (deg)")
axs[6].set_xlabel("Time (s)")
axs[6].legend(["SimElevator Deflection", "X-Plane Elevator Deflection"], loc="center right")
axs[6].grid(True)


plt.tight_layout()
plt.show()



