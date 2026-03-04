import numpy as np
import matplotlib.pyplot as plt
from scipy import signal
from integrators import euler, RK2
from scipy.integrate import solve_ivp
from aircraft_longitudinal_dynamics import aircraft_longitudinal_dynamics



t0 = 0
tf = 200
h = 0.01
U_0 = 110 * 1.6 # initial velocity, [ft/s]


params = {
    "g": 32,                                  # Acceleration of gravity
    "rho": 0.002378,                          # air density, [slug/ft^3]
    "bw": 35.8,                               # wing span, [ft]
    "cbar": 4.9,                              # average chord, [ft]
    "K": 0.054,
    "I_yy": 1346,                             # moment of inertia about pitch axis, [slug*ft^2]
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

t_rk2, x_rk2 = RK2(aircraft_longitudinal_dynamics, (0.0, tf), [U_0, 0, 0, 0], h, args=(params,))
alpha = np.arctan2(x_rk2[:,1], x_rk2[:,0])

fig, axs = plt.subplots(5, 1, figsize=(9,10), sharex=True)

axs[0].plot(t_rk2, x_rk2[:,0]/1.68, color="royalblue", linewidth=2)
axs[0].set_ylabel("u (ft/s)")
axs[0].set_title("Nonlinear Longitudinal Aircraft States (RK2)")
axs[0].legend(["u — forward body velocity"])
axs[0].grid(True)

axs[1].plot(t_rk2, x_rk2[:,1], color="darkorange", linewidth=2)
axs[1].set_ylabel("w (ft/s)")
axs[1].legend(["w — vertical body velocity (z-axis, positive down)"])
axs[1].grid(True)

axs[2].plot(t_rk2, np.rad2deg(x_rk2[:,2]), color="forestgreen", linewidth=2)
axs[2].set_ylabel("q (deg/s)")
axs[2].legend(["q — pitch rate"])
axs[2].grid(True)

axs[3].plot(t_rk2, np.rad2deg(x_rk2[:,3]), color="crimson", linewidth=2)
axs[3].set_ylabel("θ (deg)")
axs[3].set_xlabel("Time (s)")
axs[3].legend(["θ — pitch angle (aircraft attitude)"])
axs[3].grid(True)

axs[4].plot(t_rk2, np.rad2deg(alpha), color="purple", linewidth=2)
axs[4].set_ylabel("α (deg)")
axs[4].set_xlabel("Time (s)")
axs[4].legend(["α — angle of attack"])
axs[4].grid(True)


plt.tight_layout()
plt.show()
