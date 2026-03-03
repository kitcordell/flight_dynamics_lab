import numpy as np
import matplotlib.pyplot as plt
# from scipy import signal
from integrators import euler, RK2
# from scipy.integrate import solve_ivp



t0 = 0
tf = 900
h = 0.01
U_0 = 110 * 1.6 # initial velocity, [ft/s]


def longitudinal_aircraft_dynamics(t,x):
    u, w, q, theta = x
    


    g = 32
    rho = 0.002378 # air density, [slug/ft^3]
    bw = 35.8 # wing span, [ft]
    cbar = 4.9 # average chord, [ft]
    S = bw * cbar # wing surface area [ft^2]
    AR = bw / cbar
    delta_e = np.deg2rad(0) # elevator deflection, [rad]
    thrust = 0 # total thrust in, [lb]
    K = 0.054

    I_yy = 1346 # moment of inertia about pitch axis, [slug*ft^2]

    
    m = 2500 / g # aircraft mass, [slugs]
    
    V = np.sqrt(u**2+w**2)
    qbar = 0.5 * rho * V**2 # dynamic pressure, [lb/ft^2]
    alpha = np.arctan2(w,u)
    
    # Lift Calculations
    C_L_alpha = 5.143 # lift curve slope, [1/rad]
    C_L_0 = 0.31 # lift at 0 aoa
    C_L_delta_e = 0.43 # lift due to elevator
  
    
    C_L = C_L_0 + C_L_alpha * alpha +C_L_delta_e * delta_e  # coefficient of lift
    L = qbar * S * C_L # total lift

    # Drag Calculations
    C_D_0 = 0.031 # drag at 0 aoa
    C_D = C_D_0 + C_L**2 * K
    D = qbar * S * C_D
    
    # Pitching Moment Calculation
    C_m_delta_e = -1.28
    C_m_alpha = -0.89
    C_mq = -12.4
    C_m_0 = -0.015
    C_m = C_m_0 + C_m_alpha * alpha + C_m_delta_e * delta_e + C_mq * ((q*cbar)/(2*V))
    M = qbar * S * cbar * C_m

    
    X = -D * np.cos(alpha) + L * np.sin(alpha) + thrust
    Z = -D * np.sin(alpha) - L * np.cos(alpha)
    

    xdot = np.zeros_like(x)


    xdot[0] = X/m - g*np.sin(theta) - q*w
    xdot[1] = -Z/m + g*np.cos(theta) + q*u
    xdot[2] = M/I_yy
    xdot[3] = q

    

    return xdot



t_rk2, x_rk2 = RK2(longitudinal_aircraft_dynamics, (0.0, 60.0), [U_0, 0, 0, 0], h)
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