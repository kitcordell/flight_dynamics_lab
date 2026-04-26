from pathlib import Path
import sys

import numpy as np
import matplotlib.pyplot as plt
from scipy.integrate import solve_ivp

ROOT_DIR = Path(__file__).resolve().parents[1]
if str(ROOT_DIR) not in sys.path:
    sys.path.insert(0, str(ROOT_DIR))

from utils.integrators import euler, RK2, RK4

L = 1.0 # Pendulum Length, m
g = 9.8 # Gravity, m/s^2
h = 0.01  # step size

theta0 = np.deg2rad(30) # Initial angle
omega0 = np.deg2rad(0) # Initial Angular Velocity

t0 = 0 # Simulation Start Time
tf = 25.0 # Simulation end time

# Pendulum State Space Form
def pendulum(t, y, g, L):

    dydt = np.zeros_like(y)
    dydt[0] = y[1]
    dydt[1] = -(g/L)*np.sin(y[0])
    return dydt



# Euler Integration
t_e, y_e = euler(pendulum, (0.0, tf), [theta0, omega0], h, args=(g, L))

# RK2 
t_rk2, y_rk2 = RK2(pendulum, (0.0, tf), [theta0, omega0], h, args=(g, L))

# Python RK45 
t_eval = np.arange(t0, tf + h, h)  # match your integrator timestep
sol = solve_ivp(pendulum,[0, tf],[theta0, omega0],args=(g, L),method="RK45",t_eval=t_eval,)







# Angular Position

plt.figure()
plt.plot(t_e, np.rad2deg(y_e[:,0]), label="θ Euler")
plt.plot(t_rk2, np.rad2deg(y_rk2[:,0]), label="θ RK2")
plt.plot(sol.t, np.rad2deg(sol.y[0]), "--", label="θ RK45")
plt.xlabel("Time (s)")
plt.ylabel("Angle (deg)")
plt.title("Pendulum Angle")
plt.legend()
plt.show()



# Angular Velocity

plt.figure()
plt.plot(t_e, np.rad2deg(y_e[:,1]), label="ω Euler")
plt.plot(t_rk2, np.rad2deg(y_rk2[:,1]), label="ω RK2")
plt.plot(sol.t, np.rad2deg(sol.y[1]), "--", label="ω RK45")
plt.xlabel("Time (s)")
plt.ylabel("Angular velocity (deg/s)")
plt.title("Pendulum Angular Velocity")
plt.legend()
plt.show()


# def pendulum(t,y,g,L):
#     xdot = np.zeros_like(y)
#     A = np.array([[0,    1],
#                  [-g/L, 0]])
#     x = np.array([y[0],y[1]])
#     xdot = A @ x 
#     print(x)
#     print(xdot)
#     return xdot
