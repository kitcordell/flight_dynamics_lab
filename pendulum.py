import numpy as np
import matplotlib.pyplot as plt
from integrators import euler
from scipy.integrate import solve_ivp

L = 1.0 # Pendulum Length, m
g = 9.8 # Gravity, m/s^2
h = 0.01  # step size

theta0 = np.deg2rad(90) # Initial angle
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
t, y = euler(pendulum, (0.0, tf), [theta0, omega0], h, args=(g, L))


# Python RK45 Integration
sol = solve_ivp(pendulum,[0, tf],[theta0, omega0],args=(g, L),method="RK45",t_eval=t)

plt.figure()

# Angular Position Plots
plt.plot(t, np.rad2deg(y[:,0]), color="blue", label="θ Euler")
plt.plot(sol.t, np.rad2deg(sol.y[0]), "--", color="blue", label="θ RK45")

# Angular Velocity Plots
plt.plot(t, np.rad2deg(y[:,1]), color="red", label="ω Euler")
plt.plot(sol.t, np.rad2deg(sol.y[1]), "--", color="red", label="ω RK45")

plt.xlabel("Time (s)")
plt.ylabel("State (deg / deg/s)")
plt.title("Pendulum States")
plt.legend()
plt.show()