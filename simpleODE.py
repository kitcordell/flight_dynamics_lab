import numpy as np
import matplotlib.pyplot as plt
from integrators import euler, RK2
from scipy.integrate import solve_ivp

L = 1.0 # Pendulum Length, m
g = 9.8 # Gravity, m/s^2
h = 0.01  # step size

theta0 = np.deg2rad(90) # Initial angle
omega0 = np.deg2rad(0) # Initial Angular Velocity

t0 = 0 # Simulation Start Time
tf = 25.0 # Simulation end time

# Pendulum State Space Form
def ODE(t, y):

    dydt = np.zeros_like(y)
    dydt[0] = y[0]
  
    return dydt



t_e, y_e = euler(ODE, (0.0, tf), [theta0, omega0], h, args=())

# --- RK2 ---
t_rk2, y_rk2 = RK2(ODE, (0.0, tf), [theta0, omega0], h, args=())

# Python RK45 
t_eval = np.arange(t0, tf + h, h)  # match your integrator timestep
sol = solve_ivp(ODE,[0, tf],[theta0, omega0],args=(),method="RK45",t_eval=t_eval,)







plt.figure()
plt.plot(t_e, np.rad2deg(y_e[:,0]), label="Euler")
plt.plot(t_rk2, np.rad2deg(y_rk2[:,0]), label="RK2")
plt.plot(sol.t, np.rad2deg(sol.y[0]), "--", label="RK45")
plt.xlabel("Time (s)")
plt.ylabel("Angle (deg)")
plt.title("dy/dt=y Approximation")
plt.legend()
plt.show()


