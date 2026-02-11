import numpy as np
import matplotlib.pyplot as plt
from scipy.integrate import solve_ivp

L = 1 # m
g = 9.8 #m/s^2
h = 0.01
simend = 40 # sim end time

theta0 = np.deg2rad(90) # initial angle, deg
omega0 = np.deg2rad(0)





N = int(simend/h) # total number of data points 
tspan = np.linspace(0,simend,N+1)  # 

theta=np.zeros((N+1,2)) # initialize theta array

theta[0][0] = theta0 # set initial position in matrix
theta[0][1] = omega0 # set initial angular velocit in matrix

def pendulum(t,theta,g,L):
    omega = np.zeros(2) # initialize differential matrix

    omega[0] = theta[1] # Pendulum ODE in state space
    omega[1] = (-g/L) * np.sin(theta[0])

    return omega 

for i in range(0,N):
    theta[i+1][:] = theta[i][:] + h * pendulum(tspan[i],theta[i][:],g,L) # Euler's method loop







plt.figure()
plt.plot(tspan, np.rad2deg(theta[:,0]), label="Euler")

sol = solve_ivp(pendulum,[0, simend],[theta0, omega0],args=(g, L),method="RK45",t_eval=tspan)

plt.plot(sol.t, np.rad2deg(sol.y[0]), '--', label="RK45")
plt.xlabel("Time (s)")
plt.ylabel("Angle (deg)")
plt.legend()
plt.title("Angle Comparison (deg)")
plt.show()


