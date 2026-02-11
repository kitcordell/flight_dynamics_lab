import numpy as np
import matplotlib.pyplot as plt
from scipy.integrate import solve_ivp

L = 1 # m
g = 9.8 #m/s^2
h = 0.01

theta0 = np.deg2rad(180) # initial angle, deg
omega0 = np.deg2rad(0)




simend = 10 # sim end time
N = int(simend/h) # total number of data points 
tspan = np.linspace(0,simend,N+1)  # 

theta=np.zeros((N+1,2)) # initialize theta array

theta[0][0] = theta0 # set initial position in matrix
theta[0][1] = omega0 # set initial angular velocit in matrix

def pendulum(theta,g,L):
    omega = np.zeros(2) # initialize differential matrix

    omega[0] = theta[1] # Pendulum ODE in state space
    omega[1] = (-g/L) * np.sin(theta[0])

    return omega 

for i in range(0,N):
    theta[i+1][:] = theta[i][:] + h * pendulum(theta[i][:],g,L) # Euler's method loop

plt.figure(1)
plt.plot(tspan, theta[:, 0]) # Angular Position Plot 
plt.title("Angluar Position")

plt.figure(2)
plt.plot(tspan, theta[:,1]) # Angular Velocity Plot
plt.title("Angular Velocity")
plt.show()


