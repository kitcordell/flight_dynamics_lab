import numpy as np
import matplotlib.pyplot as plt

# Parameters
g = 9.81
L = 1.0

# Create grid in state space
theta = np.linspace(-2*np.pi, 2*np.pi, 25)   # angle
omega = np.linspace(-8, 8, 25)               # angular velocity

theta, omega = np.meshgrid(theta, omega)

# State derivatives
dtheta = omega
domega = -(g/L) * np.sin(theta)

mag = np.sqrt(dtheta**2 + domega**2)


# Plot
plt.figure(figsize=(10, 6))
plt.quiver(theta, omega, dtheta, domega, mag, angles='xy')
plt.xlabel(r'$\theta$ (rad)')
plt.ylabel(r'$\dot{\theta}$ (rad/s)')
plt.title('Pendulum State-Space Vector Field')
plt.grid(True)
plt.show()