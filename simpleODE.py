import numpy as np
import matplotlib.pyplot as plt


def euler_rhs(y):
    return y


# Initial condition
y0 = 1.0

# Step size
h = 1.0

# Time vector
tspan = np.arange(0, 5 + h, h)

# Preallocate arrays
y = np.zeros(len(tspan))
ydot = np.zeros(len(tspan) - 1)
delta_y = np.zeros(len(tspan) - 1)

# Set initial value
y[0] = y0

# Euler loop
for i in range(len(tspan) - 1):
    ydot[i] = euler_rhs(y[i])
    delta_y[i] = h * ydot[i]
    y[i + 1] = y[i] + delta_y[i]

# Exact solution
y_exact = np.exp(tspan)

# Plot Euler approximation with exact solution
plt.figure()
plt.plot(tspan, y, marker="o", label="Euler")
plt.plot(tspan, y_exact, marker="o", label="Exact: exp(t)", linestyle="--", color="red")

plt.xlabel("t")
plt.ylabel("y")
plt.title("Exact Solution vs Euler Approximation")
plt.grid(True)
plt.legend()

plt.show()
