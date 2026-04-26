import numpy as np

def euler(f, tspan, y0, dt, args=()):
    t0, tf = tspan # Split the start and end time into separate variables
    y0 = np.asarray(y0, dtype=float) # Ensures y0 is a vector

    N = int(np.round((tf - t0) / dt)) # Calculates the total number of steps
    t = t0 + dt * np.arange(N + 1)
    y = np.zeros((N + 1, y0.size), dtype=float) # Allocate the solution, y array with N+1 number of rows and columns equal to the amount of states
    y[0] = y0 # Input initial conditions

    for i in range(N):
        y[i + 1] = y[i] + dt * f(t[i], y[i], *args) # Euler's method

    return t, y



def RK2(f, tspan, y0, dt, args=()):
    t0, tf = tspan
    y0 = np.asarray(y0, dtype=float)

    N = int(np.round((tf - t0) / dt))
    t = t0 + dt * np.arange(N + 1)

    y = np.zeros((N + 1, y0.size), dtype=float)
    y[0] = y0

    for i in range(N):
        
        y[i + 1] = y[i] + dt * f(t[i] + (dt/2), y[i] + (dt/2) * f(t[i] ,y[i], *args), *args)


    return t, y

## Runge Kutta 4
    # f: function to integrate
    # tspan: start and end time
    # y0 : 
def RK4(f, tspan, y0, dt, args=()):
    t0, tf = tspan
    y0 = np.asarray(y0, dtype=float)

    N = int(np.round((tf - t0) / dt))
    t = t0 + dt * np.arange(N + 1)

    y = np.zeros((N + 1, y0.size), dtype=float)
    y[0] = y0

    for i in range(N):
        ti = t[i]
        yi = y[i]

        k1 = f(ti, yi, *args)
        k2 = f(ti + dt/2, yi + (dt/2) * k1, *args)
        k3 = f(ti + dt/2, yi + (dt/2) * k2, *args)
        k4 = f(ti + dt, yi + dt * k3, *args)

        y[i + 1] = yi + (dt/6) * (k1 + 2*k2 + 2*k3 + k4)

    return t, y