import numpy as np

def euler(f, tspan, y0, h, args=()):
    t0, tf = tspan # Split the start and end time into separate variables
    y0 = np.asarray(y0, dtype=float) # Ensures y0 is a vector

    N = int(np.round((tf - t0) / h)) # Calculates the total number of steps
    t = t0 + h * np.arange(N + 1)
    y = np.zeros((N + 1, y0.size), dtype=float) # Allocate the solution, y array with N+1 number of rows and columns equal to the amount of states
    y[0] = y0 # Input initial conditions

    for i in range(N):
        y[i + 1] = y[i] + h * f(t[i], y[i], *args) # Euler's method

    return t, y



def RK2(f, tspan, y0, h, args=()):
    t0, tf = tspan
    y0 = np.asarray(y0, dtype=float)

    N = int(np.round((tf - t0) / h))
    t = t0 + h * np.arange(N + 1)

    y = np.zeros((N + 1, y0.size), dtype=float)
    y[0] = y0

    for i in range(N):
        
        y[i + 1] = y[i] + h * f(t[i] + (h/2), y[i] + (h/2) * f(t[i] ,y[i], *args), *args)


    return t, y

def RK4(f, tspan, y0, h, args=()):
    t0, tf = tspan
    y0 = np.asarray(y0, dtype=float)

    N = int(np.round((tf - t0) / h))
    t = t0 + h * np.arange(N + 1)

    y = np.zeros((N + 1, y0.size), dtype=float)
    y[0] = y0

    for i in range(N):
        ti = t[i]
        yi = y[i]

        k1 = f(ti, yi, *args)
        k2 = f(ti + h/2, yi + (h/2) * k1, *args)
        k3 = f(ti + h/2, yi + (h/2) * k2, *args)
        k4 = f(ti + h, yi + h * k3, *args)

        y[i + 1] = yi + (h/6) * (k1 + 2*k2 + 2*k3 + k4)

    return t, y

