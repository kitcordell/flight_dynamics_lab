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


def RK4_controlled(f, controller, tspan, y0, dt, args=()):
    """Integrate f(t, y, u, *args) with one held control command per step."""
    t0, tf = tspan
    if dt <= 0.0:
        raise ValueError("dt must be greater than zero")

    y0 = np.asarray(y0, dtype=float)
    N = int(np.round((tf - t0) / dt))
    if N < 1:
        raise ValueError("tspan must contain at least one integration step")

    t = t0 + dt * np.arange(N + 1)
    y = np.zeros((N + 1, y0.size), dtype=float)
    y[0] = y0

    first_control = np.asarray(controller(t[0], y[0]), dtype=float)
    if first_control.ndim != 1:
        raise ValueError("controller output must be a one-dimensional vector")
    u = np.zeros((N + 1, first_control.size), dtype=float)

    for i in range(N):
        ti = t[i]
        yi = y[i]
        ui = first_control if i == 0 else np.asarray(controller(ti, yi), dtype=float)
        if ui.shape != first_control.shape:
            raise ValueError("controller output shape changed during integration")
        u[i] = ui

        k1 = f(ti, yi, ui, *args)
        k2 = f(ti + dt/2, yi + (dt/2) * k1, ui, *args)
        k3 = f(ti + dt/2, yi + (dt/2) * k2, ui, *args)
        k4 = f(ti + dt, yi + dt * k3, ui, *args)
        y[i + 1] = yi + (dt/6) * (k1 + 2*k2 + 2*k3 + k4)

    u[-1] = u[-2]
    return t, y, u
