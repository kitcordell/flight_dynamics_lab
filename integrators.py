import numpy as np

def euler(f, tspan, y0, h, args=()):
    t0, tf = tspan
    y0 = np.asarray(y0, dtype=float)

    N = int(np.round((tf - t0) / h))
    t = t0 + h * np.arange(N + 1)

    y = np.zeros((N + 1, y0.size), dtype=float)
    y[0] = y0

    for i in range(N):
        y[i + 1] = y[i] + h * f(t[i], y[i], *args)

    return t, y

