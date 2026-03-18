
import numpy as np
import matplotlib.pyplot as plt
from integrators import euler, RK2
from scipy.integrate import solve_ivp

a = np.array([[1,2],[5,5]])

b = np.array([[3],
              [2]])

c = a @ b

print(a, b, c)
