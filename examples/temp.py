
from pathlib import Path
import sys

import numpy as np
import matplotlib.pyplot as plt
from scipy.integrate import solve_ivp

ROOT_DIR = Path(__file__).resolve().parents[1]
if str(ROOT_DIR) not in sys.path:
    sys.path.insert(0, str(ROOT_DIR))

from utils.integrators import euler, RK2

a = np.array([[1,2],[5,5]])

b = np.array([[3],
              [2]])

c = a @ b

print(a, b, c)
