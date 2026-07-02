import numpy as np


def velocity_to_body(V, gamma, theta):
    alpha = theta - gamma
    U = V * np.cos(alpha)
    W = V * np.sin(alpha)
    return U, W, alpha


def body_to_velocity(U, W, theta):
    V = np.sqrt(U**2 + W**2)
    alpha = np.arctan2(W, U)
    gamma = theta - alpha
    return V, gamma, alpha


def aero_to_body(L,D,thrust,alpha):
    X = -D * np.cos(alpha) + L * np.sin(alpha) + thrust # X forces
    Z = -D * np.sin(alpha) - L * np.cos(alpha)  # Z forces

    return X, Z