import numpy as np
import matplotlib.pyplot as plt

def naca_4_digit_airfoil(x,m,p):


    mask = x <= p 
    
    z = np.zeros_like(x)

    
    z[mask] = ((m/p**2) * (2 * p * (x[mask]) - (x[mask])**2))
    
    z[~mask] = (m/(1-p)**2) * (1 - 2*p + 2*p*(x[~mask]) - (x[~mask])**2)

    return z



def naca_4_digit_airfoil_slope(x,m,p):

    mask = x <= p 
    
    dzdx = np.zeros_like(x)

    
    dzdx[mask] = (m/p**2) * (2 * p - 2 * x[mask])
    
    dzdx[~mask] = (m/(1-p)**2) * (2*p - 2*x[~mask])

    return dzdx

def alpha_L_0(theta, m, p):
    x = 0.5 * (1 - np.cos(theta))
    result = (-1/np.pi) * np.trapezoid(naca_4_digit_airfoil_slope(x, m, p) * (np.cos(theta) - 1), theta)
    return result

def fourier_coefficients(theta,m,p,alpha,N):
    x = 0.5 * (1 - np.cos(theta))
    A_0 = alpha - 1/np.pi * np.trapezoid(naca_4_digit_airfoil_slope(x,m,p), theta)

    A = np.zeros(N)
    for n in range(1,N+1):
        A[n-1] = (2/np.pi) * np.trapezoid(naca_4_digit_airfoil_slope(x,m,p) * np.cos(n * theta), theta) 
    return A_0, A

def vortex_sheet_strength(theta,m,p,alpha,N,V_inf):

    A_0, A = fourier_coefficients(theta,m,p,alpha,N)
    gamma = np.zeros_like(theta)
    total = np.zeros_like(theta)
    for n in range(1, N+1):
        total += A[n-1] * np.sin(n * theta)
    gamma = 2 * V_inf * (A_0 * (1 + np.cos(theta)) / np.sin(theta) + total)
    return gamma

## NACA 2412

m = 0.02
p = 0.4
theta = np.linspace(0, np.pi, 100)[1:-1]
V_inf = 1
alpha = np.deg2rad(5)
N = 10

alpha_L_0 = alpha_L_0(theta,m,p)
print(f'Alpha_L_0: {alpha_L_0:.4f} radians | {np.degrees(alpha_L_0):.4f} degrees')

x = np.linspace(0,1,100)
A_0, A = fourier_coefficients(theta, m, p, np.deg2rad(5), 10)
print(A_0, A[:3])


gamma = vortex_sheet_strength(theta, m, p, np.deg2rad(5), 10,V_inf)
print(gamma)


z = naca_4_digit_airfoil(x, m, p)
x_plot = 0.5 * (1 - np.cos(theta))

plt.figure()
plt.plot(x_plot, gamma)
plt.xlabel('x/c')
plt.ylabel('γ (m/s)')
plt.title('Vortex Sheet Strength - NACA 2412')
plt.grid(True)

plt.figure()
plt.plot(x, z)
plt.title('NACA 2412 Airfoil')
plt.xlabel('x/c')
plt.ylabel('z/c')
plt.grid()
plt.axis('equal')
plt.show()