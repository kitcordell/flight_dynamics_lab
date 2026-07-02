from dataclasses import dataclass
import numpy as np

@dataclass
class Wing:
    span: float          # b
    root_chord: float    # c_r
    taper_ratio: float   # c_tip / c_root
    alpha_root: float    # geometric AoA at root, radians
    twist: float         # linear washout at tip, radians
    alpha_L0: float      # zero-lift AoA from thin airfoil theory, radians
    a0: float = 2 * np.pi  # section lift curve slope


def local_chord(y,wing):
    
    c =  wing.root_chord + (wing.root_chord * (wing.taper_ratio - 1) / (wing.span/2)) * np.abs(y)

    return c
    
def local_alpha(y,wing):
    
    alpha = wing.alpha_root + (wing.twist / (wing.span/2)) * np.abs(y)

    return alpha

def cosine_spacing(wing,N):
    
    theta = np.linspace(0, np.pi, N+2)[1:-1]
    y = (wing.span/2) * np.cos(theta)

    return y, theta

def build_system(wing,N):

    y, theta = cosine_spacing(wing,N)
    alpha= local_alpha(y,wing)
    c = local_chord(y,wing)

    M= np.zeros((N,N))
    r = np.zeros(N)

    for n in range(N):
        for j in range(1,N+1):

         M[n,j-1] = np.sin(j*theta[n]) * (((j*c[n] * wing.a0) / (4 * wing.span * np.sin(theta[n]))) + 1)

        r[n] = ((wing.a0 * c[n] / (4 * wing.span))) * (alpha[n] - wing.alpha_L0)

    return M, r

def solve_liftin_line(wing,N):
    
    M, r = build_system(wing,N)
    A = np.linalg.solve(M,r)

    return A

def compute_aero(wing,N):

    A = solve_liftin_line(wing,N+1)
    S = (wing.span/2) * (wing.root_chord  + wing.taper_ratio*wing.root_chord)
    AR = wing.span**2 / S
    C_L = np.pi * AR * A[0]

    sum = np.zeros(N)
    for j in range(1,N):
        sum[j] = j  * A[j-1]**2
    sum = np.sum(sum)

    C_D_i = np.pi * AR * sum

    e = A[0]**2 / sum
    print(AR)
    return C_L, C_D_i, e

def spanwise_circulation(wing, N, V_inf):
    A = solve_liftin_line(wing, N)
    y, theta = cosine_spacing(wing, N)

    Gamma = np.zeros(N)
    for n in range(N):
        for j in range(1, N+1):
            Gamma[n] += A[j-1] * np.sin(j * theta[n])
    Gamma *= 2 * wing.span * V_inf

    return y, Gamma


wing = Wing(
    span=60, 
    root_chord=10, 
    taper_ratio=0.5, 
    alpha_root=np.deg2rad(5),
    twist=np.deg2rad(-2), 
    alpha_L0=np.deg2rad(-2), 
    a0=2*np.pi )

c = local_chord(-wing.span/2,wing)
print(c)





import matplotlib.pyplot as plt

y, Gamma = spanwise_circulation(wing, 100, 1)

# Elliptical reference
Gamma_elliptic = Gamma.max() * np.sqrt(1 - (y / (wing.span/2))**2)

plt.plot(y, Gamma, label='Tapered Wing')
plt.plot(y, Gamma_elliptic, '--', label='Elliptical')
plt.xlabel('Spanwise position y (m)')
plt.ylabel('Circulation Γ (m²/s)')
plt.title('Spanwise Circulation Distribution')
plt.legend()
plt.grid(True)
plt.show()