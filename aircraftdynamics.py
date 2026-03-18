import numpy as np
import matplotlib.pyplot as plt
from scipy import signal

# ————————————————
# AE 168 Lab 2 → Python Conversion

# Constants & flight condition
g = 32.17405   # ft/s²
U1 = 349       # ft/s
theta1 = 0     # rad

# Stability derivatives
Lp, Lb, Lr       = -2.3783, -9.4992,  0.3189
Ldelr, Ldela     =  1.6744,  19.9583
Np, Nb, Nr       = -0.0594,  8.3856, -0.5531
Ndelr, Ndela     = -2.9094, -1.2754
Yp, Yb, Yr       =  0.0,    -48.1999,  0.0
Ydelr, Ydela     = 26.7035,  0.0

# ————————————————
# Part B: State-space matrices
A = np.array([
    [0,    1,               0,            0, 0],
    [0,   Lp,              Lb,           Lr, 0],
    [g*np.cos(theta1)/U1, Yp/U1,         Yb/U1, Yr/U1 - 1, 0],
    [0,   Np,              Nb,           Nr, 0],
    [0,    0,               0,            1, 0]
])
B = np.array([
    [0,      0],
    [Ldelr,  Ldela],
    [Ydelr/U1, Ydela/U1],
    [Ndelr,  Ndela],
    [0,      0]
])
C = np.eye(5)
D = np.zeros((5, 2))
sys_lat = signal.StateSpace(A, B, C, D)

# Part C: Eigenvalues & damping
eigvals, _ = np.linalg.eig(A)
damping = -np.real(eigvals) / np.abs(eigvals)
print("Eigenvalues:", eigvals)
print("Damping ratios:", damping)

# Part E: Controllability
Co = B
for i in range(1, 5):
    Co = np.hstack((Co, np.linalg.matrix_power(A, i).dot(B)))
print("Controllability rank:", np.linalg.matrix_rank(Co))

# Part F: Open-loop TF δa → φ
B_da = B[:, 1].reshape(-1, 1)
C_phi = np.array([[1, 0, 0, 0, 0]])
D_phi = np.zeros((1, 1))
num, den = signal.ss2tf(A, B_da, C_phi, D_phi)
tf_da2phi = signal.TransferFunction(num[0], den)
print("TF δa→φ:", tf_da2phi)

# Part D: Doublet input simulation
t = np.linspace(0, 20, 2001)
u_a = np.zeros_like(t)
u_a[(t>=1)&(t<2)] = 5
u_a[(t>=2)&(t<3)] = -5
u_r = np.zeros_like(t)
u_r[(t>=1)&(t<2)] = 5
u_r[(t>=2)&(t<3)] = -5
U = np.vstack((u_r, u_a)).T

t_out, y_out, x_out = signal.lsim(sys_lat, U, t)
phi, p, beta, r, psi = x_out.T





# Plot Part D
fig, axs = plt.subplots(3, 3, figsize=(12, 8))
axs[0,0].plot(t, u_a);    axs[0,0].set(title='Aileron Input',   xlim=(0,10), ylim=(-6,6))
axs[0,1].plot(t, u_r);    axs[0,1].set(title='Rudder Input',   xlim=(0,10), ylim=(-6,6))
axs[0,2].plot(t_out, phi);axs[0,2].set(title='Roll Angle φ',   xlim=(0,20), ylim=(-6,32))
axs[1,0].plot(t_out, p);  axs[1,0].set(title='Roll Rate p',    xlim=(0,20), ylim=(-40,40))
axs[1,1].plot(t_out, beta);axs[1,1].set(title='Sideslip β',     xlim=(0,10), ylim=(-6,6))
axs[1,2].plot(t_out, r);  axs[1,2].set(title='Yaw Rate r',     xlim=(0,20), ylim=(-6,10))
axs[2,0].plot(t_out, psi);axs[2,0].set(title='Yaw Angle ψ',    xlim=(0,20))
axs[2,1].axis('off'); axs[2,2].axis('off')
for ax in axs.flat:
    ax.grid(True); ax.set(xlabel='Time [s]', ylabel='Deg')
plt.tight_layout(); plt.show()

# Part G: PID closed-loop example
Kp, Ki, Kd = 2.0, 0.5, 0.1
num_pid = [Kd, Kp, Ki]
den_pid = [1, 0]
num_ol = np.convolve(num_pid, tf_da2phi.num)
den_ol = np.convolve(den_pid, tf_da2phi.den)
den_cl = np.polyadd(den_ol, num_ol)
cl_tf = signal.TransferFunction(num_ol, den_cl)

t_cl, y_cl = signal.step(cl_tf, T=np.linspace(0, 10, 1001))
plt.figure()
plt.plot(t_cl, y_cl)
plt.title('Closed-Loop Roll Response')
plt.xlabel('Time [s]'); plt.ylabel('φ [deg]'); plt.grid(True)
plt.show()
