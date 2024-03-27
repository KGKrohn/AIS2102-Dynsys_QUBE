import numpy as np
import control as ctrl
import matplotlib.pyplot as plt

Ra = 8.4
Kb = 0.042  # V/Rad/s
Kt = 0.042  # Nm/s
Ja = 4.0 * 10 ** (-6)  # kg-m2
JL = 0.6 * 10 ** (-6)  # Module attachment moment of Inertia 0.6 × 10−6 kg-m2
Jm = Ja + JL
Dm = 2.47 * 10 ** (-5)  # Fysisk
# Dm = 7.47 * 10 ** (-6)  # Fysisk
num = np.array([0, Kt / (Jm * Ra)])
den = np.array([1, (Dm + (Kb * Kt) / Ra) / Jm, 0])

sys = ctrl.tf2ss(num, den)


# Extract state-space matrices
def get_sim():
    Af = np.flipud(sys.A)
    A = np.fliplr(Af)
    B = np.rot90(sys.C, 1, (1, 0))
    C = np.array([0, 1 / (2 * np.pi * 1 / 60)])  # 1 / (2 * np.pi * 1 / 60)
    D = sys.D
    sys_ss = ctrl.StateSpace(A, B, C, D)
    print(A)
    print(B)
    print(C)
    print(D)
    # Set a step response on the system
    t, y = ctrl.step_response(sys_ss,0.4)
    return t+3, y


"""
# Plot step response
plt.plot(t, y)
plt.xlabel('Time')
plt.xlim((0, 0.1))
plt.ylabel('Response')
plt.title('Step Response')
plt.grid()
plt.show()
"""
