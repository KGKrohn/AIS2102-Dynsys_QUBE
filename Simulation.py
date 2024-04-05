import numpy as np
import control as ctrl
import matplotlib.pyplot as plt

Ra = 8.4
Kb = 0.042  # V/Rad/s
Kt = 0.042  # Nm/s
Ja = 4.0 * 10 ** (-6)  # kg-m2
JL = 0.6 * 10 ** (-6)  # Module attachment moment of Inertia 0.6 × 10−6 kg-m2
Jd = 1.63 * 10**(-5)
Jm = Ja + JL + Jd
Dm = 2.47 * 10 ** (-5)  # Fysisk
# Dm = 7.47 * 10 ** (-6)  # Fysisk
num = np.array([0, Kt / (Jm * Ra)])
den = np.array([1, (Dm + (Kb * Kt) / Ra) / Jm, 0])
sys = ctrl.tf2ss(num, den)


def calc_l_observer():
    sys = ctrl.tf2ss(num, den)
    Af = np.flipud(sys.A)
    A = np.fliplr(Af)
    B = np.rot90(sys.C, 1, (1, 0))
    C = np.array([1, 0])  # 1 / (2 * np.pi * 1 / 60)
    D = sys.D
    print("A:", A,"B:",B,"C: ",C,"D",D)
    p1 = -41.2689 + 56.3055j
    p2 = -41.2689 - 56.3055j
    poles = [p1, p2]

    print(np.rot90(Af))
    l = ctrl.acker(np.rot90(Af), [[1],[0]], poles)
    print("l:",l)
    Acl = A - l * C

    return Acl


calc_l_observer()


# Extract state-space matrices
def get_sim():
    Af = np.flipud(sys.A)
    A = np.fliplr(Af)
    B = np.rot90(sys.C, 1, (1, 0))
    C = np.array([0, 1 / (2 * np.pi * 1 / 60)])  # 1 / (2 * np.pi * 1 / 60)
    D = sys.D
    sys_ss = ctrl.StateSpace(A, B, C, D)
    # Set a step response on the system
    t, y = ctrl.step_response(sys_ss, 0.4)
    return t + 3, y


def get_sim_ramp():
    Af = np.flipud(sys.A)
    A = np.fliplr(Af)
    B = np.rot90(sys.C, 1, (1, 0))
    C = np.array([0, 1 / (2 * np.pi * 1 / 60)])  # 1 / (2 * np.pi * 1 / 60)
    D = sys.D
    sys_ss = ctrl.StateSpace(A, B, C, D)
    # Set a step response on the system

    dt = 0.01
    duration = 10
    tot_samp = int(duration / dt)
    time_axis = np.linspace(0, duration, tot_samp)
    ramp = 18 * time_axis / duration

    t, y = ctrl.forced_response(sys_ss, time_axis, ramp)
    return t + 3, y


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
