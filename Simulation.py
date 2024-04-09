import numpy as np
import control as ctrl
import matplotlib.pyplot as plt

Ra = 8.4
Kb = 0.042#V/Rad/s
Kb = Kb *(2 * np.pi * 1 / 60)#%rad/s to rpm.
Kt = 0.042 # Nm/s
Ja = 4.0 * 10**(-6)# kg-m2
JL = 0.6 * 10**(-6)#  % Module attachment moment of Inertia 0.6 × 10−6 kg-m2
Jd = 1.63 * 10**(-5)
Jm = Ja + JL# + Jd
Dm = 2.47 * 10**(-5)#Fysisk
num = np.array([Kt / (Jm * Ra)])
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

def get_sim_val():
    tc = 0.141
    td = 0.024
    T = tc + td
    v = 1164
    num_1 = np.array([v / T])
    den_1 = np.array([1, 1/T, 0])
    sys_1 = ctrl.tf2ss(num_1, den_1)
    Af = np.flipud(sys_1.A)
    A = np.fliplr(Af)
    B = np.rot90(sys_1.C, 1, (1, 0))
    C = np.array([1, 0])
    D = sys_1.D

    sys_ss = ctrl.StateSpace(A, B, C, D)
    t, y = ctrl.step_response(sys_ss, 2)
    return t, y


# Extract state-space matrices
def get_sim(actual):
    Af = np.flipud(sys.A)
    A = np.fliplr(Af)
    B = np.rot90(sys.C, 1, (1, 0))
    K = Kt / (Jm * Ra)
    alpha  = (Dm + (Kb * Kt) / Ra) / Jm
    Ts = K/alpha
    scale = actual/Ts
    C = np.array([0,scale])
    D = sys.D
    sys_ss = ctrl.StateSpace(A, B, C, D)
    # Set a step response on the system
    t, y = ctrl.step_response(sys_ss, 2)
    return t, y


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
