import numpy as np
from numpy.linalg import inv
import matplotlib.pyplot as plt

def calc_coefs(T, init_p, init_v, init_a, target_p, target_v, target_a):
    # A = np.array([
    #     [1, 0, 0, 0, 0, 0],  # q(0) = q0
    #     [0, 1, 0, 0, 0, 0],  # q'(0) = v0
    #     [0, 0, 2, 0, 0, 0],  # q''(0) = a0
    #     [1, T, T**2, T**3, T**4, T**5],  # q(T) = qf
    #     [0, 1, 2*T, 3*T**2, 4*T**3, 5*T**4],  # q'(T) = vf
    #     [0, 0, 2, 6*T, 12*T**2, 20*T**3]  # q''(T) = af
    # ])
    # b = np.array([init_p, init_v, init_a, target_p, target_v, target_a])
    
    # # Solve for the coefficients
    # coefficients = np.linalg.solve(A, b)
    # return coefficients

    A = np.matrix([
        [1, 0, 0, 0, 0, 0],  # q(0) = q0
        [0, 1, 0, 0, 0, 0],  # q'(0) = v0
        [0, 0, 2, 0, 0, 0],  # q''(0) = a0
        [1, T, T**2, T**3, T**4, T**5],  # q(T) = qf
        [0, 1, 2*T, 3*T**2, 4*T**3, 5*T**4],  # q'(T) = vf
        [0, 0, 2, 6*T, 12*T**2, 20*T**3]  # q''(T) = af
    ])
    
    B = np.matrix([init_p, init_v, init_a, target_p, target_v, target_a])
    B = B.transpose()
    A = inv(A)
    C = A*B
    
    return C.transpose()

def calc(coefs, t):
    A = np.matrix([
        [1, t, t**2, t**3, t**4, t**5],  # q(t) = qf
        [0, 1, 2*t, 3*t**2, 4*t**3, 5*t**4],  # q'(t) = vf
        [0, 0, 2, 6*t, 12*t**2, 20*t**3]  # q''(t) = af])
    ])
    B = coefs
    B = B.transpose()
    C = A*B
    
    return C



# def compute_trajectory(q0, v0, a0, qf, vf, af, T):
#     # Define the system of equations
#     A = np.array([
#         [1, 0, 0, 0, 0, 0],  # q(0) = q0
#         [0, 1, 0, 0, 0, 0],  # q'(0) = v0
#         [0, 0, 2, 0, 0, 0],  # q''(0) = a0
#         [1, T, T**2, T**3, T**4, T**5],  # q(T) = qf
#         [0, 1, 2*T, 3*T**2, 4*T**3, 5*T**4],  # q'(T) = vf
#         [0, 0, 2, 6*T, 12*T**2, 20*T**3]  # q''(T) = af
#     ])
#     b = np.array([q0, v0, a0, qf, vf, af])
    
#     # Solve for the coefficients
#     coefficients = np.linalg.solve(A, b)
#     return coefficients

# # Example usage
# q0, v0, a0 = 500, 1875, 0  # Initial conditions
# qf, vf, af = 1000, 0.0, 0.0  # Final conditions
# T = 1.0  # Trajectory duration
# coeffs = compute_trajectory(q0, v0, a0, qf, vf, af, T)
# print("Polynomial coefficients:", coeffs)

# # Verify the solution
# t = np.linspace(0, T, 100)
# q = coeffs[0] + coeffs[1]*t + coeffs[2]*t**2 + coeffs[3]*t**3 + coeffs[4]*t**4 + coeffs[5]*t**5
# dq = coeffs[1] + 2*coeffs[2]*t + 3*coeffs[3]*t**2 + 4*coeffs[4]*t**3 + 5*coeffs[5]*t**4
# ddq = 2*coeffs[2] + 6*coeffs[3]*t + 12*coeffs[4]*t**2 + 20*coeffs[5]*t**3

# print("Initial position:", q[0], "Expected:", q0)
# print("Initial velocity:", dq[0], "Expected:", v0)
# print("Initial acceleration:", ddq[0], "Expected:", a0)
# print("Final position:", q[-1], "Expected:", qf)
# print("Final velocity:", dq[-1], "Expected:", vf)
# print("Final acceleration:", ddq[-1], "Expected:", af)

# coefs = calc_coefs(1, 0, 0, 0, 1000, 0, 0)
# 0.5 => 500, 1875, 0
V0 = 1000
A0 = 0
P0 = 500
V_max = 1200
A_max = 2000
P_max = 1000
T1 = 3/2 * V_max/A_max

P1 = T1**2 * 8 * A_max/45 * 2
V1 = V_max
A1 = 0
coefs1 = calc_coefs(T1, P0, V0, A0, P1, V1, A1)
print(coefs1)
print(f"T1={T1}, P1={P1}, V1={V1}")
T2 = (P_max - 2*P1)/V1
P2 = P_max - P1
V2 = V1
A2 = A1
coefs2 = calc_coefs(T2, P1, V1, A1, P2, V2, A2)
print(coefs2)
print(f"T2={T2}, P2={P2}, V2={V2}")

T3 = T1
P3 = P_max
V3 = 0
A3 = 0
coefs3 = calc_coefs(T3, P2, V2, A2, P3, V3, A3)
print(coefs3)
print(f"T3={T3}, P3={P3}, V3={V3}")

poss = []
vels = []
accells = []
t = []

for i in np.linspace(0, T1 + T2 + T3, 1000):
    if i <= T1:
        res = calc(coefs1, i)
    elif i <= T1 + T2:
        res = calc(coefs2, i - T1)
    else:
        res = calc(coefs3, i - T1 - T2)

    poss.append(res[0, 0])
    vels.append(res[1, 0])
    accells.append(res[2, 0])
    t.append(i)

print(f"Pos: {poss[-1]}, Vel: {vels[-1]}, Acc: {accells[-1]}")
plt.plot(t, poss, label = "pos")
plt.plot(t, vels, label = "vel")
plt.plot(t, accells, label = "accel")
plt.legend()
plt.show()