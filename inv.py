import sympy as sym

sym.init_printing()

t = sym.symbols('t')
matrix = sym.Matrix([
        [1, 0, 0, 0, 0, 0],  # q(0) = q0
        [0, 1, 0, 0, 0, 0],  # q'(0) = v0
        [0, 0, 2, 0, 0, 0],  # q''(0) = a0
        [1, t, t**2, t**3, t**4, t**5],  # q(t) = qf
        [0, 1, 2*t, 3*t**2, 4*t**3, 5*t**4],  # q'(t) = vf
        [0, 0, 2, 6*t, 12*t**2, 20*t**3]  # q''(t) = af
])

print(matrix.inv())