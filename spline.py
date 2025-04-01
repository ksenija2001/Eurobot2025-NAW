import numpy as np
import matplotlib.pyplot as plt
import math
import time

def calc_bez(p, t, order):
    sum = 0
    for i in range(order+1):
        sum += ((math.factorial(order))/(math.factorial(i)*math.factorial(order - i))) * np.power(1-t, order-i) * t**i * p[i]
    return sum

def calc_bez_speed(p, t, order):
    sum = 0
    for i in range(order):
        sum += order * ((math.factorial(order-1))/(math.factorial(i)*math.factorial(order-1-i))) * np.power(1-t, order-1-i) * t**i * (p[i+1] - p[i])
    return sum

def calc_bez_accel(p, t, order):
    sum = 0
    for i in range(order-1):
        sum += order * (order - 1) * ((math.factorial(order-2))/(math.factorial(i)*math.factorial(order-2-i))) * np.power(1-t, order-2-i) * t**i * (p[i+2] - 2*p[i+1] + p[i])
    return sum

def radius(x_s, x_a, y_s, y_a):
    return (x_s*y_a - y_s*x_a)/math.sqrt((math.pow(x_s**2 + y_s**2, 3)))

def find(px, py, order, x, y):
    left = 0
    right = 1
    for i in range(10):

        lx = calc_bez(px, left, order)
        ly = calc_bez(py, left, order)

        rx = calc_bez(px, right, order)
        ry = calc_bez(py, right, order)

        l_distance = math.sqrt((lx-x)**2+(ly-y)**2)
        r_distance = math.sqrt((rx-x)**2+(ry-y)**2)

        if r_distance < l_distance:
            left = left/2+right/2
        else:
            right = left/2+right/2

    return left/2+right/2



x = []
y = []
x_s = []
y_s = []
x_a = []
y_a = []
a = []
t = []
v = []
r = []
v_l = []
v_r = []

for i in np.linspace(0, 1, num=200):
    x.append(calc_bez([0, 0, 0, 1000, 1000, 1000], i, 5))
    y.append(calc_bez([0, 0, 500, 500, 1000, 1000], i, 5))
    x_s.append(calc_bez_speed([0, 0, 0, 1000, 1000, 1000], i, 5))
    y_s.append(calc_bez_speed([0, 0, 500, 500, 1000, 1000], i, 5))
    x_a.append(calc_bez_accel([0, 0, 0, 1000, 1000, 1000], i, 5))
    y_a.append(calc_bez_accel([0, 0, 500, 500, 1000, 1000], i, 5))
    r.append(radius(x_s[-1], x_a[-1], y_s[-1], y_a[-1]) ** -1)

    t.append(i)


for i in range(200):
    v.append(math.sqrt(x_s[i]**2 + y_s[i]**2))
    a.append(math.sqrt(x_a[i]**2 + y_a[i]**2))  
    v_l.append((v[i]*(r[i]-100)) / r[i])
    v_r.append((v[i]*(r[i]+100)) / r[i])

max = 0
for i in range(200):
    if(v[i] > max):
        max = v[i]
koef = 500/max

v = [koef*i for i in v]
v_l = [koef*i for i in v_l]
v_r = [koef*i for i in v_r]

figure, axis = plt.subplots(2, 4)
s = time.time()
ttt = find([0, 0, 0, 1000, 1000, 1000], [0, 0, 500, 500, 1000, 1000], 5, 725, 268)
print(time.time()-s)
print(ttt)
# a = calc_bez([0, 0, 0, 1000, 1000, 1000], ttt, 5)
# b = calc_bez([0, 0, 500, 500, 1000, 1000], ttt, 5)

axis[0, 0].set_xlim(0, 1000)
axis[0, 0].set_ylim(0, 1000)
axis[0, 0].plot(x, y)
#axis[0, 0].scatter(725, 268)
#axis[0, 0].scatter(a, b)
axis[0, 0].set_title("Position")
axis[1, 0].plot(t, v)
axis[1, 0].plot(t, v_l)
axis[1, 0].plot(t, v_r)
axis[1, 0].set_title("Velocity")
axis[0, 1].plot(t, x_s)
axis[0, 1].set_title("X speed")
axis[1, 1].plot(t, y_s)
axis[1, 1].set_title("Y speed")

axis[0, 2].plot(t, x)
axis[0, 2].set_title("X")
axis[1, 2].plot(t, y)
axis[1, 2].set_title("Y")
axis[0, 3].plot(t, a)
axis[0, 3].set_title("A")










plt.show()