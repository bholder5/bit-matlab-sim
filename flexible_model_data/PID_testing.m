syms theta1 theta2 dtheta1 dtheta2
x = []

kp = 10
ki = 15
kip = 1


A = [0 1 0 0; 0 kp/100 ki/100 kip/100; 0 1 -0.01 0; 1 0 0 -0.01]
eig(A)