import numpy as np

# UR5e Joint value (radian)
j = np.array([j1_r, j2_r, j3_r, j4_r, j5_r, j6_r])

# UR5e DH parameters
a = np.array([0., -0.425, -0.3922, 0., 0., 0.])

d = np.array([0.1625, 0., 0., 0.1333, 0.0997, 0.0996])

alpha = np.array([np.pi/2, 0., 0., np.pi/2, -np.pi/2, 0.])

# cos, sin
def c(theta):
    cc = np.cos(theta)
    return cc
def s(theta):
    ss = np.sin(theta)
    return ss

# Jacobian matrix
J11 = (d[3]*c(j[0]))+s(j[0])*(a[1]*s(j[1])+a[2]*s(j[1]+j[2])+d[4]*s(j[1]+j[2]+j[3]))
J12 = -(c(j[0])*(a[1]*c(j[1])+a[2]*c(j[1]+j[2])+d[4]*c(j[1]+j[2]+j[3])))
J13 = -(c(j[0])*(a[2]*c(j[1]+j[2])+d[4]*c(j[1]+j[2]+j[3])))
J14 = -(d[4]*c(j[0])*c(j[1]+j[2]+j[3]))
J15 = 0
J16 = 0
J21 = (d[3]*s(j[0]))-c(j[0])*(a[1]*s(j[1])+a[2]*s(j[1]+j[2])+d[4]*s(j[1]+j[2]+j[3]))
J22 = -(s(j[0])*(a[1]*c(j[1])+a[2]*c(j[1]+j[2])+d[4]*c(j[1]+j[2]+j[3])))
J23 = -(s(j[0])*(a[2]*c(j[1]+j[2])+d[4]*c(j[1]+j[2]+j[3])))
J24 = -(d[4]*s(j[0])*c(j[1]+j[2]+j[3]))
J25 = 0
J26 = 0
J31 = 0
J32 = -(a[1]*s(j[1]))-(a[2]*s(j[1]+j[2]))-(d[4]*s(j[1]+j[2]+j[3]))
J33 = -(a[2]*s(j[1]+j[2]))-(d[4]*s(j[1]+j[2]+j[3]))
J34 = -(d[4]*s(j[1]+j[2]+j[3]))
J35 = 0
J36 = 0
J = np.array([[J11, J12, J13, J14, J15, J16], [J21, J22, J23, J24, J25, J26], [J31, J32, J33, J34, J35, J36]])