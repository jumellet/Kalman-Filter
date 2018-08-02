#from reception import *
from math import *
import numpy as np


# INITIALSATION
#base, axis, centroids, accelerations = parse_data(logic.port)


#Measurement of postion and orientation of LightHouse basis
d = 2.610 #Distance between the 2 Basis

p1 = [0, d/2, 2]
p2 = [0, -d/2, 2]

#Rotation Matrices LH1 and LH2 on world Basis
RB_R =   [[cos(0.576), -sin(0.576) , 0],
          [sin(0.576), cos(0.576)  , 0],
          [0          , 0          , 1]]


RC_R =   [[cos(-0.698),-sin(-0.698), 0],
          [sin(-0.698),cos(-0.698) , 0],
          [0          ,0           , 1]]

#logic.R_RB = np.transpose(logic.RB_R)
#logic.R_RC = np.transpose(logic.RC_R)
"""
h10, v10, h20, v20 = np.zeros(4)
h11, v11, h21, v21 = np.zeros(4)
h12, v12, h22, v22 = np.zeros(4)
h13, v13, h23, v23 = np.zeros(4)
"""

def Pos(h1, v1, h2, v2):
    vecH1_loc = [sin(h1), cos(h1), 0]
    vecV1_loc = [sin(v1), 0, cos(v1)]

    vecH2_loc = [sin(h2), cos(h2), 0]
    vecV2_loc = [sin(v2), 0, cos(v2)]

    u = [vecH1_loc[0]+vecV1_loc[0], vecH1_loc[1]+vecV1_loc[1], vecH1_loc[2]+vecV1_loc[2]]
    v = [vecH2_loc[0]+vecV2_loc[0], vecH2_loc[1]+vecV2_loc[1], vecH2_loc[2]+vecV2_loc[2]]

    norm_u = sqrt(u[0]*u[0] + u[1]*u[1] + u[2]*u[2])
    norm_v = sqrt(v[0]*v[0] + v[1]*v[1] + v[2]*v[2])

    u_loc = np.array([u[0]/norm_u, u[1]/norm_u, u[2]/norm_u])
    v_loc = np.array([v[0]/norm_v, v[1]/norm_v, v[2]/norm_v])

    #Rotations matrix of lighthouses
    m1 = RB_R
    m2 = RC_R

    # STEP: transform line from relative coordinates to global lighthouse coordinate system (defined by matrix) (multiply vector by matrix)

    u = np.matmul(RB_R, u_loc)
    v = np.matmul(RC_R, v_loc)

    # Transform position

    p0 = p1
    q0 = p2
    #print(p0," & ",q0)

    # STEP: resolve the system of imperfect intersection
    w0 = np.array([p2[0] - p1[0], p2[1] - p1[1], 0])
    #w0 = p0 - q0
    #print(w0)
    a = np.dot(u, u)    #scalar product of u and w0
    b = np.dot(u, v)
    c = np.dot(v, v)

    d = np.dot(u, w0)
    e = np.dot(v, w0)

    #Resolution of the linear system
    #k = np.array([[uu, -uv], [uv, -vv]])
    #l = np.array([ABu, ABv])
    #lambda_mu = np.linalg.solve(k, l)

    denom = a*c - b*b

    if denom >= 1e-6:
        s = (e * b - c * d) / denom
        t = (a * e - b * d) / denom

        pS0 = p0[0] + s*u[0]
        pS1 = p0[1] + s*u[1]
        pS2 = p0[2] + s*u[2]

        qT0 = q0[0] + t*v[0]
        qT1 = q0[1] + t*v[1]
        qT2 = q0[2] + t*v[2]

        I0 = (pS0 + qT0) / 2
        I1 = (pS1 + qT1) / 2
        I2 = (pS2 + qT2) / 2
        I = [I0, I1, I2]

        # STEP: Convert output into standard coordinates
        """
        transforMatrix = [[ 0,  1, 0],
                         [ 0,  0, 1],
                         [ -1, 0, 0]]

        I = np.matmul(transforMatrix,I)
        """

        return I
