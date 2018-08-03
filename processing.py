#!/usr/bin/python
# Compute the position of a Lighthouse given
# sensor readings in a known configuration.

#from math import *
#import serial
from math import *
import numpy as np
#from reception import *

#########################################################
# PROCESSING LightHouse
#########################################################
# INITIALSATION
#base, axis, centroids, accelerations = parse_data(logic.port)
"""
wasReceptionInit = False
if not wasReceptionInit :
    # Initialize serial port and prepare data buffer
    logic.port = serial_init()
    wasReceptionInit = True
"""
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

def diode_pos(h1, v1, h2, v2):
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

        return I




##########################################################
# PROCESSING IMU
##########################################################
# Period of measurement
T = 1/120.

# Initialization of IMU
"""
accelerationX = np.zeros(2)
accelerationY = np.zeros(2)
accelerationZ = np.zeros(2)

velocityX = np.zeros(2)
velocityY = np.zeros(2)
velocityZ = np.zeros(2)


positionX = np.zeros(2)
positionY = np.zeros(2)
positionZ = np.zeros(2)
"""



"""
if (Position_LH.base == 0 and Position_LH.axis == 0) :
    h10 = Position_LH.centroids[0] * pi / 8333
    h13 = Position_LH.centroids[3] * pi / 8333

if (Position_LH.base == 0 and Position_LH.axis == 1) :
    v10 = Position_LH.centroids[0] * pi / 8333
    v13 = Position_LH.centroids[3] * pi / 8333

if (Position_LH.base == 1 and Position_LH.axis == 0) :
    h20 = Position_LH.centroids[0] * pi / 8333
    h23 = Position_LH.centroids[3] * pi / 8333

if (Position_LH.base == 1 and Position_LH.axis == 1) :
    v20 = Position_LH.centroids[0] * pi / 8333
    v23 = Position_LH.centroids[3] * pi / 8333
"""

#I_init = Position_LH.Pos( (h10+h13) / 2, (v10+v13) / 2, (h20+h23) / 2, (v20+v23) / 2)
#positionX, positionY, positionZ = I_init

def position(accel, prevVel, prevPos):
    # Initilisation of arrays : Two vectors old "0" and new "1" values
    acceleration = np.zeros((2,3))

    velocityX = np.zeros(2)
    velocityY = np.zeros(2)
    velocityZ = np.zeros(2)

    positionX = np.zeros(2)
    positionY = np.zeros(2)
    positionZ = np.zeros(2)

    acceleration[0] = [ accel[0], accel[1], accel[2]]

    velocityX[0] = prevVel[0]
    velocityY[0] = prevVel[1]
    velocityZ[0] = prevVel[2]

    positionX[0] = prevPos[0]
    positionY[0] = prevPos[1]
    positionZ[0] = prevPos[2]

    # Here begin the function
    # First integration
    velocityX[1] = velocityX[0] + acceleration[0][0] * T
    velocityY[1] = velocityY[0] + acceleration[0][1] * T
    velocityZ[1] = velocityZ[0] + acceleration[0][2] * T
    # Second integration
    positionX[1] = positionX[0] + velocityX[0] * T
    positionY[1] = positionY[0] + velocityY[0] * T
    positionZ[1] = positionZ[0] + velocityZ[0] * T

    return [[positionX[1],positionY[1],positionZ[1]], [velocityX[1], velocityY[1], velocityZ[1]]]

#while 1 :
    #print(parse_data(port))
    #base, axis, centroids, accelerations = parse_data(port)

    #print(accelerations)

    #position(accelerations)
    #print(position(accelerations))

    """
    accelerationX[1] = + accelerations[1]
    accelerationY[1] = - accelerations[0]
    accelerationZ[1] = - accelerations[2] + 9.81

    # First integration
    velocityX[1] = velocityX[0] + accelerationX[0] * 1/120
    velocityY[1] = velocityY[0] + accelerationY[0] * 1/120
    velocityZ[1] = velocityZ[0] + accelerationZ[0] * 1/120
    # Second integration
    positionX[1] = positionX[0] + velocityX[0] * 1/120
    positionY[1] = positionY[0] + velocityY[0] * 1/120
    positionZ[1] = positionZ[0] + velocityZ[0] * 1/120

    print(positionX[1], "  ,  ", positionY[1], "  ,  ", positionZ[1])

    # Update previous values
    accelerationX[0] = accelerationX[1]
    accelerationY[0] = accelerationY[1]
    accelerationZ[0] = accelerationZ[1]
    velocityX[0] = velocityX[1]
    velocityY[0] = velocityY[1]
    velocityZ[0] = velocityZ[1]
    positionX[0] = positionX[1]
    positionY[0] = positionY[1]
    positionZ[0] = positionZ[1]
    """

    #if time.clock() >= 5 :
        #break

#########################################################
# MAIN
#########################################################
