#!/usr/bin/python
# Compute the position of a Lighthouse given
# sensor readings in a known configuration.

#from math import *
import serial
from math import *
import numpy as np
from reception import *

#########################################################
# PROCESSING LightHouse
#########################################################
# INITIALSATION
#port = serial_init()
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

def diode_pos(angle_scan):
    vecH1_loc = [sin(angle_scan[0]), cos(angle_scan[0]), 0]
    vecV1_loc = [sin(angle_scan[1]), 0, cos(angle_scan[1])]

    vecH2_loc = [sin(angle_scan[2]), cos(angle_scan[2]), 0]
    vecV2_loc = [sin(angle_scan[3]), 0, cos(angle_scan[3])]

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
    pS = np.zeros(3)
    qT = np.zeros(3)
    I = np.zeros(3)

    if denom >= 1e-6:
        s = (e * b - c * d) / denom
        t = (a * e - b * d) / denom

        for i in range(3):
            pS[i] = p0[i] + s*u[i]
            qT[i] = q0[i] + t*v[i]
            I[i] = (pS[i] + qT[i]) / 2

        """
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
        """
        return I

##########################################################
# PROCESSING IMU
##########################################################
# Period of measurement of the IMU
T = 1/120.

def IMU_pos(prevPos, prevVel, accel):
    # Initilisation of arrays. Two vectors the old "0" and  the new "1" values
    acceleration = np.zeros((2,3))
    velocity = np.zeros((2,3))
    position = np.zeros((2,3))

    acceleration[0] = [accel[0], accel[1], accel[2]]

    for i in range(3):
        velocity[0][i] = prevVel[i]
        position[0][i] = prevPos[i]

    # Here begin the function
    # First integration
    for i in range(3):
        velocity[1][i] = velocity[0][i] + acceleration[0][i] * T
    # Second integration
    for i in range(3):
        position[1][i] = position[0][i] + velocity[0][i] * T

    return [position[1], velocity[1], acceleration[0]]

#########################################################
# MAIN
#########################################################
def init_position():
    # Initialize serial port and prepare data buffer
    return Reception()

# Initialize Angle calculated from timings of LH
scanAngle = [[0, 0, 0, 0],
             [0, 0, 0, 0],
             [0, 0, 0, 0],
             [0, 0, 0, 0]]
# Initailize positions of diodes 0, 1, 2  and 3
I_diode = [[0, 0, 0],
           [0, 0, 0],
           [0, 0, 0],
           [0, 0, 0]]
prev_I_diode = [[0, 0, 0],
           [0, 0, 0],
           [0, 0, 0],
           [0, 0, 0]]

time = 4
I_Accelero = [0, 0, 0]
# Previous velocity
velocity = [0,0,0]
wasIMUInit = False

def get_position(rx):
    global time, wasIMUInit, velocity, I_Accelero, prev_I_diode

    if not wasIMUInit :
        velocity = [0, 0, 0]
        wasIMUInit = True

    # Refresh data
    # base = 0 or 1 (B or C)
    # axis = 0 or 1 (horizontal or vertical)
    # centroids = array of 4 floats in microseconds
    # accelerations = array of 3 floats in G (AKA m/s^2)
    base, axis, centroids, accelerations = rx.parse_data()

    # Periode of one scan in micro seconds
    T_scan = 8333
    time += 1

    # Convert time of scanning into angle in radians
    for i in range(4):
           scanAngle[i][base*2 + axis] = centroids[i] * pi / T_scan

    # For Lighthouses
    for i in range(4):
        I_diode[i] = diode_pos(scanAngle[i])

    I_LH = [(I_diode[0][0] + I_diode[3][0]) / 2, (I_diode[0][1] +  I_diode[3][1]) / 2, (I_diode[0][2] +  I_diode[3][2]) / 2]
    averagePos = I_LH

    # For IMU

    # Reset position of IMU at (1/120 * 4)ms
    if time >= 4 :
        off_set = averagePos
        I_Accelero = [0, 0, 0]
        time = 0
        for i in range(3):
            I_Accelero[i] = off_set[i]
            velocity[i] = 0

    # Update data of the accelerometer
    I_Accelero, velocity, accel = IMU_pos(I_Accelero, velocity, accelerations)

    #Low pass filter on optical data
    factor = 0.2
    for j in range(4):
        for i in range(3):

            I_diode[j][i] = (1 - factor) * prev_I_diode[j][i] + factor * I_diode[j][i]
            prev_I_diode[j][i] = I_diode[j][i]

    return I_diode, [I_Accelero, velocity, accel]
