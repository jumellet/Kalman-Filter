#!/usr/bin/python
# Compute the position of a Lighthouse given
# sensor readings in a known configuration.

#from math import *
import serial
from math import *
import numpy as np
from reception import *

FILTER = 2
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

        return I

##########################################################
# PROCESSING IMU
##########################################################
# Period of measurement of the IMU
T = 1/120.
# Standard deviation of IMU (m/s^2) considered the same on 3 axis
S_ACC = 0.0423639918

def IMU_pos(prevPos, prevVel, accel):
    # Initilisation of arrays. Two vectors the old "0" and  the new "1" values
    velocity = np.zeros((2,3))
    position = np.zeros((2,3))

    for i in range(3):
        velocity[0][i] = prevVel[i]
        position[0][i] = prevPos[i]

    # Here begin the function
    # First integration
    for i in range(3):
        velocity[1][i] = velocity[0][i] + accel[i] * T

    # Second integration
    for i in range(3):
        position[1][i] = position[0][i] + velocity[0][i] * T

    return [position[1], velocity[1], accel]

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

raw_diode = np.zeros((4,4,3))
# Circular buffer index
cbi = 0

time = 4
I_Accelero = [0, 0, 0]
# Previous velocity
velocity = [0,0,0]
# Standard deviation of IMU
s_I_Accelero = [0, 0, 0]
s_velocity = [0, 0, 0]
s_accelerations = [S_ACC, S_ACC, S_ACC]

wasIMUInit = False

def get_position(rx):
    global I_Accelero, velocity, s_I_Accelero, s_velocity, s_accelerations
    global FILTER, S_ACC, raw_diode, time, wasIMUInit, cbi, factor

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
    # Factor of smoothness for low pass filter
    factor = 0.2

    # Convert time of scanning into angle in radians
    for i in range(4):
           scanAngle[i][base*2 + axis] = centroids[i] * pi / T_scan

    # For Lighthouses
    for i in range(4):
        I_diode[i] = diode_pos(scanAngle[i])

    # Low pass filter
    if FILTER == 1 :
        for d in range(4):
            for xyz in range(3):
                I_diode[d][xyz] = (1 - factor) * raw_diode[0][d][xyz] + factor * I_diode[d][xyz]
                raw_diode[0][d][xyz] = I_diode[d][xyz]


    # Low pass filter using 4 last optical data
    elif FILTER == 2 :
        for d in range(4):
            for xyz in range(3):
                raw_diode[cbi][d][xyz] = I_diode[d][xyz]
                average = 0
                for t in range(4):
                    average += raw_diode[t][d][xyz]
                I_diode[d][xyz] = average / 4

    # Circular buffer index
    cbi = (cbi+1) % 4

    I_LH = [(I_diode[0][0] + I_diode[3][0]) / 2, (I_diode[0][1] +  I_diode[3][1]) / 2, (I_diode[0][2] +  I_diode[3][2]) / 2]
    # Position where the IMU will be at calibration
    averagePos = I_LH

    # For IMU

    # Reset position of IMU at (1/120 * 4)s
    # We consider variance on measurement, the same on 3 axis
    if time >= 4 :
        # off_set allows to calibrate position of the IMU
        off_set = averagePos
        I_Accelero = [0, 0, 0]
        time = 0
        for i in range(3):
            I_Accelero[i] = off_set[i]
            velocity[i] = 0
            s_I_Accelero[i] = 0
            s_velocity[i] = 0
            s_accelerations[i] = S_ACC

    # Update data of the accelerometer
    I_Accelero, velocity, accel = IMU_pos(I_Accelero, velocity, accelerations)
    # Update standard deviation of accelerometer
    s_I_Accelero, s_velocity, s_accelerations = IMU_pos(s_I_Accelero, s_velocity, s_accelerations)

    return I_diode, [I_Accelero, velocity, accel], [s_I_Accelero, s_velocity, s_accelerations]
