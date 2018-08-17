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
# Apply changement of coordinate M = [[0, 0, -1], [1, 0, 0], [0, 1, 0]]
# RB_R = Mt . [Bonsai given rotation matrix] . M
# t stand for transposed

"""
[[0.8633447, 0.02179115, -0.5041437],
[-0.07533064, -0.9823064, -0.1714628],
[-0.49896, 0.186009, -0.8464276]]
0.2055409 2.522384 -2.553286 1

[[-0.8772146, 0.03632253, 0.4787225],
[0.05409878, -0.9833049, 0.1737381],
[0.4770408, 0.1783039, 0.8606042]]
2.326427 2.428492 1.591011 1

### TESTS

[[0.7709669, 0.0003505305, 0.6368753],
[-0.00685263, -0.9999374, 0.008845782],
[0.6368385, -0.01118407, -0.7709163]]

[[0.9063203, -0.003801087, -0.4225747],
[0.001591456, -0.9999218, 0.01240765],
[-0.4225887, -0.01191781, -0.9062433]]

"""
# INITIALSATION
# Positionning LH
p1 = [0.2055409, 2.522384, -2.553286]
p2 = [2.326427, 2.428492, 1.591011]

# Rotation Matrices LH1 and LH2 on world Basis

m1 = [[0.8633447, 0.02179115, -0.5041437],
    [-0.07533064, -0.9823064, -0.1714628],
    [-0.49896, 0.186009, -0.8464276]]

m2 = [[-0.8772146, 0.03632253, 0.4787225],
    [0.05409878, -0.9833049, 0.1737381],
    [0.4770408, 0.1783039, 0.8606042]]

# Homogeneous coordinate matrices given by Vive sdk
h1 = [[0.8633447, 0.02179115, -0.5041437, 0],
      [-0.07533064, -0.9823064, -0.1714628, 0],
      [-0.49896, 0.186009, -0.8464276, 0],
      [0.2055409, 2.522384, -2.553286, 1]]

h2 = [[-0.8772146, 0.03632253, 0.4787225, 0],
      [0.05409878, -0.9833049, 0.1737381, 0],
      [0.4770408, 0.1783039, 0.8606042, 0],
      [2.326427, 2.428492, 1.591011, 1]]

# Changement base matrix from OpenGL to Blender

R_ob = [[ 1, 0, 0],
        [ 0, 0,-1],
        [ 0, 1, 0]]
"""
R_ob = [[ 0, 0,-1],
        [ 1, 0, 0],
        [ 0, 1, 0]]
"""
# Translation vectors after base Changement
p1b = np.dot(R_ob,p1)
#print(p1b)
p2b = np.dot(R_ob,p2)
#print(p2b)
# Rotation matrix in the base changement

R1 = np.matmul(np.linalg.inv(R_ob), np.matmul(m1, R_ob))
R2 = np.matmul(np.linalg.inv(R_ob), np.matmul(m2, R_ob))
"""
R1 = np.matmul(R_ob, np.matmul(m1, np.linalg.inv(R_ob)))
R2 = np.matmul(R_ob, np.matmul(m2, np.linalg.inv(R_ob)))
"""
def vect_uv(angle_scan):
    global h1, h2
    """
    vecH1_loc = np.array([-cos(angle_scan[0]), 0, sin(angle_scan[0]), 1])
    vecV1_loc = np.array([0, -cos(angle_scan[1]), sin(angle_scan[1]), 1])

    vecH2_loc = np.array([-cos(angle_scan[2]), 0, sin(angle_scan[2]), 1])
    vecV2_loc = np.array([0, -cos(angle_scan[3]), sin(angle_scan[3]), 1])
    """
    vecH1_loc = np.array([0, cos(angle_scan[0]), sin(angle_scan[0]), 1])
    vecV1_loc = np.array([cos(angle_scan[1]), 0, -sin(angle_scan[1]), 1])

    vecH2_loc = np.array([0, cos(angle_scan[2]), sin(angle_scan[2]), 1])
    vecV2_loc = np.array([cos(angle_scan[3]), 0, -sin(angle_scan[3]), 1])

    u = vecH1_loc + vecV1_loc
    v = vecH2_loc + vecV2_loc

    #print(angle_scan[2])
    #print(angle_scan[3])

    norm_u = np.norm(u)
    norm_v = np.norm(v)

    # u & v in homogeneous coordinates normalized
    u_loc = np.array([u[0]/norm_u, u[1]/norm_u, - u[2]/norm_u)
    v_loc = np.array([v[0]/norm_v, v[1]/norm_v, - v[2]/norm_v)

    # Transform line from relative coordinates to global lighthouse coordinate system (defined by matrix) (multiply vector by matrix)

    # For LH1
    # we need to transpose to convert from column-major to row-major
    h1 = np.array(h1).T # raw base transform
    p1 = np.array([0,0,0,1]) # (0,0,0) in homogeneous coordinates
    p1 = np.matmul(h1,p1) # p1 is position of base A
    u = np.matmul(h1,u_loc) # u vector after scanning of base A

    # now we fix all this to Blender space (swap Z with Y)
    swizzle = [0,2,1,3]
    p1 = p1[swizzle]
    u = u[swizzle]

    # For LH2
    h2 = np.array(h2).T # raw base transform
    p2 = np.array([0,0,0,1]) # (0,0,0) in homogeneous coordinates
    p2 = np.matmul(h2,p2) # p2 is position of base A
    v = np.matmul(h2,v_loc) # v vector after scanning of base A

    p2 = p2[swizzle]
    v = v[swizzle]

    return u[0:3], v[0:3]

def diode_pos(angle_scan):
    global R1, R2, p1b, p2b, m1, m2, p1, p2
    vecH1_loc = [0, cos(angle_scan[0]), sin(angle_scan[0])]
    vecV1_loc = [cos(angle_scan[1]), 0, -sin(angle_scan[1])]
    #vecH1_loc = [sin(angle_scan[0]), -cos(angle_scan[0]), 0]
    #vecV1_loc = [sin(angle_scan[1]), 0, -cos(angle_scan[1])]

    vecH2_loc = [0, cos(angle_scan[0]), sin(angle_scan[0])]
    vecV2_loc = [cos(angle_scan[1]), 0, -sin(angle_scan[1])]
    #vecH2_loc = [sin(angle_scan[2]), -cos(angle_scan[2]), 0]
    #vecV2_loc = [sin(angle_scan[3]), 0, -cos(angle_scan[3])]

    #print(angle_scan[3])
    """
    u = [vecH1_loc[0]+vecV1_loc[0], vecH1_loc[1]+vecV1_loc[1], vecH1_loc[2]+vecV1_loc[2]]
    v = [vecH2_loc[0]+vecV2_loc[0], vecH2_loc[1]+vecV2_loc[1], vecH2_loc[2]+vecV2_loc[2]]
    """
    u = np.cross(vecH1_loc, vecV1_loc)
    v = np.cross(vecH2_loc, vecV2_loc)
    #print(angle_scan[2])
    #print(angle_scan[3])
    """
    norm_u = sqrt(u[0]*u[0] + u[1]*u[1] + u[2]*u[2])
    norm_v = sqrt(v[0]*v[0] + v[1]*v[1] + v[2]*v[2])

    u_loc = np.array([u[0]/norm_u, u[1]/norm_u, u[2]/norm_u])
    v_loc = np.array([v[0]/norm_v, v[1]/norm_v, v[2]/norm_v])
    """
    # STEP: transform line from relative coordinates to global lighthouse coordinate system (defined by matrix) (multiply vector by matrix)

    u = np.matmul(m1, u)
    v = np.matmul(m2, v)

    # Transform position

    p0 = p1
    q0 = p2
    #print(p0," & ",q0)

    # STEP: resolve the system of imperfect intersection
    w0 = np.array([p2[0] - p1[0], p2[1] - p1[1], p2[2] - p1[2]])
    #w0 = p0 - q0
    #print(w0)
    a = np.dot(u, u)    #scalar product of u and w0
    b = np.dot(u, v)
    c = np.dot(v, v)

    d = np.dot(u, w0)
    e = np.dot(v, w0)

    # Resolution of the linear system
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

        I = np.dot(R1, I)
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

def get_vect_uv(rx):
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

    # Convert time of scanning into angle in radians
    for i in range(4):
           scanAngle[i][base*2 + axis] = centroids[i] * pi / T_scan

    # For Lighthouses
    #for i in range(4):
    # Scan for diode number 0
    return vect_uv(scanAngle[0])

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
