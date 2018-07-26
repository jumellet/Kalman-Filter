#!/usr/bin/python
# Compute the position of a Lighthouse given
# sensor readings in a known configuration.

from sympy import *
from sympy import solve_poly_system
from math import pi
from math import *
from sys import stdin
import numpy as np
from reception import *
import time

port = serial_init()
base, axis, centroids, accelerations = parse_data(port)
print(parse_data(port))

velocityx = np.zeros(2)

# Period of measurement
T = 1/120

"""
def position(accel):
    accelerationx = np.zeros(2)
    velocityx = np.zeros(2)
    positionX = np.zeros(2)
    
    accelerationx[1] = accel[1]

    # First integration
    velocityx[1] = velocityx[0] + accelerationx[0] * 1/120
    # Second integration
    positionX[1] = positionX[0] + velocityx[0] * 1/120
    
    return positionX[1]
    
    accelerationx[0] = accelerationx[1]
    velocityx[0] = velocityx[1]
    positionX[0] = positionX[1]
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

while 1 :
    #print(parse_data(port))
    base, axis, centroids, accelerations = parse_data(port)
    
    #print(accelerations)
    
    accelerationX[1] = + accelerations[1]
    accelerationY[1] = - accelerations[0]
    accelerationZ[1] = - accelerations[2]

    # First integration
    velocityX[1] = velocityX[0] + accelerationX[0] * 1/120
    velocityY[1] = velocityY[0] + accelerationY[0] * 1/120
    velocityZ[1] = velocityZ[0] + accelerationZ[0] * 1/120
    # Second integration
    positionX[1] = positionX[0] + velocityX[0] * 1/120
    positionY[1] = positionY[0] + velocityY[0] * 1/120
    positionZ[1] = positionZ[0] + velocityZ[0] * 1/120
    
    #print(positionX[1])
    
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
    
    
    #position(accelerations)
    #print(position(accelerations))
    
    #print("time = ",time.clock()," s" )
    print(time.clock())