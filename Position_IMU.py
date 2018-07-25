#!/usr/bin/python
# Compute the position of a Lighthouse given
# sensor readings in a known configuration.

from sympy import *
from sympy import solve_poly_system
from math import pi
from math import *
from sys import stdin
import numpy as np
from reception_IMU import *
import time

port = serial_init()
base, axis, centroids, accelerations = parse_data(port)

velocityx = [0,0]

# Period of measurement
T = 1/120

def position():
    accelerationx[1] = accelerations[1]

    # First integration
    velocityx[1] = velocityx[0] + accelerationx[0] + ((accelerationx[1] - accelerationx[0])/2) * T
    # Second integration
    positionX[1] = positionX[0] + velocityx[0] + ((velocityx[1] - velocityx[0])/2) * T
    
    return positionX[1]
    
    accelerationx[0] = accelerationx[1]
    velocityx[0] = velocityx[1]
    positionX[0] = positionX[1]

    
    

while 1 :
    #print(parse_data(port))
    position()
    print(position())
    
    time.sleep(1/120)    