import serial
import time
import numpy as np


"""
ser = serial.Serial("/dev/ttyUSB0", 115200)
#f = open("/home/temporary/test2.txt","w")

#Add prev value of t, acceleration, speed and position


#Period measurated on 1000 values But it will be better to measure T each time
T = 0.391246994304657

#initialization of t, acceleration, velocity and position
t_prev = time.time()

acc_or = ser.readline().split("\t")


v_prev = [T*float(acc_or[1]), T*float(acc_or[2]), T*float(acc_or[3])]
xyz_prev = [0,0,0] 

#print("v_x = ",v_prev[0], " & ", "x_= ", xyz_prev[0])

for i in range(100) :
    
    #Reading acceleration on serial port
    acc_or = ser.readline().split("\t")
    t = time.time()
    T = t-t_prev
    #accelaration on x, y, z axis
    a = [float(acc_or[1]), float(acc_or[2]), float(acc_or[3])]
    v = np.sum([v_prev, [T*a[0], T*a[1], T*a[2]]], axis=0)
    xyz = np.sum([xyz_prev, [T*v_prev[0], T*v_prev[1], T*v_prev[2]]], axis=0)

    #print(a, v, xyz)
    #print(str(acc_or[1]), " & ", str(T) )
    
    #Update previous values
    v_prev = v
    xyz_prev = xyz
    t_prev = t
"""
    
###############################################################################
def main():
    port = serial_init()
    while True:
        # orientation = array of 3 floats in degrees
        # acceleration = array of 3 floats in m/s^2
        print(parse_data(port))

###############################################################################
def serial_init():
    ser = serial.Serial("/dev/ttyUSB0", 115200)
    port = ser
 
    return port

###############################################################################
def parse_data(port):
    # Acceleration and orientation readen on the serial port
    acc_or = port.readline().split("\t")
    
    acceleration = [float(acc_or[1]), float(acc_or[2]), float(acc_or[3])]
    orientation = [float(acc_or[5]), float(acc_or[6]), float(acc_or[7][:-1])]

    return acceleration, orientation


port = serial_init()
acceleration, orientation = parse_data(port)

#print(acceleration)

for i in range(100) :
    port = serial_init()
    acceleration, orientation = parse_data(port)
    print(orientation)
