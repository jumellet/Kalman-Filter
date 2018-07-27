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


def linearKFDDD(uAccelero, sUAccelero, uLightH, sULightHI, uTrue, sUTrueI):

    uEst = np.zeros(3)		# u estimated (=u predicted)
    sUEst = np.zeros((3,3))	# and the matrix u of covariances associated
    K = = np.zeros((3,3))	# Kalman Gain
    tempMat = np.zeros((3,3))	# Temporary matrix used to easly compute K
    sUTrue = = np.zeros((3,3))	# Matrix of covariances of sUTrueI (I stand for input)
    sULightH = np.zeros((3,3))  # matrix sULightH covariances
    I = np.eye(3)	        # Identity matrix


    # PREDICTION (which normally follows a mathematical law, but here the prediction is given by IMU)
    # Also there is the construction of covariances matrix of sUEst
    """
    for i in range(3):
        uEst[i] = uAccelero[i]
        for j in range(3):
            if i=j :
                sUEst[i][j]=sqrt(sUAccelero[i]) * sqrt(sUAccelero[j])
            else :
                sUEst[i][j]=0
    """            
    uEst = uAccelero
    sUEst = sUAccelero

    # Construction of sUTrue an sULightH, the covariances matrix from sUTrueI
    for i in range(3):
        for j in range(3):
            #sULightH[i][j] = sqrt(sULightHI[i]) * sqrt(sULightHI[j]);
            if i=j :
                sUTrue[i][j] = sqrt(sUTrueI[i]) * sqrt(sUTrueI[j])
                sULightH[i][j] = sqrt(sULightHI[i]) * sqrt(sULightHI[j])
            else :
                sUTrue[i][j] = 0
                sULightH[i][j] = 0

    #UPDATE

    # Coefficient of the 2D matricial inverse, it's also Innovation (or residual) covariance
    #coefK = 1 / ((sUEst[0][0]+sULightH[0][0])*(sUEst[1][1]+sULightH[1][1]) - (sUEst[0][1]+sULightH[0][1])*(sUEst[1][0]+sULightH[1][0]));

    # Calculus of Kalman gain K
    
    K = sUEst * np.linalg.inv(sUEst + sULightH)
    
    """
    for i in range(3):
        for j in range(3):
            #Multiply it by sUEst and uTrue.transpose

            K[i][j] = coefK * (sUEst[i][0]*tempMat[0][j] + sUEst[i][1]*tempMat[1][j]);

    """
    
    
    # Calculus of uTrue
    """
    for (i=0; i<2; i++){
        //uTrue[i] = uEst[i] + (K[i][0]*(uLightH[0]-uEst[0]) + K[i][1]*(uLightH[1]-uEst[1]))   ;
            //uTrue[i] += K[i][i] * (uLightH[i] - uEst[i]);
            //vvvv HERE uTrue doesn't realy follow KF Algorithm, but allow good results
            uTrue[i] = uEst[i] + K[i][i] * (uEst[i] - uLightH[i]);
    """
    uTrue += K * (uLightH - uEst) 
    # vvvv HERE uTrue doesn't really follow KF Algorithm, but allow good results
    #uTrue[i] = uEst + K * (uEst - uLightH)
    
    # Calculus of sUTrue
    """
    for i in range(2):
        for (j=0; j<2; j++){
                //sUTrue[i][j] = sUEst[i][j] - (K[i][0]*sUEst[0][j] + K[i][1]*sUEst[1][j]) ;
                    tempMat[i][j] = (I[i][j]-K[i][j]);
                    sUTrue[i][j] = tempMat[i][0]*sUEst[0][j] + tempMat[i][1]*sUEst[1][j];
                    //cout << "sUTrue"<<i<<","<<j<<" = "<<sUTrue[i][j]<< endl;
    """
    sUTrue = (I - K) * sUTrue
    # vvvv HERE SUTrue doesn't really follow KF Algorithm, but allow good results
    #sUTrue = (I - K) * sUEst
    

    print(uTrue)
    print(sUTrue)
    
    return [uTrue,sUTrue]


#===========================================================
#  LINEAR KALMAN FILTER 2D
#===========================================================


# INITIALISATION

# For KF
result = [[0, 0], [0.0001, 0.0001]]    # Matrix to return uTrue & sUTrue (respectively u vector calculated and his covariance)
# uTrue is initialized to x = y = 0   &   sUTrue is initialized sX = sY = 1mm

# For IMU
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
while 1 :
    # For IMU
    
    #print(parse_data(port))
    base, axis, centroids, accelerations = parse_data(port)
    
    #print(accelerations)
    
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
    
    uAccelero = [positionX[0], positionY[0], positionZ[0]]
    ########################################################################
    # For LH
    uLightH = 
    
    #########################################################################
    
    # For KF
    result = linearKFDD(uAccelero, sUAccelero, uLightH, sULightH, result[0], result[1]);
    print(result)
 """   
      
# Measurement on KF Algorithm



