from sympy import *
from sympy import solve_poly_system
from math import pi
from math import *
from sys import stdin
import numpy as np
from reception import *
import time
import Position_IMU
import Position_LH

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
    

    #print(uTrue)
    #print(sUTrue)
    
    return [uTrue,sUTrue]





