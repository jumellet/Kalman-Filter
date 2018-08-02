from sympy import *
import numpy as np
from math import*

def linearKFDDD(uAccelero, sUAccelero, uLightH, sULightHI, uTrue, sUTrueI):

    uEst = np.zeros(3)		# u estimated (=u predicted)
    sUEst = np.zeros((3,3))	# and the matrix u of covariances associated
    K = np.zeros((3,3))	# Kalman Gain
    tempMat = np.zeros((3,3))	# Temporary matrix used to easly compute K
    #sUTrue = np.zeros((3,3))	# Matrix of covariances of sUTrueI (I stand for input)
    #sULightH = np.zeros((3,3)) # matrix sULightH covariances
    I = np.eye(3)	        # Identity matrix
    """
    print("uAccelero : ", uAccelero)
    print("sUAccelero : ", sUAccelero)
    print("uLightH : ", uLightH)
    print("sULightHI : ", sULightHI)
    print("uTrue : ", uTrue)
    print("sUTrueI : ", sUTrueI)
    """
    # PREDICTION (which normally follows a mathematical law, but here the prediction is given by IMU)
    # Also there is the construction of covariances matrix of sUEst
    """
    for i in range(3):
        for j in range(3):
            if i=j :
                sUEst[i][j]=sqrt(sUAccelero[i]) * sqrt(sUAccelero[j])
            else :
                sUEst[i][j]=0
    """
    uEst = uAccelero
    sUEst = [[np.abs(sUAccelero[0]),0                    ,0                    ],          # Matrix of covariances of sUTrueI (I stand for input)
             [0                    ,np.abs(sUAccelero[1]),0                    ],
             [0                    ,0                    ,np.abs(sUAccelero[2])]]

    # Construction of sUTrue an sULightH, the covariances matrix from sUTrueI
    '''
    for i in range(3):
        sUTrue[i][i] = np.abs(sUTrueI[i])
        sULightH[i][i] = np.abs(sULightHI[i])
    '''
    sUTrue = [[np.abs(sUTrueI[0]),0                 ,0                 ],        # Matrix of covariances of sUTrueI (I stand for input)
              [0                 ,np.abs(sUTrueI[1]),0                 ],
              [0                 ,0                 ,np.abs(sUTrueI[2])]]

    sULightH = [[np.abs(sULightHI[0]),0                   ,0                   ],  # Matrix sULightH covariances
                [0                   ,np.abs(sULightHI[1]),0                   ],
                [0                   ,0                   ,np.abs(sULightHI[2])]]

    #UPDATE

    # Coefficient of the 2D matricial inverse, it's also Innovation (or residual) covariance
    #coefK = 1 / ((sUEst[0][0]+sULightH[0][0])*(sUEst[1][1]+sULightH[1][1]) - (sUEst[0][1]+sULightH[0][1])*(sUEst[1][0]+sULightH[1][0]));

    # Calculus of Kalman gain K
    for i in range(3):
        for j in range(3):
            K[i][j] = sUEst[i][j] + sULightH[i][j]
    K = sUEst * np.linalg.inv(K)
    #print(K)

    """
    for i in range(3):
        uTrue[i] += (K[i][0]*(uLightH[0] - uEst[0]) + K[i][1]*(uLightH[1] - uEst[1]) + K[i][2]*(uLightH[2] - uEst[2]) )
    """
    # vvvv HERE uTrue doesn't really follow KF Algorithm, but allow good results
    #uTrue[i] = uEst + K * (uEst - uLightH)
    for i in range(3):
        uTrue[i] = uEst[i] + (K[i][0]*(uLightH[0] - uEst[0]) + K[i][1]*(uLightH[1] - uEst[1]) + K[i][2]*(uLightH[2] - uEst[2]) )
    # Calculus of sUTrue

    #print(I)
    #print(K)
    for i in range(3):
        for j in range(3):
            tempMat[i][j] = I[i][j] - K[i][j]
            #print(tempMat[i][j])
    #sUTrue = np.dot(tempMat, SUTrue)
    #print(sUTrue)
    #print("tempMat : ",tempMat)
    sUTrue = tempMat.dot(sUTrue)
    # vvvv HERE SUTrue doesn't really follow KF Algorithm, but allow good results
    #sUTrue = (I - K) * sUEst


    #print(uTrue)
    #print(sUTrue)

    return [uTrue, [sUTrue[0][0], sUTrue[1][1], sUTrue[2][2]]]
