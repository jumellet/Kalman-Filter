//============================================================================
// Name        : Linear_Kalman_Filter.cpp
// Author      : Julien
// Version     :
// Copyright   : Your copyright notice
// Description : KF linear  1D
//============================================================================

//#include <graphics.h>

#include <math.h>
//#include <conio.h>
#include <stdlib.h>
#include <iostream>
using namespace std;

//===========================================================
//FUNCTION OF 1D LINEAR KALMAN FILTER
//===========================================================
float *linearKFD(float xAccelero, float sXAccelero, float xLightH, float sXLightH, float xTrue, float sXTrue)
{
	float xEst, sXEst;	//x estimated and covariances associated
	float K;	//Kalman Gain

	//Predict (which normally follows a mathematical law, but here the prediction is given by IMU)
	xEst=xAccelero;
	sXEst=sXAccelero;

	//Update
	//K=xEst/(xEst+sXTrue);
	//K=sXEst/(sXEst+sXTrue);
	K=sXEst/(sXEst+sXLightH);
	//cout << "K = " <<K<< endl;
	xTrue=(xAccelero*sXLightH + xLightH*sXAccelero) / (sXAccelero + sXLightH);
				//weighted average for a 1D filter
	sXTrue=(1-K)*sXEst;

	float *normXTrue = new float[2];
	normXTrue[0]=xTrue;
	normXTrue[1]=sXTrue;

	return normXTrue;
}

//===========================================================
//FUNCTION OF 2D LINEAR KALMAN FILTER
//===========================================================
//Vectors have to be input


float **linearKFDD(float *uAccelero, float *sUAccelero, float *uLightH, float *sULightHI, float *uTrue, float *sUTrueI)
{
	int i,j ;	//allows the increment on for loops
	float *uEst = new float[2];		//u estimated
	float **sUEst = new float*[2];	//and the matrix u of covariances associated
	sUEst[0] = new float[2];
	sUEst[1] = new float[2];
	float **K = new float*[2];	//Kalman Gain
	K[0] = new float[2];
	K[1] = new float[2];
	float **tempMat = new float*[2];	//Temporary matrix used to easly compute K
	tempMat[0] = new float[2];
	tempMat[1] = new float[2];
	float **sUTrue = new float*[2];	//Matrix of covariances of sUTrueI (I stand for input)
	sUTrue[0] = new float[2];
	sUTrue[1] = new float[2];
	float **sULightH = new float*[2];	//matrix sULightH covariances
	sULightH[0] = new float[2];
	sULightH[1] = new float[2];

	//PREDICT (which normally follows a mathematical law, but here the prediction is given by IMU)
		//Also there is the construction of covariances matrix of sUEst
	for (i=0; i<2; i++){
		uEst[i]=uAccelero[i];
		for (j=0; j<2; j++){
			sUEst[i][j]=sqrt(sUAccelero[i]) * sqrt(sUAccelero[j]);
			//cout << "uEst"<<i<<" = "<< uEst[i] << endl;
			//cout << "sUEst"<<i<<","<<j<<" = "<< sUEst[i][j] << endl;
		}
	}
		//Construction of sUTrue an sULightH, the covariances matrix from sUTrueI
	for (i=0; i<2; i++){
		for (j=0; j<2; j++){
			sULightH[i][j] = sqrt(sULightHI[i]) * sqrt(sULightHI[j]);
			if (i==j){
				sUTrue[i][j] = sqrt(sUTrueI[i]) * sqrt(sUTrueI[j]);
				//sULightH[i][j] = sqrt(sULightHI[i]) * sqrt(sULightHI[j]);
			}
			else {
				sUTrue[i][j] = 0;
				//sULightH[i][j] = 0;
			}
			cout << "sUTrue"<<i<<","<<j<<" = "<< sUTrue[i][j]<< endl;
			cout << "sULightH"<<i<<","<<j<<" = "<< sULightH[i][j]<< endl;
		}
	}
		//^^^^Test with another covariance (not covariance put diag) construction^^^^

	//UPDATE

		//coefficient of the 2D matricial inverse, it's also Innovation (or residual) covariance
	float coefK = 1 / ((sUEst[0][0]+sUTrue[0][0])*(sUEst[1][1]+sUTrue[1][1]) - (sUEst[0][1]+sUTrue[0][1])*(sUEst[1][0]+sUTrue[1][0]));
	tempMat[0][0]=sUEst[1][1]+sUTrue[1][1];
	tempMat[0][1]=-sUEst[0][1]-sUTrue[0][1];
	tempMat[1][0]=-sUEst[1][0]-sUTrue[1][0];
	tempMat[1][1]=sUEst[0][0]+sUTrue[0][0];
	/*float coefK = 1 / ((sUEst[0][0]+sULightH[0][0])*(sUEst[1][1]+sULightH[1][1]) - (sUEst[0][1]+sULightH[0][1])*(sUEst[1][0]+sULightH[1][0]));
	tempMat[0][0]=sUEst[1][1]+sULightH[1][1];
	tempMat[0][1]=-sUEst[0][1]-sULightH[0][1];
	tempMat[1][0]=-sUEst[1][0]-sULightH[1][0];
	tempMat[1][1]=sUEst[0][0]+sULightH[0][0];*/
		//Calculus of Kalman gain K
	for (i=0; i<2; i++){
		for (j=0; j<2; j++){
			K[i][j] = coefK * (sUEst[i][0]*tempMat[0][j] + sUEst[i][1]*tempMat[1][j]);
			cout << "K"<<i<<","<<j<<" = "<<K[i][j]<< endl;
		}
	}
		//Calculus of uTrue
	for (i=0; i<2; i++){
		uTrue[i] = uEst[i] + (K[i][0]*(uLightH[0]-uEst[0]) + K[i][1]*(uLightH[1]-uEst[1]))   ;
		cout << "uTrue"<<i<<" = "<<uTrue[i]<< endl;
	}
	cout <<"=====================================" << endl;
		//Calculus of sUTrue
	for (i=0; i<2; i++){
		for (j=0; j<2; j++){
			tempMat[i][j] =
			sUTrue[i][j] = sUEst[i][j] - (K[i][0]*sUEst[0][j] + K[i][1]*sUEst[1][j]) ;
			cout << "sUTrue"<<i<<","<<j<<" = "<<sUTrue[i][j]<< endl;
		}
	}
	cout <<"=====================================" << endl;
/*
	K=xEst/(xEst+sXTrue);
	xTrue=(xAccelero*sXLightH + xLightH*sXAccelero) / (sXAccelero + sXLightH);
				//weighted average for a 1D filter
	sXTrue=(1-K)*sXEst;
*/
	float **normUTrue = new float*[2];
	normUTrue[0] = new float[2];
	normUTrue[0][0] = uTrue[0];
	normUTrue[0][1] = uTrue[1];
	normUTrue[1] = new float[2];
	normUTrue[1][0] = sUTrue[0][0];
	normUTrue[1][1] = sUTrue[1][1];

	cout << "normUTrue [0] : " << normUTrue[0][0] <<", "<< normUTrue[0][1] << endl;
	cout << "normUTrue [1] : " << normUTrue[1][0] <<", "<< normUTrue[1][1] << endl;

	delete[] K[0];
	delete[] K[1];
	delete[] K;

	delete[] tempMat[0];
	delete[] tempMat[1];
	delete[] tempMat;

	delete[] uEst;

	delete[] sUEst[0];
	delete[] sUEst[1];
	delete[] sUEst;

	delete[] sUTrue[0];
	delete[] sUTrue[1];
	delete[] sUTrue;

	delete[] sULightH[0];
	delete[] sULightH[1];
	delete[] sULightH;

	return normUTrue;
}

/////////////////////////////////////
//MAIN PROGRAM
////////////////////////////////////

int main() {

	/*
///////////////////////////////////////////////////
//NOISY IMU Simulation
	int noise, ka=0, D=9; //Noise and Constant ka to simulate Derivation D of IMU
	for (int i=0; i<200; i++){
		//IMU as a linear variation
		//noise=rand() % 100+1; //noise in the range 0 to 100
		//noise-=50;
		//ka+=1;
		//D=ka+noise;
		//cout <<  D <<endl;

		//IMU as an non linear variation
		noise=rand() % 20+1; //noise in the range 0 to 100
		noise-=9;
		D+=noise;
		//cout <<  D <<endl;
	} */

/////////////////////////////////////////////////////
	/*//IMU SIMULATION
	xAcceleroTab[0]=110.0;	//Mean of the accelerosXAcceleroTab[3],
	sXAcceleroTab[0]=20.0;	//Covariance of the accelero

	//Simulation of Accelero derivation : x derive & variance associated increase

	//cout<<xAcceleroTab[0]<<", "<<sXAcceleroTab[0]<<endl;
	for (int i=1; i<=3; i++){
		xAcceleroTab[i]=xAcceleroTab[0]+i*10;
		sXAcceleroTab[i]=sXAcceleroTab[0]+i*5;
		//cout<<xAcceleroTab[i]<<", "<<sXAcceleroTab[i]<<endl;
	}*/
	//, xAcceleroTab[3];

///////////////////////////////////////////////////////////
/*  //NOISY LH Measurement Simulation
	float G, s=10.0, m=70.0; //Value of the gaussian distribution & his variance & mean
	for (int i=0; i<200; i++){
		//noise=rand() % 100+1; //noise in the range 0 to 100
		//noise-=50;			//make the noise average equal to zero
		//noise/=5;
		//Equation of a gaussian
		G=1000*1/(pow((2.0*3.1415),0.5)*sqrt(s))*exp(-pow((i-m),2)/(2.0*pow(sqrt(s),2.0)));
		//cout <<(G+noise)<<endl;
		//cout <<G<<endl;
	} */

///////////////////////////////////////////////////
	/* AFFICHER GRAPH
	initwindow(800,600);
	int x,y;
	line(0,300,getmaxx(),300);
	line(400,0,400,getmaxy());
	float pi=3.1415;
	for (int i =-360; i<=360;i++){
		x=400+i;
		y=300sin(i*pi/100)*25;
		putpixel(x,y,WHITE);
	}
	getch();
	closegraph();
	*/



//===========================================================
//LINEAR KALMAN FILTER 1D
//===========================================================
/*
	//1D circular movement around point 0 radius 4
	float circ[21]={-4.0,-3.98,-3.92,-3.82,-3.67,-3.46,-3.2,-2.86,-2.4,-1.74,0,1.74,2.4,2.86,3.2,3.46,3.67,3.82,3.92,3.98,4.0};

	//INITIALISATION
	float *result = new float[2]; //Array to return xTrue & sXTrue (respectively x calculated and his covariance)
	result[0] = 0.0;	//xTrue is here fixed to 70.0
	result[1] = 20.0;	//The covariance sXTrue is initialized to 20.0

	//IMU SIMULATION
	//Accelero derivation : x derive & variance associated increase
	float xAcceleroTab[4], sXAcceleroTab[4];
	xAcceleroTab[0]=0.0;	//Mean of the accelero
	sXAcceleroTab[0]=20.0;	//Covariance of the accelero

	//Light House Simulation
	//Position stay constant according to the last measurement, but covariance increase
	float xLightHTab[4], sXLightHTab[4];
	xLightHTab[0] = 0.0;
	sXLightHTab[0] = 20.0;

	//Measurement loops using KF
	for (int i=0; i<=4; i++){
		for (int j=0; j<=3; j++){
			int t = j+i*4; //Instants t

			//IMU Simulation
			//xAcceleroTab[j]=t*1.2;	//Linear movement
			xAcceleroTab[j]=circ[t]*1.2;	//Circular movement
			sXAcceleroTab[j]=sXAcceleroTab[0]+j*5;
			//cout<<xAcceleroTab[i]<<", "<<sXAcceleroTab[i]<<endl;

			//LightHouse Simulation
			//xLightHTab[j]=i*4;	//Linear movement
			xLightHTab[j]=circ[i*4];	//Circular movement
			sXLightHTab[j]=sXLightHTab[0]+3*j;

			result = linearKFD(xAcceleroTab[j], sXAcceleroTab[j],  xLightHTab[j], sXLightHTab[j], result[0], result[1]);

			//cout << "x"<<i<<"  "<< result[0] << "\nsx"<<i<<" " << result[1] <<"\n======"<< endl;
			//cout << circ[t] <<" , "<< result[0] << " , "<< result[1] << endl;
			cout << result[1] << endl;

		}
	}
*/
//===========================================================
//LINEAR KALMAN FILTER 2D
//===========================================================

	//INITIALISATION
	float **result = new float*[2]; //Matrix to return uTrue & sUTrue (respectively u vector calculated and his covariance)
	result[0] = new float[2];
	result[0][0] = 0.0;	//uTrue is initialized to x=y=0
	result[0][1] = 0.0;
	result[1] = new float[2];
	result[1][0] = 20.0;	//sUTrue is initialized sX=sY=20
	result[1][1] = 20.0;

	//IMU Simulation
	float *uAccelero = new float[2];
	uAccelero[0] = 0;
	uAccelero[1] = 0;
	float *sUAccelero = new float[2];
	sUAccelero[0] = 40;
	sUAccelero[1] = 40;

	//Light House Simulation
	float *uLightH = new float[2];
	uLightH[0] = 10;
	uLightH[1] = 10;
	float *sULightH = new float[2];
	sULightH[0] = 20;
	sULightH[1] = 20;

	//Measurement on KF Algorithm
	result = linearKFDD(uAccelero, sUAccelero, uLightH, sULightH, result[0], result[1]);

	cout << "x  & y  : "<< result[0][0] << " & "<< result[0][1] << endl;
	cout << "sX & sY : "<< result[1][0] << " & "<< result[1][1] << endl;

	delete[] uAccelero;
	delete[] sUAccelero;

	delete[] uLightH;
	delete[] sULightH;

	delete[] result[0];
	delete[] result[1];

	delete[] result;

	return 0;
}
