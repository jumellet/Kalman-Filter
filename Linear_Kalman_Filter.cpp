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
//FUNCTION OF LINEAR KALMAN FILTER
//===========================================================
float *linearKFD(float xAccelero, float sXAccelero, float xLightH, float sXLightH, float sTrue, float sXTrue)
{
	float xEst, xTrue;	//Tab(~Memory of previous measurement) & Output
	float sXEst;	//Covariances associated
	float K;	//Kalman Gain

	//Predict (which normally follows a mathematical law, but here the prediction is given by IMU)
	xEst=xAccelero;
	sXEst=sXAccelero;

	//Update
	K=xEst/(xEst+sXTrue);
	xTrue=(xAccelero*sXLightH + xLightH*sXAccelero) / (sXAccelero + sXLightH);
				//weighted average for a 1D filter
	sXTrue=(1-K)*sXEst;

	float *normXTrue = new float[2];
	normXTrue[0]=xTrue;
	normXTrue[1]=sXTrue;

	return normXTrue;
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

	//1D circular movement around point 0 radius 4
	float circ[21]={-4.0,-3.98,-3.92,-3.82,-3.67,-3.46,-3.2,-2.86,-2.4,-1.74,0,1.74,2.4,2.86,3.2,3.46,3.67,3.82,3.92,3.98,4.0};

//===========================================================
//LINEAR KALMAN FILTER
//===========================================================

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
			cout << circ[t] <<" , "<< result[0] << " , "<< result[1] << endl;
			//cout << result[0] << endl;

		}
	}
	return 0;
}
