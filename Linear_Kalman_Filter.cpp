//============================================================================
// Name        : Linear_Kalman_Filter.cpp
// Author      : Julien
// Version     : 1.1
// Copyright   : Your copyright notice
// Description : 1D Linear Kalman Filter used for a IMU simulation
//============================================================================

//#include <graphics.h>

#include <math.h>
//#include <conio.h>
#include <stdlib.h>
#include <iostream>
using namespace std;

int main() {

//IMU Simulation
	int noise, ka=0, D; //Noise and Constant ka to simulate Derivation D of IMU
	for (int i=0; i<200; i++){
		noise=rand() % 100+1; //noise in the range 0 to 100
		//noise-=50;
		ka+=1;
		D=ka+noise;
		//cout <<  D <<endl;
	}

//Measurement Simulation
	float G, s=10, m=70; //Value of the gaussian distribution & his variance & mean
	for (int i=0; i<200; i++){
		//noise=rand() % 100+1; //noise in the range 0 to 100
		//noise-=50;			//make the noise average equal to zero
		//noise/=5;
		//Equation of a gaussian
		G=1000*1/(pow((2.0*3.1415),0.5)*sqrt(s))*exp(-pow((i-m),2)/(2.0*pow(sqrt(s),2.0)));
		//cout <<(G+noise)<<endl;
		//cout <<G<<endl;
	}


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
//LINEAR KALMAN FILTER
//===========================================================

	float xAccelero[3], xLightH, xEst, xTrue; //Inputs, State estimate & Output
	float sXAccelero[3], sXLightH, sXEst, sXTrue=20; //Covariances associated
	float K;	//Kalman Gain

	sXLightH=s;   //Using covariance of the Light House simulation
	xLightH=m;	//Using mean of the Light House simulation
	xAccelero[0]=110.0;	//Mean of the accelero
	sXAccelero[0]=20.0;	//Covariance of the accelero

	//Simulation of Accelero derivation : x derive & variance associated increase

	//cout<<xAccelero[0]<<", "<<sXAccelero[0]<<endl;
	for (int i=1; i<=3; i++){
		xAccelero[i]=xAccelero[0]+i*10;
		sXAccelero[i]=sXAccelero[0]+i*5;
		//cout<<xAccelero[i]<<", "<<sXAccelero[i]<<endl;

	}

	for (int i=0; i<4; i++){

		//Predict (which normally follows a mathematical law)
		xEst=xAccelero[i];
		sXEst=sXAccelero[i];


		//Update
		K=xEst/(xEst+sXTrue);
		xTrue=(xAccelero[i]*sXLightH + xLightH*sXAccelero[i]) / (sXAccelero[i] + sXLightH);
				//weighted average for a 1D filter
		sXTrue=(1-K)*sXEst;

		cout <<xEst<<", "<<sXEst<<", "<<xTrue<<", "<<sXTrue<<", "<<K <<endl;
		cout <<"=============================="<<endl;
	}

	return 0;
}
