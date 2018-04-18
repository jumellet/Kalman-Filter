//============================================================================
// Name        : Linear_Kalman_Filter.cpp
// Author      : Julien
// Version     :
// Copyright   : Your copyright notice
// Description : Hello World in C++, Ansi-style
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
	float G, s=0.869565, m=83.333; //Value of the gaussian distribution & his variance & mean
	for (int i=0; i<200; i++){
		//noise=rand() % 100+1; //noise in the range 0 to 100
		//noise-=50;			//make the noise average equal to zero
		//noise/=5;
		G=1000*1/(pow((2.0*3.1415),0.5)*s)*exp(-pow((i-m),2)/(2.0*pow(s,2.0)));
		//cout <<(G+noise)<<endl;
		cout <<G<<endl;
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

	float xAccelero, xLightH, xEst, xTrue; //Inputs, State estimate & Output
	float sXAccelero, sXLightH, sXEst, sXTrue=5; //Covariances associated
	float K;	//Kalman Gain

//Predict
	sXLightH=s;   //Using covariance of the Light House simulation
	xLightH=m;	//Using mean of the Light House simulation
	xAccelero=110.0;	//Mean of the accelero
	sXAccelero=20.0;	//Covariance of the accelero

	xEst=xAccelero;
	sXEst=sXAccelero;


//Update
	K=xEst/(xEst+sXTrue);
	xTrue=(xAccelero*sXLightH + xLightH*sXAccelero) / (sXAccelero + sXLightH);
			//weighted average for a 1D filter
	sXTrue=(1-K)*sXEst;

	//cout <<xEst<<", "<<sXEst<<", "<<xTrue<<", "<<sXTrue<<", "<<K <<endl;

	return 0;
}
