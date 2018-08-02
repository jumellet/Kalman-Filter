//============================================================================
// Name        : Linear_Kalman_Filter.cpp
// Author      : Julien
// Version     :
// Copyright   : Your copyright notice
// Description : KF linear  1D & 2D
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
	xTrue=(xEst*sXLightH + xLightH*sXEst) / (sXEst + sXLightH);
				//weighted average for a 1D filter
	//vvvvvv New covariance to test vvvvvv !!!!!!!
	//sXTrue = (1-K*sXTrue)*sXEst;
	sXTrue=(1-K)*sXEst;

	float *normXTrue = new float[2];
	normXTrue[0]=xTrue;
	normXTrue[1]=sXTrue;

	return normXTrue;
}

//===========================================================
//FUNCTION OF 2D LINEAR KALMAN FILTER
//===========================================================

//2x2 Matrix Multiplication
float **MatMultiplicationDD(float **A, float **B){
	float **C = new float*[2];
	C[0] = new float[2];
	C[1] = new float[2];	

	for (int i=0; i<2; i++){
		for (int j=0; j<2; j++){
			C[i][j] = A[i][0]*B[0][j] + A[i][1]*B[1][j];
		}
	}
	return C;
}

//3x3 Matrix Multiplication
float **MatMultiplicationDDD(float **A, float **B){
	float **C = new float*[3];
	C[0] = new float[3];
	C[1] = new float[3];
	C[2] = new float[3];

	for (int i=0; i<3; i++){
		for (int j=0; j<3; j++){
			C[i][j] = A[i][0]*B[0][j] + A[i][1]*B[1][j] + A[i][2]*B[2][j];
		}
	}
	return C;
}

//2x2 Matrix Inversion
float **MatInverseDD(float **A){
	float **C = new float*[2];
	C[0] = new float[2];
	C[1] = new float[2];
	
	float det_C = A[0][0]*A[1][1] - A[0][1]*A[1][0];

	C[0][0] = 1/det_C * A[1][1];
	C[0][1] = -1/det_C * A[0][1];
	C[1][0] = -1/det_C * A[1][0];
	C[1][1] = 1/det_C * A[0][0];

	return C;

}

//3x3 Matrix Inversion by Sarrus
float **MatInverseDDD(float **A){
	float **C = new float*[3];
	C[0] = new float[3];
	C[1] = new float[3];
	C[2] = new float[3];

	float det_C = A[0][0]*A[1][1]*A[2][2] + A[0][1]*A[1][2]*A[2][0] + A[0][2]*A[1][0]*A[2][1] - A[0][2]*A[1][1]*A[2][0] - A[0][0]*A[1][2]*A[2][1] - A[0][1]*A[1][0]*A[2][2];

	C[0][0] = 1/det_C * (A[1][1]*A[2][2] - A[1][2]*A[2][1]);
	C[0][1] = 1/det_C * (A[0][2]*A[2][1] - A[0][1]*A[2][2]);
	C[0][2] = 1/det_C * (A[0][1]*A[1][2] - A[0][2]*A[1][1]);
	C[1][0] = 1/det_C * (A[1][2]*A[2][0] - A[1][0]*A[2][2]);
	C[1][1] = 1/det_C * (A[0][0]*A[2][2] - A[0][2]*A[2][0]);
	C[1][2] = 1/det_C * (A[0][2]*A[1][0] - A[0][0]*A[1][2]);
	C[2][0] = 1/det_C * (A[1][0]*A[2][1] - A[1][1]*A[2][0]);
	C[2][1] = 1/det_C * (A[0][1]*A[2][0] - A[0][0]*A[2][1]);
	C[2][2] = 1/det_C * (A[0][0]*A[1][1] - A[0][1]*A[1][0]);
	
	return C;	
}

//2x2 Matrix Transpose
float **MatTransposeDD(float **A){
	float **C = new float*[2];
	C[0] = new float[2];
	C[1] = new float[2];

	C[0][0] = A[0][0];
	C[0][1] = A[1][0];
	C[1][0] = A[0][1];
	C[1][1] = A[1][1];

	return C;
}

//3x3 Matrix Transpose
float **MatTransposeDDD(float **A){
	float **C = new float*[3];
	C[0] = new float[3];
	C[1] = new float[3];
	C[2] = new float[3];

	for (int i=0; i<3; i++){
		for (j=0; j<3; j++){
			C[i][j] = A[j][i] ;
		}
	}
	return C;
}

//Vectors have to be input


float **linearKFDD(float *uAccelero, float *sUAccelero, float *uLightH, float *sULightHI, float *uTrue, float *sUTrueI)
{
	int i,j ;	//allows the increment on for loops
	float *uEst = new float[2];		//u estimated (=u predicted)
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
	float **I = new float*[2];	//Identity matrix
	I[0] = new float[2];
	I[1] = new float[2];
	I[0][0]=1; I[0][1]=0; I[1][0]=0; I[1][1]=1;

	//PREDICT (which normally follows a mathematical law, but here the prediction is given by IMU)
		//Also there is the construction of covariances matrix of sUEst
	for (i=0; i<2; i++){
		uEst[i]=uAccelero[i];
		for (j=0; j<2; j++){
			if (i==j){
				sUEst[i][j]=sqrt(sUAccelero[i]) * sqrt(sUAccelero[j]);
			}
			else{
				sUEst[i][j]=0;
			}
			//cout << "uEst"<<i<<" = "<< uEst[i] << endl;
			//cout << "sUEst"<<i<<","<<j<<" = "<< sUEst[i][j] << endl;
		}
	}
		//Construction of sUTrue an sULightH, the covariances matrix from sUTrueI
	for (i=0; i<2; i++){
		for (j=0; j<2; j++){
			//sULightH[i][j] = sqrt(sULightHI[i]) * sqrt(sULightHI[j]);
			if (i==j){
				sUTrue[i][j] = sqrt(sUTrueI[i]) * sqrt(sUTrueI[j]);
				sULightH[i][j] = sqrt(sULightHI[i]) * sqrt(sULightHI[j]);
			}
			else {
				sUTrue[i][j] = 0;
				sULightH[i][j] = 0;
			}
			//cout << "sUTrue"<<i<<","<<j<<" = "<< sUTrue[i][j]<< endl;
			//cout << "sULightH"<<i<<","<<j<<" = "<< sULightH[i][j]<< endl;
		}
	}

	//UPDATE

		//coefficient of the 2D matricial inverse, it's also Innovation (or residual) covariance
	/*
	float coefK = 1 / ((sUEst[0][0]+sUTrue[0][0])*(sUEst[1][1]+sUTrue[1][1]) - (sUEst[0][1]+sUTrue[0][1])*(sUEst[1][0]+sUTrue[1][0]));
	tempMat[0][0]=sUEst[1][1]+sUTrue[1][1];
	tempMat[0][1]=-sUEst[0][1]-sUTrue[0][1];
	tempMat[1][0]=-sUEst[1][0]-sUTrue[1][0];
	tempMat[1][1]=sUEst[0][0]+sUTrue[0][0];
	*/

	float coefK = 1 / ((sUEst[0][0]+sULightH[0][0])*(sUEst[1][1]+sULightH[1][1]) - (sUEst[0][1]+sULightH[0][1])*(sUEst[1][0]+sULightH[1][0]));
	tempMat[0][0]=sUEst[1][1]+sULightH[1][1];
	tempMat[0][1]=-sUEst[0][1]-sULightH[0][1];
	tempMat[1][0]=-sUEst[1][0]-sULightH[1][0];
	tempMat[1][1]=sUEst[0][0]+sULightH[0][0];
		//Calculus of Kalman gain K
	for (i=0; i<2; i++){
		for (j=0; j<2; j++){
			//Multiply it by sUEst and uTrue.transpose

			K[i][j] = coefK * (sUEst[i][0]*tempMat[0][j] + sUEst[i][1]*tempMat[1][j]);
			//cout << "K"<<i<<","<<j<<" = "<<K[i][j]<< endl;
			//cout << "S"<<i<<","<<j<<" = "<<tempMat[i][j]<< endl;
		}
	}
		//Calculus of uTrue
	for (i=0; i<2; i++){
		//uTrue[i] = uEst[i] + (K[i][0]*(uLightH[0]-uEst[0]) + K[i][1]*(uLightH[1]-uEst[1]))   ;
		//uTrue[i] += K[i][i] * (uLightH[i] - uEst[i]);
		//vvvv HERE uTrue doesn't realy follow KF Algorithm, but allow good results
		uTrue[i] = uEst[i] + K[i][i] * (uEst[i] - uLightH[i]);
		//cout << "uLightH"<<i<<" = "<<uLightH[i]<< endl;
		//cout << "uEst"<<i<<" = "<<uEst[i]<< endl;
		//cout << "uTrue"<<i<<" = "<<uTrue[i]<< endl;
	}
	//cout <<"=====================================" << endl;
		//Calculus of sUTrue
	for (i=0; i<2; i++){
		for (j=0; j<2; j++){
			//sUTrue[i][j] = sUEst[i][j] - (K[i][0]*sUEst[0][j] + K[i][1]*sUEst[1][j]) ;
			tempMat[i][j] = (I[i][j]-K[i][j]);
			sUTrue[i][j] = tempMat[i][0]*sUEst[0][j] + tempMat[i][1]*sUEst[1][j];
			//cout << "sUTrue"<<i<<","<<j<<" = "<<sUTrue[i][j]<< endl;
		}
	}
	//cout <<"=====================================" << endl;

	float **normUTrue = new float*[2];
	normUTrue[0] = new float[2];
	normUTrue[0][0] = uTrue[0];
	normUTrue[0][1] = uTrue[1];
	normUTrue[1] = new float[2];
	normUTrue[1][0] = sUTrue[0][0];
	normUTrue[1][1] = sUTrue[1][1];

	//cout << "normUTrue [0] : " << normUTrue[0][0] <<", "<< normUTrue[0][1] << endl;
	//cout << "normUTrue [1] : " << normUTrue[1][0] <<", "<< normUTrue[1][1] << endl;

	delete[] K[0];
	delete[] K[1];
	delete[] K;

	delete[] I[0];
	delete[] I[1];
	delete[] I;

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
			//xAcceleroTab[j]=i*4+j*1.2;	//Linear movement
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
cout<<"+++++++++++START++++++++++"<<endl;

//INITIALISATION
	float **result = new float*[2]; //Matrix to return uTrue & sUTrue (respectively u vector calculated and his covariance)
	result[0] = new float[2];
	result[0][0] = 0.0;	//uTrue is initialized to x=y=0
	result[0][1] = 0.0;
	result[1] = new float[2];
	result[1][0] = 20.0;	//sUTrue is initialized sX=sY=20
	result[1][1] = 20.0;

	//IMU Simulation
	float **uAcceleroTab = new float*[4];	//Matrix of 4 coordinates of points observed
	uAcceleroTab[0] = new float[2];
	uAcceleroTab[1] = new float[2];
	uAcceleroTab[2] = new float[2];
	uAcceleroTab[3] = new float[2];
	uAcceleroTab[0][0] = 0.0;
	uAcceleroTab[0][1] = 0.0;
	/*
	uAcceleroTab[0][0] = 0.0-4.0;
	uAcceleroTab[0][1] = 0.0-4.0;

	for(int i=1; i<=4; i++){
		uAcceleroTab[i][0]=i*1.2-4.0;
		uAcceleroTab[i][1]=i*1.2-4.0;
	}*/


	float **sUAcceleroTab = new float*[4];	//Matrix of 4 covariances associated of points observed
	sUAcceleroTab[0] = new float[2];
	sUAcceleroTab[1] = new float[2];
	sUAcceleroTab[2] = new float[2];
	sUAcceleroTab[3] = new float[2];
	sUAcceleroTab[0][0] = 20;
	sUAcceleroTab[0][1] = 20;

	//Light House Simulation
	float *uLightHTab = new float[2];
	uLightHTab[0] = 0;
	uLightHTab[1] = 0;
	float **sULightHTab = new float*[2];
	sULightHTab[0] = new float[2];
	sULightHTab[1] = new float[2];
	sULightHTab[2] = new float[2];
	sULightHTab[3] = new float[2];
	sULightHTab[0][0] = 20;
	sULightHTab[0][1] = 20;

	//Deplacement Simulation
	//2D circular movement around point 0 radius 4
		float circX[21]={-4.0,-3.98,-3.92,-3.82,-3.67,-3.46,-3.2,-2.86,-2.4,-1.74,0,1.74,2.4,2.86,3.2,3.46,3.67,3.82,3.92,3.98,4.0};
		float circY[21]={4.0,3.98,3.92,3.82,3.67,3.46,3.2,2.86,2.4,1.74,0,1.74,2.4,2.86,3.2,3.46,3.67,3.82,3.92,3.98,4.0};

	//Measurement on KF Algorithm
	for (int i=0; i<=4; i++){
			for (int j=0; j<=3; j++){
				int t = j+i*4; //Instants t

				//cout<<t<<" "<<t<<"+++++++ITERATION+++++++"<<t<<" "<<t<<endl;

				//IMU Simulation
				//uAcceleroTab[j][0]=i*4+j*1.2;	//Linear movement
				//uAcceleroTab[j][1]=i*4+j*1.2;
				uAcceleroTab[j][0]=circX[t]*1.2;	//Circular movement
				uAcceleroTab[j][1]=circY[t]*1.2;
				sUAcceleroTab[j][0]=sUAcceleroTab[0][0]+j*3;
				sUAcceleroTab[j][1]=sUAcceleroTab[0][1]+j*3;
				//cout<<"xAccelero"<<i<<","<<j<<" = "<<uAcceleroTab[j][0]<< endl;
				//cout<<"yAccelero"<<i<<","<<j<<" = "<<uAcceleroTab[j][1]<< endl;
				//cout<<"sXAccelero"<<i<<","<<j<<" = "<<sUAcceleroTab[j][0]<< endl;
				//cout<<"sYAccelero"<<i<<","<<j<<" = "<<sUAcceleroTab[j][1]<< endl;

				//cout <<"=====================================" << endl;

				//LightHouse Simulation
				//uLightHTab[0]=i*4;	//Linear movement
				//uLightHTab[1]=i*4;
				uLightHTab[0]=circX[i*4];	//Circular movement
				uLightHTab[1]=circY[i*4];
				sULightHTab[j][0]=sULightHTab[0][0]+5*j;
				sULightHTab[j][1]=sULightHTab[0][1]+5*j;
				//cout<<"xLightH"<<i<<","<<j<<" = "<<uLightHTab[0]<< endl;
				//cout<<"yLightH"<<i<<","<<j<<" = "<<uLightHTab[1]<< endl;
				//cout<<"sXLightH"<<i<<","<<j<<" = "<<sULightHTab[j][0]<< endl;
				//cout<<"sYLightH"<<i<<","<<j<<" = "<<sULightHTab[j][1]<< endl;

				result = linearKFDD(uAcceleroTab[j], sUAcceleroTab[j], uLightHTab, sULightHTab[j], result[0], result[1]);

				//cout << "x"<<i<<"  "<< result[0] << "\nsx"<<i<<" " << result[1] <<"\n======"<< endl;
				//cout << circX[t] <<" , "<< result[0][0] << " , "<< result[1][0] <<" , "<< circY[t] <<" , "<< result[0][1] << " , "<< result[1][1] << endl;
				cout << result[1][1] << endl;

				//cout << "vvvvvvvvvvvvvvvvvvvvvvvvvv" <<endl;
				//cout << "x  & y  : "<< result[0][0] << " & "<< result[0][1] << endl;
				//cout << "sX & sY : "<< result[1][0] << " & "<< result[1][1] << endl;
				//cout << "^^^^^^^^^^^^^^^^^^^^^^^^^^" <<endl;

			}
	}
	//result = linearKFDD(uAccelero, sUAccelero, uLightH, sULightH, result[0], result[1]);

	//cout << "x  & y  : "<< result[0][0] << " & "<< result[0][1] << endl;
	//cout << "sX & sY : "<< result[1][0] << " & "<< result[1][1] << endl;

	/*
	for (int i=0 ; i<4 ; i++){
		delete[] uAcceleroTab[i];
		delete[] sUAcceleroTab[i];
		delete[] sULightHTab[i];
	}
	delete[] uAcceleroTab;
	delete[] sUAcceleroTab;

	delete[] uLightHTab;
	delete[] sULightHTab;
*/
	delete[] result[0];
	delete[] result[1];

	delete[] result;

	cout<<"++++++++++++END+++++++++++"<<endl;

	return 0;
}
