#include "CPG.h"
#include <arm_math.h>
//extern CPG C;
float data1[8]={23.6933, 18.5434, 12.9354, 4.7784, 23.9841, 18.5434, 13.0312, 4.9651};
float data2[8]={ 6.2832, 12.5664, 6.2832, 12.5664, 6.2832, 12.5664, 6.2832, 12.5664 };
float data3[8]={ 11.3428, 11.3428, 3.7560, 3.7560, 11.4681, 11.4681, 3.9333, 3.9333 };
float data4[8][8]={{ 0,-1.6351, 0 ,0,0.0396,0,0,0}, 
                                      {0.8176,0 , -0.3513 ,0,0,0,0,0},
                                      {0,0.9793, 0,-0.0712,0,0,0,0},
                                      {0,0,0.0356 ,0, 0, 0, 0, 0},
                                      {-0.0396,0,0 ,0, 0, -1.6751, 0, 0},
                                      {0,0,0 ,0, 0.83755, 0, -0.49055, 0},
                                      {0,0,0 ,0,0, 0.9811, 0, 0.0125},
                                      {0,0,0 ,0,0, 0, 0.00625, 0},};
  
float data5[8][8]=	{ { 0, 1, 0, 0, 1, 0, 0, 0},  
                              { 1, 0, 1, 0, 0, 0, 0, 0},
                              { 0, 1, 0, 1, 0, 0, 0, 0}, 
                              { 0, 0, 1, 0, 0, 0, 0, 0},
                              { 1, 0, 0, 0, 0, 1, 0, 0}, 
                              { 0, 0, 0, 0, 1, 0, 1, 0},
                              { 0, 0, 0, 0, 0, 1, 0, 1},
                              { 0, 0, 0, 0, 0, 0, 1, 0}};																		
float data0[8]={0, 0, 0, 0, 0, 0, 0, 0};


float freqConst1=6.2832;
float freqConst2=12.566;
R dxPosition;
R dxVelocity;
R dxAcceleration;
float dxPositionTemp1[8][7];
float dxPositionTemp2[8][7];
float dxVelocityTemp1[8][7],dxVelocityTemp2[8][7];
float dxAccelerationTemp1[8][7],dxAccelerationTemp2[8][7];
void CPG_Init(CPG *C)
{
	
	for (int i=0;i<8;i++)
	{
	   C->ampConst[i]=data1[i]; 
	   C->freqConst[i]=data2[i];
	   C->offSet[i]=data3[i];
		 C->offSetvel[i]=0;
		 C->offSetacc[i]=0;
	}
 C->freqConst[0] = freqConst1; C->freqConst[1] = freqConst2; C->freqConst[2] = freqConst1; C->freqConst[3] = freqConst2;
 C->freqConst[4] = freqConst1; C->freqConst[5] = freqConst2; C->freqConst[6] = freqConst1; C->freqConst[7] = freqConst2;
 
 for (int i=0;i<8;i++)
	 {
		 C->freqConstvel[i] = C->freqConst[i];
		 C->freqConstacc[i] = C->freqConst[i];
	 }
	
	C->ampConstvel[0] = freqConst1 * C->ampConst[0]; C->ampConstvel[1] = freqConst2 * C->ampConst[1];
  C->ampConstvel[2] = freqConst1 * C->ampConst[2]; C->ampConstvel[3] = freqConst2 * C->ampConst[3];
  C->ampConstvel[4] = freqConst1 * C->ampConst[4]; C->ampConstvel[5] = freqConst2 * C->ampConst[5];
  C->ampConstvel[6] = freqConst1 * C->ampConst[6]; C->ampConstvel[7] = freqConst2 * C->ampConst[7];
  C->ampConstacc[0] = freqConst1 * C->ampConstvel[0]; C->ampConstacc[1] = freqConst2 * C->ampConstvel[1];
  C->ampConstacc[2] = freqConst1 * C->ampConstvel[2]; C->ampConstacc[3] = freqConst2 * C->ampConstvel[3];
  C->ampConstacc[4] = freqConst1 * C->ampConstvel[4]; C->ampConstacc[5] = freqConst2 * C->ampConstvel[5];
  C->ampConstacc[6] = freqConst1 * C->ampConstvel[6]; C->ampConstacc[7] = freqConst2 * C->ampConstvel[7];
	
	

	for (int i=0;i<8;i++)
	   for (int j=0;j<8;j++)
	   {
	       C-> phaseDiffpos[i][j]=data4[i][j];
	       C->diffFlag[i][j]=data5[i][j];
		 }
		
		 C->phaseWeights =2;
		 C->ampGain=10;
		 C->freqGain=10;
		 C->offGain =10;
		 C->Step=0.01;		 		
}

R GetStateVariables(float x[][7],CPG *C)
{

	dxPosition=OdeRungeKutta(x,C);
	
	return dxPosition;
}

R OdeRungeKutta(float x[][7],CPG *C)
{
	
	float Xtemp[8][7];
	float step;
	step=C->Step;
	R K1,K2,K3,K4;
	
	K1=CpgEquation(x,C);
	for (int i=0;i<8;i++)
	   for (int j=0;j<7;j++)
		     Xtemp[i][j]=x[i][j]+step*K1[i][j]/2;	
	
	K2=CpgEquation(Xtemp,C);
	for (int i=0;i<8;i++)
	   for (int j=0;j<7;j++)
		     Xtemp[i][j]=x[i][j]+step*K2[i][j]/2;		

	K3=CpgEquation(Xtemp,C);
	for (int i=0;i<8;i++)
	   for (int j=0;j<7;j++)
		     Xtemp[i][j]=x[i][j]+step*K3[i][j];		

	K4=CpgEquation(Xtemp,C);
	
	for (int i=0;i<8;i++)
	   for (int j=0;j<7;j++)
		    dxPositionTemp1[i][j]=x[i][j]+(K1[i][j]+2*K2[i][j]+2*K3[i][j]+K4[i][j])/6*step;		

		
	return dxPositionTemp1;
}

R CpgEquation(float xn[][7],CPG *C)
{
	
	float dxTemp[8][7];

	for (int i = 0; i < 8; i++)
            {
                dxTemp[i][0] = xn[i][3] + C->diffFlag[0][i] * C->phaseWeights *  arm_sin_f32(xn[0][0] - C->freqConst[0] / C->freqConst[i] * xn[i][0] - C->phaseDiffpos[i][0])
                                        + C->diffFlag[1][i] * C->phaseWeights *  arm_sin_f32(xn[1][0] - C->freqConst[1] / C->freqConst[i] * xn[i][0] - C->phaseDiffpos[i][1])
                                        + C->diffFlag[2][i] * C->phaseWeights *  arm_sin_f32(xn[2][0] - C->freqConst[2] / C->freqConst[i] * xn[i][0] - C->phaseDiffpos[i][2])
                                        + C->diffFlag[3][i] * C->phaseWeights *  arm_sin_f32(xn[3][0] - C->freqConst[3] / C->freqConst[i] * xn[i][0] - C->phaseDiffpos[i][3])
                                        + C->diffFlag[4][i] * C->phaseWeights *  arm_sin_f32(xn[4][0] - C->freqConst[4] / C->freqConst[i] * xn[i][0] - C->phaseDiffpos[i][4])
                                        + C->diffFlag[5][i] * C->phaseWeights *  arm_sin_f32(xn[5][0] - C->freqConst[5] / C->freqConst[i] * xn[i][0] - C->phaseDiffpos[i][5])
                                        + C->diffFlag[6][i] * C->phaseWeights *  arm_sin_f32(xn[6][0] - C->freqConst[6] / C->freqConst[i] * xn[i][0] - C->phaseDiffpos[i][6])
                                        + C->diffFlag[7][i] * C->phaseWeights *  arm_sin_f32(xn[7][0] - C->freqConst[7] / C->freqConst[i] * xn[i][0] - C->phaseDiffpos[i][7])
                                        ;
                dxTemp[i][1] = xn[i][2];
                dxTemp[i][2] = C->ampGain * (C->ampGain / 4 * (C->ampConst[i] - xn[i][1]) - xn[i][2]);
                dxTemp[i][3] = xn[i][4];
                dxTemp[i][4] = C->freqGain * (C->freqGain / 4 * (C->freqConst[i] - xn[i][3]) - xn[i][4]);
                dxTemp[i][5] = xn[i][6];
                dxTemp[i][6] = C->offGain * (C->offGain / 4 * (C->offSet[i] - xn[i][5]) - xn[i][6]);
            }
	for (int i=0;i<8;i++)
	   for (int j=0;j<7;j++)
         dxPositionTemp2[i][j]=dxTemp[i][j]; 

	return dxPositionTemp2;
}





R GetVelocity(float x[][7],float xnVel[][7],CPG *C)
{	
	dxVelocity=OdeRungeKuttaVel(x,xnVel,C);
	
	return dxVelocity;
}

R OdeRungeKuttaVel(float x[][7],float xnVel[][7],CPG *C)
{

	float Xtemp[8][7];
	float step;
	step=C->Step;
	R K1,K2,K3,K4;
	
	K1=CpgEquationVel(x,xnVel,C);
	for (int i=0;i<8;i++)
	   for (int j=0;j<7;j++)
	{
		Xtemp[i][j]=x[i][j]+step*K1[i][j]/2;		
	}
	K2=CpgEquationVel(Xtemp,xnVel,C);
	for (int i=0;i<8;i++)
	   for (int j=0;j<7;j++)
	{
		Xtemp[i][j]=x[i][j]+step*K2[i][j]/2;		
	}
	K3=CpgEquationVel(Xtemp,xnVel,C);
	for (int i=0;i<8;i++)
	   for (int j=0;j<7;j++)
	{
		Xtemp[i][j]=x[i][j]+step*K3[i][j];		
	}
	K4=CpgEquationVel(Xtemp,xnVel,C);
	
	for (int i=0;i<8;i++)
	   for (int j=0;j<7;j++)
	{
		dxVelocityTemp1[i][j]=xnVel[i][j]+(K1[i][j]+2*K2[i][j]+2*K3[i][j]+K4[i][j])/6*step;		
	}
		
	return dxVelocityTemp1;
}

R CpgEquationVel(float xn[][7],float xnVel[][7],CPG *C)
{
	float dxTemp[8][7];

	for (int i = 0; i < 8; i++)
            {
                dxTemp[i][0] = xnVel[i][3] + C->phaseWeights *  arm_sin_f32(xn[i][0] - xnVel[i][0] - C->phaseDiffvel);
                dxTemp[i][1] = xnVel[i][2];
                dxTemp[i][2] = C->ampGain * (C->ampGain / 4 * (C->ampConstvel[i] - xnVel[i][1]) - xnVel[i][2]);
                dxTemp[i][3] = xnVel[i][4];
                dxTemp[i][4] = C->freqGain * (C->freqGain / 4 * (C->freqConstvel[i] - xnVel[i][3]) - xnVel[i][4]);
                dxTemp[i][5] = xnVel[i][6];
                dxTemp[i][6] = C->offGain * (C->offGain / 4 * (C->offSetvel[i] - xnVel[i][5]) - xnVel[i][6]);
            }
	for (int i=0;i<8;i++)
	   for (int j=0;j<7;j++)
	{
         dxVelocityTemp2[i][j]=dxTemp[i][j]; 
	}
	return dxVelocityTemp2;
}


R GetAcceleration(float x[][7],float xnAcc[][7],CPG *C)
{
	
	dxAcceleration=OdeRungeKuttaAcc(x,xnAcc,C);
	
	return dxAcceleration;
}

R OdeRungeKuttaAcc(float x[][7],float xnAcc[][7],CPG *C)
{

	float Xtemp[8][7];
	float step;
	step=C->Step;
	R K1,K2,K3,K4;
	
	for (int i=0;i<8;i++)
	   for (int j=0;j<7;j++)
		     Xtemp[i][j]=x[i][j];	
	
	K1=CpgEquationAcc(Xtemp,xnAcc,C);
	
	for (int i=0;i<8;i++)
	   for (int j=0;j<7;j++)
		     Xtemp[i][j]=x[i][j]+step*K1[i][j]/2;		

	K2=CpgEquationAcc(Xtemp,xnAcc,C);
	
	for (int i=0;i<8;i++)
	   for (int j=0;j<7;j++)
	{
		Xtemp[i][j]=x[i][j]+step*K2[i][j]/2;		
	}
	K3=CpgEquationAcc(Xtemp,xnAcc,C);
	
	for (int i=0;i<8;i++)
	   for (int j=0;j<7;j++)
	{
		Xtemp[i][j]=x[i][j]+step*K3[i][j];		
	}
	K4=CpgEquationAcc(Xtemp,xnAcc,C);
	
	for (int i=0;i<8;i++)
	   for (int j=0;j<7;j++)
	{
		dxAccelerationTemp1[i][j]=xnAcc[i][j]+(K1[i][j]+2*K2[i][j]+2*K3[i][j]+K4[i][j])/6*step;		
	}
		
	return dxAccelerationTemp1;
}

R CpgEquationAcc(float xn[][7],float xnAcc[][7],CPG *C)
{

	float dxTemp[8][7];

	for (int i = 0; i < 8; i++)
            {
                dxTemp[i][0] = xnAcc[i][3] + C->phaseWeights *  arm_sin_f32(xn[i][0] - xnAcc[i][0] - C->phaseDiffacc);
                dxTemp[i][1] = xnAcc[i][2];
                dxTemp[i][2] = C->ampGain * (C->ampGain / 4 * (C->ampConstacc[i] - xnAcc[i][1]) - xnAcc[i][2]);
                dxTemp[i][3] = xnAcc[i][4];
                dxTemp[i][4] = C->freqGain * (C->freqGain / 4 * (C->freqConstacc[i] - xnAcc[i][3]) - xnAcc[i][4]);
                dxTemp[i][5] = xnAcc[i][6];
                dxTemp[i][6] = C->offGain * (C->offGain / 4 * (C->offSetacc[i] - xnAcc[i][5]) - xnAcc[i][6]);
            }
//	for (int i=0;i<8;i++)
//	   for (int j=0;j<7;j++)
//	{
//         dxAccelerationTemp2[i][j]=dxTemp[i][j]; 
//	}
//	int a=3;
//	int b=3;
	return dxTemp;
}

