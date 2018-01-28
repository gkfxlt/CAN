//#include "stm32f7xx_hal.h"

///* USER CODE BEGIN Includes */
//#include "stdio.h"
//#include <math.h>
//#include "string.h"
//#include "stm32f7xx_it.h"


typedef struct
{	
	float                 ampGain;  

  float                 freqGain;      

	float                 Step;
	
  float                 offGain;

  float                 phaseWeights;

  float                 phaseDiffvel;

  float                 phaseDiffacc;
	
	float                 ampConst[8];
		
	float                 freqConst[8];
	
	float                 offSet[8];
	
	float                 phaseDiffpos[8][8];
	
	float                 diffFlag[8][8];
	
	float                 ampConstvel[8];
		
	float                 freqConstvel[8];
	
	float                 offSetacc[8];
		
	float                 ampConstacc[8];
		
	float                 freqConstacc[8];
	
	float                 offSetvel[8];
			
}CPG;
typedef float (*R)[7]; 

void CPG_Init(CPG * C);
R GetStateVariables(float x[][7],CPG * C);
R OdeRungeKutta(float x[][7],CPG * C);
R CpgEquation(float x[][7],CPG * C);
R GetVelocity(float x[][7],float xnVel[][7],CPG * C);
R OdeRungeKuttaVel(float x[][7],float xnVel[][7],CPG * C);
R CpgEquationVel(float x[][7],float xnvel[][7],CPG * C);
R GetAcceleration(float x[][7],float xnAcc[][7],CPG * C);
R OdeRungeKuttaAcc(float x[][7],float xnAcc[][7],CPG * C);
R CpgEquationAcc(float x[][7],float xnAcc[][7],CPG * C);
