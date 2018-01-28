/**
  ******************************************************************************
  * @file    stm32f7xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  *
  * COPYRIGHT(c) 2016 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "stm32f7xx_hal.h"
#include "stm32f7xx.h"
#include "stm32f7xx_it.h"

/* USER CODE BEGIN 0 */

#include "stdio.h"
#include <arm_math.h>
#include "string.h"
#include "main.h"


extern float kneePOS,kneeVEL,kneeACC,hipPOS,hipVEL,hipACC;
float Amp[8]={23.6933, 18.5434, 12.9354, 4.7784, 23.9841, 18.5434, 13.0312, 4.9651};
float Fre[8]={1, 2, 6.2832, 12.5664, 6.2832, 12.5664, 6.2832, 12.5664 };
float Offset[8]={ 11.3428, 11.3428, 3.7560, 3.7560, 11.4681, 11.4681, 3.9333, 3.9333 };			
extern float aHipFinal[3];
extern float aHip[3];
extern float phiHip[3];
extern float aKneeFinal[3];
extern float aKnee[3];
extern float phiKnee[3];
float Pi=3.14159;
int flag;
extern CanTxMsgTypeDef            pTxMsgSensor;
extern CanTxMsgTypeDef            pTxMsgSensor1;
extern CanRxMsgTypeDef            pRxMsgSensor;
extern CanRxMsgTypeDef            pRxMsgHipSensor;
extern CanRxMsgTypeDef            pRxMsgKneeSensor;
extern CanTxMsgTypeDef            pTxMsgTorque;
extern CanRxMsgTypeDef            pRxMsgTorque;
extern HAL_StatusTypeDef          stateCAN;
extern float controlTime;
int flagAxisA=0,flagAxisB=0;
int voltageHip=0,voltageKnee=0;
float hipTorqueTemp,kneeTorqueTemp,hipTorque,kneeTorque;
float temp1=0,temp2=0,temp3=0,temp4=0,temp5=0,temp6=0,temp7=0,temp8=0,temp9=0,temp10=0,temp11=0,temp12=0,temp13=0,temp0=0;
extern float sensorHipPositiont1,sensorHipPositiont2,sensorHipVelocity;
extern float sensorKneePositiont1,sensorKneePositiont2,sensorKneeVelocity;
float qrDotKNEE=0,qrDDotKNEE=0;
float qrDotHIP=0,qrDDotHIP=0;
float KP1=10,KD1=10;//For HIP
float KP2=5,KD2=10;//For KNEE
float signHipVEL=0;
float signKneeVEL=0;
float hHat[4],cHat[4],gHat[2],fHat[2];
extern float paraVector[19];
float Y1[11];
float Y2[11];
float error[2];
float dt=0.001;
float nominalKneeTorque=28.9;
float nominalHipTorque=53.28;
float g=9.81;
float torHIP=0;
float torKNEE=0;
float sinEncposKNEE;
float cosEncposKNEE;
float cosEncposHIP;
float cosEncposHIPKNEE;
float sinEncposHIPKNEE;
float sinEncposHIP;
float ratio=0.017453;
extern uint32_t uhADCxConvertedValue[8];
extern ADC_HandleTypeDef hadc1;
extern ADC_ChannelConfTypeDef sConfig;
extern float filter_a[3];
extern float filter_b[3];
float filter_Ta[3]={1,-1.143,0.4128};
float filter_Tb[3]={0.0675,0.1349,0.0675};
extern float filterdADCvalue[8][4];
extern float rawADCvalue[8][3];
float absFun(float);

uint8_t packetHeader[4]={0x0F,0x0F,0x0F,0x0F};
uint8_t packetTail[4]={0xF0,0xF0,0xF0,0xF0};
uint8_t desPos[2]={0x0F,0xFF};
uint8_t actPos[2]={0xFF,0xFF};
uint8_t emg1[2]={0xFF,0xFF};
uint8_t emg2[2]={0xFF,0xFF};
uint8_t emg3[2]={0xFF,0xFF};
uint8_t emg4[2]={0xFF,0xFF};
uint8_t tor1[2]={0xFF,0xFF};
uint8_t tor2[2]={0xFF,0xFF};
uint8_t para[2]={0xFF,0xFF};
uint8_t sum1[3]={0xFF,0xFF};

float KP=20,KI=4.0;
float offSet[8]={0,0,0,0,0,0,0,0};
float torSumKnee=0,torSumHip=0;
float errTorKnee=0,errTorHip=0;
int tick=0;
extern float piHalf;
extern uint8_t aRxBuffer[1];
extern float Gamma[11];
extern int usartSend;
/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;

/******************************************************************************/
/*            Cortex-M7 Processor Interruption and Exception Handlers         */ 
/******************************************************************************/

/**
* @brief This function handles Non maskable interrupt.
*/
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  HAL_RCC_NMI_IRQHandler();
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */

  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
* @brief This function handles Hard fault interrupt.
*/
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
  }
  /* USER CODE BEGIN HardFault_IRQn 1 */

  /* USER CODE END HardFault_IRQn 1 */
}

/**
* @brief This function handles Memory management fault.
*/
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
  }
  /* USER CODE BEGIN MemoryManagement_IRQn 1 */

  /* USER CODE END MemoryManagement_IRQn 1 */
}

/**
* @brief This function handles Pre-fetch fault, memory access fault.
*/
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
  }
  /* USER CODE BEGIN BusFault_IRQn 1 */

  /* USER CODE END BusFault_IRQn 1 */
}

/**
* @brief This function handles Undefined instruction or illegal state.
*/
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
  }
  /* USER CODE BEGIN UsageFault_IRQn 1 */

  /* USER CODE END UsageFault_IRQn 1 */
}

/**
* @brief This function handles Debug monitor.
*/
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  while (1)
  {
  }
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
* @brief This function handles System tick timer.
*/
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  HAL_SYSTICK_IRQHandler();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F7xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f7xx.s).                    */
/******************************************************************************/

/**
* @brief This function handles TIM2 global interrupt.
*/
void TIM2_IRQHandler(void)
{
  /* USER CODE BEGIN TIM2_IRQn 0 */
  HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_SET);
  /* USER CODE END TIM2_IRQn 0 */
  HAL_TIM_IRQHandler(&htim2);
  /* USER CODE BEGIN TIM2_IRQn 1 */
	
	/*PA1,U4*/
	sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  HAL_ADC_ConfigChannel(&hadc1, &sConfig);
	HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1, 10);
    /* ADC conversion completed */
    /*##-5- Get the converted value of regular channel  ########################*/
  if(HAL_ADC_GetState(&hadc1) == HAL_ADC_STATE_EOC_REG+1) 
	 {
    HAL_GPIO_WritePin(LD3_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
		uhADCxConvertedValue[0] = HAL_ADC_GetValue(&hadc1);
	 }
	 
	/*PA2,U5*/	
	sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_15CYCLES;
  HAL_ADC_ConfigChannel(&hadc1, &sConfig);
	HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1, 10);
    /* ADC conversion completed */
    /*##-5- Get the converted value of regular channel  ########################*/
  if(HAL_ADC_GetState(&hadc1) == HAL_ADC_STATE_EOC_REG+1) 
	 {
    HAL_GPIO_WritePin(LD3_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
		uhADCxConvertedValue[1] = HAL_ADC_GetValue(&hadc1);
	 }
		
  /*PA6,U6*/	 
	sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_15CYCLES;
  HAL_ADC_ConfigChannel(&hadc1, &sConfig);
	HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1, 10);
    /* ADC conversion completed */
    /*##-5- Get the converted value of regular channel  ########################*/
  if(HAL_ADC_GetState(&hadc1) == HAL_ADC_STATE_EOC_REG+1) 
	 {
    HAL_GPIO_WritePin(LD3_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
		uhADCxConvertedValue[2] = HAL_ADC_GetValue(&hadc1);
	 }
 
	/*PA7,U7*/
	sConfig.Channel = ADC_CHANNEL_7;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_15CYCLES;
  HAL_ADC_ConfigChannel(&hadc1, &sConfig);
	HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1, 10);
    /* ADC conversion completed */
    /*##-5- Get the converted value of regular channel  ########################*/
  if(HAL_ADC_GetState(&hadc1) == HAL_ADC_STATE_EOC_REG+1) 
	 {
    HAL_GPIO_WritePin(LD3_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
		uhADCxConvertedValue[3] = HAL_ADC_GetValue(&hadc1);
	 }
	 
	/*PB1,U8*/
	sConfig.Channel = ADC_CHANNEL_9;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_15CYCLES;
  HAL_ADC_ConfigChannel(&hadc1, &sConfig);
	HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1, 10);
    /* ADC conversion completed */
    /*##-5- Get the converted value of regular channel  ########################*/
  if(HAL_ADC_GetState(&hadc1) == HAL_ADC_STATE_EOC_REG+1) 
	 {
    HAL_GPIO_WritePin(LD3_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
		uhADCxConvertedValue[4] = HAL_ADC_GetValue(&hadc1);
	 }
	
  /*PC0,U9*/	 
	sConfig.Channel = ADC_CHANNEL_10;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_15CYCLES;
  HAL_ADC_ConfigChannel(&hadc1, &sConfig);
	HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1, 10);
    /* ADC conversion completed */
    /*##-5- Get the converted value of regular channel  ########################*/
  if(HAL_ADC_GetState(&hadc1) == HAL_ADC_STATE_EOC_REG+1) 
	 {
    HAL_GPIO_WritePin(LD3_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
		uhADCxConvertedValue[5] = HAL_ADC_GetValue(&hadc1);
	 }
	
  /*PC2,U11*/	 
	sConfig.Channel = ADC_CHANNEL_12;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_15CYCLES;
  HAL_ADC_ConfigChannel(&hadc1, &sConfig);
	HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1, 10);
    /* ADC conversion completed */
    /*##-5- Get the converted value of regular channel  ########################*/
  if(HAL_ADC_GetState(&hadc1) == HAL_ADC_STATE_EOC_REG+1) 
	 {
    HAL_GPIO_WritePin(LD3_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
		uhADCxConvertedValue[6] = HAL_ADC_GetValue(&hadc1);
	 }
	 
	/*PC3,U10*/	
	sConfig.Channel = ADC_CHANNEL_13;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_15CYCLES;
  HAL_ADC_ConfigChannel(&hadc1, &sConfig);
	HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1, 10);
    /* ADC conversion completed */
    /*##-5- Get the converted value of regular channel  ########################*/
  if(HAL_ADC_GetState(&hadc1) == HAL_ADC_STATE_EOC_REG+1) 
	 {
    HAL_GPIO_WritePin(LD3_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
		uhADCxConvertedValue[7] = HAL_ADC_GetValue(&hadc1);
	 }

	 if(tick==0)
	 {
		 for(int i=0;i<5;i++)
		 {
		   filterdADCvalue[i][0]=(float)(uhADCxConvertedValue[i])-2048;
			 filterdADCvalue[i][0]=absFun(filterdADCvalue[i][0])/4096*3.3;
			 rawADCvalue[i][0]=filterdADCvalue[i][0];
		 }
		 for(int i=5;i<8;i++)
		 {
		   filterdADCvalue[i][0]=(float)(uhADCxConvertedValue[i]);
			 filterdADCvalue[i][0]=(filterdADCvalue[i][0]/4096*3.3*2-2.5)*24.39;
			 rawADCvalue[i][0]=filterdADCvalue[i][0];
			 
		 }
	 }
	 	 if(tick==1)
	 {
		 for(int i=0;i<5;i++)
		 {
		   filterdADCvalue[i][1]=(float)(uhADCxConvertedValue[i])-2048;
			 filterdADCvalue[i][1]=absFun(filterdADCvalue[i][1])/4096*3.3;
			 rawADCvalue[i][1]=filterdADCvalue[i][1];
		 }
		 for(int i=5;i<8;i++)
		 {
		   filterdADCvalue[i][1]=(float)(uhADCxConvertedValue[i]);
			 filterdADCvalue[i][1]=(filterdADCvalue[i][1]/4096*3.3*2-2.5)*24.39-offSet[i];
			 rawADCvalue[i][1]=filterdADCvalue[i][1];
			 
		 }
		 
	 }
	 	 if(tick==2)
	 {
		 for(int i=0;i<5;i++)
		 {
		  // filterdADCvalue[i][2]=uhADCxConvertedValue[i];
			 rawADCvalue[i][2]=(float)(uhADCxConvertedValue[i])-2048;
			 rawADCvalue[i][2]=absFun(rawADCvalue[i][2])/4096*3.3;
		 }
		  for(int i=5;i<8;i++)
		 {
		   filterdADCvalue[i][2]=(float)(uhADCxConvertedValue[i]);
			 filterdADCvalue[i][2]=(filterdADCvalue[i][2]/4096*3.3*2-2.5)*24.39-offSet[i];
			 rawADCvalue[i][2]=filterdADCvalue[i][2];
			 
		 }
	 }
	 
     if(tick>2)
	 {
	     for(int i=0;i<5;i++)
		 {
		     filterdADCvalue[i][2]=filter_b[0]*rawADCvalue[i][2]+filter_b[1]*rawADCvalue[i][1]+filter_b[2]*rawADCvalue[i][0]
		                          -filter_a[1]*filterdADCvalue[i][1]-filter_a[2]*filterdADCvalue[i][0];
			   rawADCvalue[i][0]=rawADCvalue[i][1];
			   rawADCvalue[i][1]=rawADCvalue[i][2];
			   rawADCvalue[i][2]=(float)(uhADCxConvertedValue[i])-2048;
			   rawADCvalue[i][2]=absFun(rawADCvalue[i][2])/4096*3.3;
			   filterdADCvalue[i][0]=filterdADCvalue[i][1];
			   filterdADCvalue[i][1]=filterdADCvalue[i][2];
			 
		 }		
		  for(int i=5;i<8;i++)
		 {
		     filterdADCvalue[i][2]=filter_Tb[0]*rawADCvalue[i][2]+filter_Tb[1]*rawADCvalue[i][1]+filter_Tb[2]*rawADCvalue[i][0]
		                          -filter_Ta[1]*filterdADCvalue[i][1]-filter_Ta[2]*filterdADCvalue[i][0];
			   rawADCvalue[i][0]=rawADCvalue[i][1];
			   rawADCvalue[i][1]=rawADCvalue[i][2];
			   rawADCvalue[i][2]=(float)(uhADCxConvertedValue[i]);
			   rawADCvalue[i][2]=(rawADCvalue[i][2]/4096*3.3*2-2.5)*24.39-offSet[i];
			   filterdADCvalue[i][0]=filterdADCvalue[i][1];
			   filterdADCvalue[i][1]=filterdADCvalue[i][2];
			 
		 }		
		 if(tick==100)
			 for(int i=0;i<8;i++)
		 {
			 offSet[i]=filterdADCvalue[i][1];
		 }

		 
	 }
			
	tick++;
	
	if(tick>100)
	{

	
	errTorHip=filterdADCvalue[7][2];
	torSumHip=torSumHip+errTorHip*dt;
	torHIP=KP*errTorHip+KI*torSumHip;
	 
  voltageHip=-1*(torHIP/nominalHipTorque)*1000;
  if(voltageHip>500)
		voltageHip=500;
	if(voltageHip<-500)
		voltageHip=-500;
	tempCounter=voltageHip;
  
	if(flagAxisA==0)
	{
	flagAxisA=1;
	pTxMsg.StdId=0x202;
  pTxMsg.IDE =CAN_ID_STD;
	pTxMsg.RTR =CAN_RTR_DATA;
	pTxMsg.DLC =2;
	pTxMsg.Data[0]=0x0f;
	pTxMsg.Data[1]=0x00;
	hcan1.pTxMsg=&pTxMsg;
	HAL_CAN_Transmit(&hcan1,10);
	}
	positionLow [0]=tempCounter&(0xFF);	
	positionHigh[0]=tempCounter>>8&(0xFF);
	pTxMsgTorque.StdId=0x302;
  pTxMsgTorque.IDE =CAN_ID_STD;
	pTxMsgTorque.RTR =CAN_RTR_DATA;
	pTxMsgTorque.DLC =2;
	pTxMsgTorque.Data[0]=positionLow [0];
	pTxMsgTorque.Data[1]=positionHigh[0];
	Timeout=10;
	hcan1.pTxMsg=&pTxMsgTorque;
	HAL_CAN_Transmit(&hcan1,10);
	
					
}
	
	HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_RESET);
  }
  /* USER CODE END TIM2_IRQn 1 */


/**
* @brief This function handles TIM3 global interrupt.
*/
void TIM3_IRQHandler(void)
{
  /* USER CODE BEGIN TIM3_IRQn 0 */

  /* USER CODE END TIM3_IRQn 0 */
  HAL_TIM_IRQHandler(&htim3);
  /* USER CODE BEGIN TIM3_IRQn 1 */
	
if(aRxBuffer[0]!=5)
	{
		short usartSend[35];
		short usartSum=0;
		
	HAL_UART_Transmit(&huart2, packetHeader, 4, 10);
		
	usartSend[0]=1000*hipPOS;		
	usartSend[1]=1000*sensorHipPositiont2;		
  usartSend[2]=1000*kneePOS;	
	usartSend[3]=1000*sensorKneePositiont2;

		
	usartSend[4]=100*hipTorque;	
	usartSend[5]=100*kneeTorque;	
	
	usartSend[6]=100*paraVector[0];	
	usartSend[7]=100*paraVector[1];	
	usartSend[8]=100*paraVector[2];	
	usartSend[9]=100*paraVector[3];	
	usartSend[10]=100*paraVector[4];	
	usartSend[11]=100*paraVector[5];	
	usartSend[12]=100*paraVector[6];	
	usartSend[13]=100*paraVector[7];	
	usartSend[14]=100*paraVector[8];	
	usartSend[15]=100*paraVector[9];	
	usartSend[16]=100*paraVector[10];	
	usartSend[17]=100*paraVector[11];	
	usartSend[18]=100*paraVector[12];	
	usartSend[19]=100*paraVector[13];	
	usartSend[20]=100*paraVector[14];
	usartSend[21]=100*paraVector[15];	
	usartSend[22]=100*paraVector[16];	
	usartSend[23]=100*paraVector[17];	
	usartSend[24]=100*paraVector[18];		
	
	usartSend[25]=100*filterdADCvalue[1][2];	
	usartSend[26]=100*filterdADCvalue[2][2];
	usartSend[27]=100*filterdADCvalue[3][2];	
	usartSend[28]=100*filterdADCvalue[4][2];	
	usartSend[29]=100*filterdADCvalue[6][2];	
	usartSend[30]=100*filterdADCvalue[7][2];	
	
	usartSend[31]=100*hipTorqueTemp;	
	usartSend[32]=100*kneeTorqueTemp;	
		
	usartSum=usartSum+usartSend[0];
	desPos[0]=usartSend[0]>>8&(0xFF);
	desPos[1]=usartSend[0]&(0xFF);	
	HAL_UART_Transmit(&huart2, desPos, 2, 10);			
  usartSum=usartSum+usartSend[1];		
  actPos[0]=usartSend[1]>>8&(0xFF);
	actPos[1]=usartSend[1]&(0xFF);	
	HAL_UART_Transmit(&huart2, actPos, 2, 10);	
  usartSum=usartSum+usartSend[2];
	desPos[0]=usartSend[2]>>8&(0xFF);
	desPos[1]=usartSend[2]&(0xFF);	
	HAL_UART_Transmit(&huart2, desPos, 2, 10);			
  usartSum=usartSum+usartSend[3];		
  actPos[0]=usartSend[3]>>8&(0xFF);
	actPos[1]=usartSend[3]&(0xFF);	
	HAL_UART_Transmit(&huart2, actPos, 2, 10);



  usartSum=usartSum+usartSend[4];		
  para[0]=usartSend[4]>>8&(0xFF);
	para[1]=usartSend[4]&(0xFF);	
	HAL_UART_Transmit(&huart2, para, 2, 10);
  usartSum=usartSum+usartSend[5];		
  para[0]=usartSend[5]>>8&(0xFF);
	para[1]=usartSend[5]&(0xFF);	
	HAL_UART_Transmit(&huart2, para, 2, 10);
//	
//	
//	
  usartSum=usartSum+usartSend[6];		
  para[0]=usartSend[6]>>8&(0xFF);
	para[1]=usartSend[6]&(0xFF);	
	HAL_UART_Transmit(&huart2, para, 2, 10);

  usartSum=usartSum+usartSend[7];		
  para[0]=usartSend[7]>>8&(0xFF);
	para[1]=usartSend[7]&(0xFF);	
	HAL_UART_Transmit(&huart2, para, 2, 10);

  usartSum=usartSum+usartSend[8];		
  para[0]=usartSend[8]>>8&(0xFF);
	para[1]=usartSend[8]&(0xFF);	
	HAL_UART_Transmit(&huart2, para, 2, 10);

  usartSum=usartSum+usartSend[9];		
  para[0]=usartSend[9]>>8&(0xFF);
	para[1]=usartSend[9]&(0xFF);	
	HAL_UART_Transmit(&huart2, para, 2, 10);

  usartSum=usartSum+usartSend[10];		
  para[0]=usartSend[10]>>8&(0xFF);
	para[1]=usartSend[10]&(0xFF);	
	HAL_UART_Transmit(&huart2, para, 2, 10);

  usartSum=usartSum+usartSend[11];		
  para[0]=usartSend[11]>>8&(0xFF);
	para[1]=usartSend[11]&(0xFF);	
	HAL_UART_Transmit(&huart2, para, 2, 10);
	
	usartSum=usartSum+usartSend[12];		
  para[0]=usartSend[12]>>8&(0xFF);
	para[1]=usartSend[12]&(0xFF);	
	HAL_UART_Transmit(&huart2, para, 2, 10);
	
	usartSum=usartSum+usartSend[13];		
  para[0]=usartSend[13]>>8&(0xFF);
	para[1]=usartSend[13]&(0xFF);	
	HAL_UART_Transmit(&huart2, para, 2, 10);
	
	usartSum=usartSum+usartSend[14];		
  para[0]=usartSend[14]>>8&(0xFF);
	para[1]=usartSend[14]&(0xFF);	
	HAL_UART_Transmit(&huart2, para, 2, 10);
	
	usartSum=usartSum+usartSend[15];		
  para[0]=usartSend[15]>>8&(0xFF);
	para[1]=usartSend[15]&(0xFF);	
	HAL_UART_Transmit(&huart2, para, 2, 10);
	
	usartSum=usartSum+usartSend[16];		
  para[0]=usartSend[16]>>8&(0xFF);
	para[1]=usartSend[16]&(0xFF);	
	HAL_UART_Transmit(&huart2, para, 2, 10);
	
	usartSum=usartSum+usartSend[17];		
  para[0]=usartSend[17]>>8&(0xFF);
	para[1]=usartSend[17]&(0xFF);	
	HAL_UART_Transmit(&huart2, para, 2, 10);
	
	usartSum=usartSum+usartSend[18];		
  para[0]=usartSend[18]>>8&(0xFF);
	para[1]=usartSend[18]&(0xFF);	
	HAL_UART_Transmit(&huart2, para, 2, 10);
	
	usartSum=usartSum+usartSend[19];		
  para[0]=usartSend[19]>>8&(0xFF);
	para[1]=usartSend[19]&(0xFF);	
	HAL_UART_Transmit(&huart2, para, 2, 10);
	
	usartSum=usartSum+usartSend[20];		
  para[0]=usartSend[20]>>8&(0xFF);
	para[1]=usartSend[20]&(0xFF);	
	HAL_UART_Transmit(&huart2, para, 2, 10);
	
	usartSum=usartSum+usartSend[21];		
  para[0]=usartSend[21]>>8&(0xFF);
	para[1]=usartSend[21]&(0xFF);	
	HAL_UART_Transmit(&huart2, para, 2, 10);
	
	usartSum=usartSum+usartSend[22];		
  para[0]=usartSend[22]>>8&(0xFF);
	para[1]=usartSend[22]&(0xFF);	
	HAL_UART_Transmit(&huart2, para, 2, 10);
	
	usartSum=usartSum+usartSend[23];		
  para[0]=usartSend[23]>>8&(0xFF);
	para[1]=usartSend[23]&(0xFF);	
	HAL_UART_Transmit(&huart2, para, 2, 10);
	
	usartSum=usartSum+usartSend[24];		
  para[0]=usartSend[24]>>8&(0xFF);
	para[1]=usartSend[24]&(0xFF);	
	HAL_UART_Transmit(&huart2, para, 2, 10);
	


	usartSum=usartSum+usartSend[25];
  emg1[0]=usartSend[25]>>8&(0xFF);
	emg1[1]=usartSend[25]&(0xFF);	
	HAL_UART_Transmit(&huart2, emg1, 2, 10);	
  usartSum=usartSum+usartSend[26];	
  emg2[0]=usartSend[26]>>8&(0xFF);
	emg2[1]=usartSend[26]&(0xFF);	
	HAL_UART_Transmit(&huart2, emg2, 2, 10);	
  usartSum=usartSum+usartSend[27];	
  emg3[0]=usartSend[27]>>8&(0xFF);
	emg3[1]=usartSend[27]&(0xFF);	
	HAL_UART_Transmit(&huart2, emg3, 2, 10);		
  usartSum=usartSum+usartSend[28];	
  emg4[0]=usartSend[28]>>8&(0xFF);
	emg4[1]=usartSend[28]&(0xFF);	
	HAL_UART_Transmit(&huart2, emg4, 2, 10);	
  usartSum=usartSum+usartSend[29];	
  tor1[0]=usartSend[29]>>8&(0xFF);
	tor1[1]=usartSend[29]&(0xFF);	
	HAL_UART_Transmit(&huart2, tor1, 2, 10);		
  usartSum=usartSum+usartSend[30];	
  tor2[0]=usartSend[30]>>8&(0xFF);
	tor2[1]=usartSend[30]&(0xFF);	
	HAL_UART_Transmit(&huart2, tor2, 2, 10);

  usartSum=usartSum+usartSend[31];	
  para[0]=usartSend[31]>>8&(0xFF);
	tor2[1]=usartSend[31]&(0xFF);	
	HAL_UART_Transmit(&huart2, para, 2, 10);
	usartSum=usartSum+usartSend[32];	
  para[0]=usartSend[32]>>8&(0xFF);
	para[1]=usartSend[32]&(0xFF);	
	HAL_UART_Transmit(&huart2, para, 2, 10);


  //usartSum=usartSum/100;
	sum1[0]=usartSum>>8&(0xFF);
	sum1[1]=usartSum&(0xFF);	
	HAL_UART_Transmit(&huart2, sum1, 2, 10);
	
	HAL_UART_Transmit(&huart2, packetTail, 4, 10);
	

  }
  /* USER CODE END TIM3_IRQn 1 */
}

/* USER CODE BEGIN 1 */
float absFun(float temp)
{
	float absTemp;
	if(temp>0)
		absTemp=temp;
	else
		absTemp=-temp;
	return absTemp;
}
/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
