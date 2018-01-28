/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
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

/* USER CODE BEGIN Includes */
#include "stdio.h"
#include <arm_math.h>
#include "string.h"
#include "stm32f7xx_it.h"
#include "main.h"
#include "CPG.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

CAN_HandleTypeDef hcan1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_CAN1_Init(void);
static void MX_TIM3_Init(void);
static void SystemClock_Config(void);
static void MX_ADC1_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

//void cr4_fft_256_stm32(void *pssOUT, void *pssIN, uint16_t Nbin);
 void CAN1_Filter_Init(void);
 void Servo_Init(void);
 int func(char s[],int x);
CPG C;
double c=1,a=1.00000001;
float cc=1,aa=1.00000001;

float x[8][7]={ { 0, 0, 0, 0, 0, 0, 0},  
                { 0, 0, 0, 0, 0, 0, 0},
                { 0, 0, 0, 0, 0, 0, 0}, 
                { 0, 0, 0, 0, 0, 0, 0},
                { 0, 0, 0, 0, 0, 0, 0}, 
                { 0, 0, 0, 0, 0, 0, 0},
                { 0, 0, 0, 0, 0, 0, 0},
                { 0, 0, 0, 0, 0, 0, 0}};
float xVel[8][7]={ { 0, 0, 0, 0, 0, 0, 0},  
                { 0, 0, 0, 0, 0, 0, 0},
                { 0, 0, 0, 0, 0, 0, 0}, 
                { 0, 0, 0, 0, 0, 0, 0},
                { 0, 0, 0, 0, 0, 0, 0}, 
                { 0, 0, 0, 0, 0, 0, 0},
                { 0, 0, 0, 0, 0, 0, 0},
                { 0, 0, 0, 0, 0, 0, 0}};
float xAcc[8][7]={ { 0, 0, 0, 0, 0, 0, 0},  
                { 0, 0, 0, 0, 0, 0, 0},
                { 0, 0, 0, 0, 0, 0, 0}, 
                { 0, 0, 0, 0, 0, 0, 0},
                { 0, 0, 0, 0, 0, 0, 0}, 
                { 0, 0, 0, 0, 0, 0, 0},
                { 0, 0, 0, 0, 0, 0, 0},
                { 0, 0, 0, 0, 0, 0, 0}};
float paraVector[19];
float sensorHipPositiont1=-PI/2,sensorHipPositiont2=-PI/2,sensorHipVelocity=0;
float sensorKneePositiont1=0,sensorKneePositiont2=0,sensorKneeVelocity=0;
float aHipFinal[3]={7.512,12.9354,4.7784};
float aHip[3]={0,0,0};
float phiHip[3]={0,-0.2828,-0.6368};
float aKneeFinal[3]={22.6855,23.6933,18.5434};
float aKnee[3]={0,0,0};
float phiKnee[3]={0,1.0244,0.4137};
float controlTime=0;
float kneePOS,kneeVEL,kneeACC,hipPOS,hipVEL,hipACC;
float timeStep=0.005;
float piHalf=1.5708;
														
uint32_t timeB,timeA,timeInterval;
uint32_t  transmitmailbox;
uint32_t Timeout;
CanTxMsgTypeDef            pTxMsg;
CanTxMsgTypeDef            pTxMsg1;
CanRxMsgTypeDef            pRxMsg;
CanRxMsgTypeDef            pRxMsg1;
CanTxMsgTypeDef            pTxMsgSensor;
CanTxMsgTypeDef            pTxMsgSensor1;
CanRxMsgTypeDef            pRxMsgSensor;
CanRxMsgTypeDef            pRxMsgHipSensor;
CanRxMsgTypeDef            pRxMsgKneeSensor;
CanTxMsgTypeDef            pTxMsgTorque;
CanRxMsgTypeDef            pRxMsgTorque;
HAL_StatusTypeDef          stateCAN;


uint8_t positionLow[1];
uint8_t positionLow1[1];
uint8_t positionLow2[1];
uint8_t positionHigh[1];

int sensorCounter,tempCounter,usartSend;
								
int sensorZero;
uint8_t aRxBuffer[1];
float Gamma[11]={1,1,1,1,1,1,1,1,1,1,1};

uint32_t uhADCxConvertedValue[8];
ADC_ChannelConfTypeDef sConfig;
float filter_a[3]={1,-1.9733,0.9737};
float filter_b[3]={8.7656e-5,1.7531e-4,8.7656e-5};
float filterdADCvalue[8][4];
float rawADCvalue[8][3];


int flagADC=0;

/* RLS */
float HL[4];
float K_NN[4];
float T_N0[4]={0,0,0,0};
float T_NN[4];
float P_NN[4][4];

float P_N0[4][4];

float rlsTemp0;
float rlsTemp1;

int rlsConst=10000;

/* END */

int fputc(int ch, FILE *f)
{
	uint8_t temp[1]={ch};
	HAL_UART_Transmit(&huart2, temp, 1, 2);
	return (ch);
}

void instructionParse(uint8_t aRxBuffer)
{
	if(aRxBuffer==5)
	{
	pTxMsg.StdId=0x201;
  pTxMsg.IDE =CAN_ID_STD;
	pTxMsg.RTR =CAN_RTR_DATA;
	pTxMsg.DLC =2;
	pTxMsg.Data[0]=0x05;
	pTxMsg.Data[1]=0x00;
	hcan1.pTxMsg=&pTxMsg;
	HAL_CAN_Transmit(&hcan1,10);	

//	HAL_UART_Transmit(&huart2, ackComm, 1, 10);
	pTxMsg.StdId=0x202;
  pTxMsg.IDE =CAN_ID_STD;
	pTxMsg.RTR =CAN_RTR_DATA;
	pTxMsg.DLC =2;
	pTxMsg.Data[0]=0x05;
	pTxMsg.Data[1]=0x00;
	hcan1.pTxMsg=&pTxMsg;
	HAL_CAN_Transmit(&hcan1,10);
		
	}
}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{ 
	uint8_t i=0; 
 // positionHigh[0]=1;		
	//HAL_UART_Transmit(&huart2, positionHigh, 1, 10);
	if(huart->Instance==USART2)
	{ 
		// positionHigh[0]=2;	
	 //  HAL_UART_Transmit(&huart2, positionHigh, 1, 10);
		instructionParse(aRxBuffer[0]);
	}
	HAL_UART_Receive_IT(&huart2,aRxBuffer,1);
	__HAL_UART_ENABLE_IT(huart, UART_IT_RXNE);
	__HAL_UART_DISABLE_IT(&huart2,UART_IT_RXNE);
	
}

/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */
  //uint8_t i=0;

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_TIM2_Init();
  MX_CAN1_Init();
  MX_TIM3_Init();
  SystemClock_Config();
  MX_ADC1_Init();

  /* USER CODE BEGIN 2 */
	
	CPG_Init(&C);
	Servo_Init();
	HAL_GPIO_WritePin(LD3_GPIO_Port, LD1_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(LD3_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
  //printf("\n\r UART Printf Example: retarget the C library printf function to the UART\n\r");
  //printf("** Test finished successfully. ** \n\r");
	
//	timeB=HAL_GetTick();
//	
//	for(unsigned int i=0;i<60000;i++ )
//	    cc=cc*aa;

//	timeA=HAL_GetTick();
//	timeInterval=timeA-timeB;
	
  for(int i=0;i<11;i++)
	{
     Gamma[i]=1/0.2;	
		 paraVector[i]=0;
	}
//	 Gamma[4]=0.2;
//	 Gamma[5]=0.2;
//	 Gamma[6]=0.2;
	 Gamma[7]=1;
	 Gamma[8]=1;
	 Gamma[9]=1;
	 Gamma[10]=1;
	 
sensorHipPositiont1=-PI/2,sensorHipPositiont2=-PI/2,sensorHipVelocity=0;
sensorKneePositiont1=0,sensorKneePositiont2=0,sensorKneeVelocity=0;
controlTime=0;
	  for(int i=0;i<3;i++)
	{
     aHip[i]=0;
     aKnee[i]=0;
	}
	
/* RLS Initialization */ 
	for(int i=0;i<4;i++)
	   for(int j=0;j<4;j++)
	{
	        if(i==j)
	           P_N0[i][j]=rlsConst*rlsConst;
	        else
						 P_N0[i][j]=0;
	}
	
	

/* END */
	

  CAN1_Filter_Init();
  HAL_TIM_Base_Start_IT(&htim2);
	HAL_TIM_Base_Start_IT(&htim3);
	
	if(HAL_UART_Receive_IT(&huart2,aRxBuffer,1)==HAL_OK)
{
	positionLow[0]=0;
	
}


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
		timeB=HAL_GetTick();
		
		
	
	HAL_Delay(1);
				
	sensorKneePositiont1=sensorKneePositiont2;
	timeA=HAL_GetTick();
	timeInterval=timeA-timeB;
	timeStep=(float)timeInterval/1000;
							
  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct;

  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 192;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  HAL_PWREx_EnableOverDrive();

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV4;
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_6);

  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USART2;
  PeriphClkInitStruct.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct);

  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* ADC1 init function */
void MX_ADC1_Init(void)
{

  ADC_ChannelConfTypeDef sConfig;

    /**Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
    */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  HAL_ADC_Init(&hadc1);

    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
    */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  HAL_ADC_ConfigChannel(&hadc1, &sConfig);

}

/* CAN1 init function */
void MX_CAN1_Init(void)
{

  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 6;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SJW = CAN_SJW_1TQ;
  hcan1.Init.BS1 = CAN_BS1_5TQ;
  hcan1.Init.BS2 = CAN_BS2_2TQ;
  hcan1.Init.TTCM = DISABLE;
  hcan1.Init.ABOM = DISABLE;
  hcan1.Init.AWUM = DISABLE;
  hcan1.Init.NART = DISABLE;
  hcan1.Init.RFLM = DISABLE;
  hcan1.Init.TXFP = DISABLE;
  HAL_CAN_Init(&hcan1);

}

/* TIM2 init function */
void MX_TIM2_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 999;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 95;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  HAL_TIM_Base_Init(&htim2);

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig);

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig);

}

/* TIM3 init function */
void MX_TIM3_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 999;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 950;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  HAL_TIM_Base_Init(&htim3);

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig);

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig);

}

/* USART2 init function */
void MX_USART2_UART_Init(void)
{

  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  HAL_UART_Init(&huart2);

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LD1_Pin|LD3_Pin|LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : LD1_Pin LD3_Pin LD2_Pin */
  GPIO_InitStruct.Pin = LD1_Pin|LD3_Pin|LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

void CAN1_Filter_Init(void)
{
	//32-Bit Filter;
	//     0000 0000 (000)0 0000 0000 0000 0000 0000,括号内是STID的低三位
	//Mask 0000 0000 (111)0 0000 0000 0000 0000 0000
	CAN_FilterConfTypeDef sFilterConfig;
	sFilterConfig.FilterNumber=0;
	sFilterConfig.FilterMode=CAN_FILTERMODE_IDMASK;
	sFilterConfig.FilterScale=CAN_FILTERSCALE_32BIT;
  sFilterConfig.FilterIdHigh=0x0000;
  sFilterConfig.FilterIdLow=0x0000;
  sFilterConfig.FilterMaskIdHigh=0x0000;
  sFilterConfig.FilterMaskIdLow=0x0000;
  sFilterConfig.FilterFIFOAssignment=CAN_FIFO0;
	sFilterConfig.FilterActivation=ENABLE;
  HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig);

}

void Servo_Init(void)
{
		//????
	pTxMsg.StdId=0x00;
  pTxMsg.IDE =CAN_ID_STD;
	pTxMsg.RTR =CAN_RTR_DATA;
	pTxMsg.DLC =8;
	pTxMsg.Data[0]=0x01;
	pTxMsg.Data[1]=0x00;
	Timeout=10;
	hcan1.pTxMsg=&pTxMsg;
	HAL_CAN_Transmit(&hcan1,10);
	
	
	//取消181的功能
	pTxMsg.StdId=0x601;
  pTxMsg.IDE =CAN_ID_STD;
	pTxMsg.RTR =CAN_RTR_DATA;
	pTxMsg.DLC =8;
	pTxMsg.Data[0]=0x23;
	pTxMsg.Data[1]=0x00;
	pTxMsg.Data[2]=0x18;
	pTxMsg.Data[3]=0x01;
	pTxMsg.Data[4]=0x81;
	pTxMsg.Data[5]=0x01;
	pTxMsg.Data[6]=0x00;
	pTxMsg.Data[7]=0x80;
	hcan1.pTxMsg=&pTxMsg;
	HAL_CAN_Transmit(&hcan1,10);
	HAL_CAN_Receive(&hcan1,CAN_FIFO0,10);
	
	//取消281的功能
	pTxMsg.StdId=0x601;
  pTxMsg.IDE =CAN_ID_STD;
	pTxMsg.RTR =CAN_RTR_DATA;
	pTxMsg.DLC =8;
	pTxMsg.Data[0]=0x23;
	pTxMsg.Data[1]=0x01;
	pTxMsg.Data[2]=0x18;
	pTxMsg.Data[3]=0x01;
	pTxMsg.Data[4]=0x81;
	pTxMsg.Data[5]=0x02;
	pTxMsg.Data[6]=0x00;
	pTxMsg.Data[7]=0x80;
	hcan1.pTxMsg=&pTxMsg;
	HAL_CAN_Transmit(&hcan1,10);
	HAL_CAN_Receive(&hcan1,CAN_FIFO0,10);
	
	
	//取消182的功能
	pTxMsg.StdId=0x602;
  pTxMsg.IDE =CAN_ID_STD;
	pTxMsg.RTR =CAN_RTR_DATA;
	pTxMsg.DLC =8;
	pTxMsg.Data[0]=0x23;
	pTxMsg.Data[1]=0x00;
	pTxMsg.Data[2]=0x18;
	pTxMsg.Data[3]=0x01;
	pTxMsg.Data[4]=0x82;
	pTxMsg.Data[5]=0x01;
	pTxMsg.Data[6]=0x00;
	pTxMsg.Data[7]=0x80;
	hcan1.pTxMsg=&pTxMsg;
	HAL_CAN_Transmit(&hcan1,10);
	HAL_CAN_Receive(&hcan1,CAN_FIFO0,10);
	
	//取消282的功能
	pTxMsg.StdId=0x602;
  pTxMsg.IDE =CAN_ID_STD;
	pTxMsg.RTR =CAN_RTR_DATA;
	pTxMsg.DLC =8;
	pTxMsg.Data[0]=0x23;
	pTxMsg.Data[1]=0x01;
	pTxMsg.Data[2]=0x18;
	pTxMsg.Data[3]=0x01;
	pTxMsg.Data[4]=0x82;
	pTxMsg.Data[5]=0x02;
	pTxMsg.Data[6]=0x00;
	pTxMsg.Data[7]=0x80;
	hcan1.pTxMsg=&pTxMsg;
	HAL_CAN_Transmit(&hcan1,10);
	HAL_CAN_Receive(&hcan1,CAN_FIFO0,10);	
  	
		
		
	//下伺服
	pTxMsg.StdId=0x202;
  pTxMsg.IDE =CAN_ID_STD;
	pTxMsg.RTR =CAN_RTR_DATA;
	pTxMsg.DLC =2;
	pTxMsg.Data[0]=0x05;
	pTxMsg.Data[1]=0x00;
	hcan1.pTxMsg=&pTxMsg;
	HAL_CAN_Transmit(&hcan1,10);
	////?????

	pTxMsg.StdId=0x201;
  pTxMsg.IDE =CAN_ID_STD;
	pTxMsg.RTR =CAN_RTR_DATA;
	pTxMsg.DLC =2;
	pTxMsg.Data[0]=0x05;
	pTxMsg.Data[1]=0x00;
	hcan1.pTxMsg=&pTxMsg;
	HAL_CAN_Transmit(&hcan1,10);
	
  //读编码器PDO
	pTxMsg.StdId=0x601;
  pTxMsg.IDE =CAN_ID_STD;
	pTxMsg.RTR =CAN_RTR_DATA;
	pTxMsg.DLC =8;
	pTxMsg.Data[0]=0x23;
	pTxMsg.Data[1]=0x02;
	pTxMsg.Data[2]=0x18;
	pTxMsg.Data[3]=0x01;
	pTxMsg.Data[4]=0x81;
	pTxMsg.Data[5]=0x03;
	pTxMsg.Data[6]=0x00;
	pTxMsg.Data[7]=0x80;
	Timeout=10;
	hcan1.pTxMsg=&pTxMsg;
	HAL_CAN_Transmit(&hcan1,10);
	HAL_CAN_Receive(&hcan1,CAN_FIFO0,10);
	
	pTxMsg.StdId=0x601;
  pTxMsg.IDE =CAN_ID_STD;
	pTxMsg.RTR =CAN_RTR_DATA;
	pTxMsg.DLC =5;
	pTxMsg.Data[0]=0x2F;
	pTxMsg.Data[1]=0x02;
	pTxMsg.Data[2]=0x18;
	pTxMsg.Data[3]=0x02;
	pTxMsg.Data[4]=0xFF;
	Timeout=10;
	hcan1.pTxMsg=&pTxMsg;
	HAL_CAN_Transmit(&hcan1,10);
	HAL_CAN_Receive(&hcan1,CAN_FIFO0,10);

	pTxMsg.StdId=0x601;
  pTxMsg.IDE =CAN_ID_STD;
	pTxMsg.RTR =CAN_RTR_DATA;
	pTxMsg.DLC =5;
	pTxMsg.Data[0]=0x2F;
	pTxMsg.Data[1]=0x02;
	pTxMsg.Data[2]=0x1A;
	pTxMsg.Data[3]=0x00;
	pTxMsg.Data[4]=0x00;
	Timeout=10;
	hcan1.pTxMsg=&pTxMsg;
	HAL_CAN_Transmit(&hcan1,10);
	HAL_CAN_Receive(&hcan1,CAN_FIFO0,10);

	pTxMsg.StdId=0x601;
  pTxMsg.IDE =CAN_ID_STD;
	pTxMsg.RTR =CAN_RTR_DATA;
	pTxMsg.DLC =8;
	pTxMsg.Data[0]=0x23;
	pTxMsg.Data[1]=0x02;
	pTxMsg.Data[2]=0x1A;
	pTxMsg.Data[3]=0x01;
	pTxMsg.Data[4]=0x20;
	pTxMsg.Data[5]=0x00;
	pTxMsg.Data[6]=0x64;
	pTxMsg.Data[7]=0x60;
	Timeout=10;
	hcan1.pTxMsg=&pTxMsg;
	HAL_CAN_Transmit(&hcan1,10);
	HAL_CAN_Receive(&hcan1,CAN_FIFO0,10);

	pTxMsg.StdId=0x601;
  pTxMsg.IDE =CAN_ID_STD;
	pTxMsg.RTR =CAN_RTR_DATA;
	pTxMsg.DLC =5;
	pTxMsg.Data[0]=0x2F;
	pTxMsg.Data[1]=0x02;
	pTxMsg.Data[2]=0x1A;
	pTxMsg.Data[3]=0x00;
	pTxMsg.Data[4]=0x01;
	Timeout=10;
	hcan1.pTxMsg=&pTxMsg;
	HAL_CAN_Transmit(&hcan1,10);
	HAL_CAN_Receive(&hcan1,CAN_FIFO0,10);

	pTxMsg.StdId=0x601;
  pTxMsg.IDE =CAN_ID_STD;
	pTxMsg.RTR =CAN_RTR_DATA;
	pTxMsg.DLC =8;
	pTxMsg.Data[0]=0x23;
	pTxMsg.Data[1]=0x02;
	pTxMsg.Data[2]=0x18;
	pTxMsg.Data[3]=0x01;
	pTxMsg.Data[4]=0x81;
	pTxMsg.Data[5]=0x03;
	pTxMsg.Data[6]=0x00;
	pTxMsg.Data[7]=0x00;
	Timeout=10;
	hcan1.pTxMsg=&pTxMsg;
	HAL_CAN_Transmit(&hcan1,10);
	HAL_CAN_Receive(&hcan1,CAN_FIFO0,10);
	
		
  //力矩PDO
	pTxMsg.StdId=0x601;
  pTxMsg.IDE =CAN_ID_STD;
	pTxMsg.RTR =CAN_RTR_DATA;
	pTxMsg.DLC =8;
	pTxMsg.Data[0]=0x23;
	pTxMsg.Data[1]=0x01;
	pTxMsg.Data[2]=0x14;
	pTxMsg.Data[3]=0x01;
	pTxMsg.Data[4]=0x01;
	pTxMsg.Data[5]=0x03;
	pTxMsg.Data[6]=0x00;
	pTxMsg.Data[7]=0x80;
	Timeout=10;
	hcan1.pTxMsg=&pTxMsg;
	HAL_CAN_Transmit(&hcan1,10);
	HAL_CAN_Receive(&hcan1,CAN_FIFO0,10);
	
	pTxMsg.StdId=0x601;
  pTxMsg.IDE =CAN_ID_STD;
	pTxMsg.RTR =CAN_RTR_DATA;
	pTxMsg.DLC =5;
	pTxMsg.Data[0]=0x2F;
	pTxMsg.Data[1]=0x01;
	pTxMsg.Data[2]=0x14;
	pTxMsg.Data[3]=0x02;
	pTxMsg.Data[4]=0xFF;
	Timeout=10;
	hcan1.pTxMsg=&pTxMsg;
	HAL_CAN_Transmit(&hcan1,10);
	HAL_CAN_Receive(&hcan1,CAN_FIFO0,10);

	pTxMsg.StdId=0x601;
  pTxMsg.IDE =CAN_ID_STD;
	pTxMsg.RTR =CAN_RTR_DATA;
	pTxMsg.DLC =5;
	pTxMsg.Data[0]=0x2F;
	pTxMsg.Data[1]=0x01;
	pTxMsg.Data[2]=0x16;
	pTxMsg.Data[3]=0x00;
	pTxMsg.Data[4]=0x00;
	Timeout=10;
	hcan1.pTxMsg=&pTxMsg;
	HAL_CAN_Transmit(&hcan1,10);
	HAL_CAN_Receive(&hcan1,CAN_FIFO0,10);

	pTxMsg.StdId=0x601;
  pTxMsg.IDE =CAN_ID_STD;
	pTxMsg.RTR =CAN_RTR_DATA;
	pTxMsg.DLC =8;
	pTxMsg.Data[0]=0x23;
	pTxMsg.Data[1]=0x01;
	pTxMsg.Data[2]=0x16;
	pTxMsg.Data[3]=0x01;
	pTxMsg.Data[4]=0x10;
	pTxMsg.Data[5]=0x00;
	pTxMsg.Data[6]=0x71;
	pTxMsg.Data[7]=0x60;
	Timeout=10;
	hcan1.pTxMsg=&pTxMsg;
	HAL_CAN_Transmit(&hcan1,10);
	HAL_CAN_Receive(&hcan1,CAN_FIFO0,10);

	pTxMsg.StdId=0x601;
  pTxMsg.IDE =CAN_ID_STD;
	pTxMsg.RTR =CAN_RTR_DATA;
	pTxMsg.DLC =5;
	pTxMsg.Data[0]=0x2F;
	pTxMsg.Data[1]=0x01;
	pTxMsg.Data[2]=0x16;
	pTxMsg.Data[3]=0x00;
	pTxMsg.Data[4]=0x01;
	Timeout=10;
	hcan1.pTxMsg=&pTxMsg;
	HAL_CAN_Transmit(&hcan1,10);
	HAL_CAN_Receive(&hcan1,CAN_FIFO0,10);

	pTxMsg.StdId=0x601;
  pTxMsg.IDE =CAN_ID_STD;
	pTxMsg.RTR =CAN_RTR_DATA;
	pTxMsg.DLC =8;
	pTxMsg.Data[0]=0x23;
	pTxMsg.Data[1]=0x01;
	pTxMsg.Data[2]=0x14;
	pTxMsg.Data[3]=0x01;
	pTxMsg.Data[4]=0x01;
	pTxMsg.Data[5]=0x03;
	pTxMsg.Data[6]=0x00;
	pTxMsg.Data[7]=0x00;
	Timeout=10;
	hcan1.pTxMsg=&pTxMsg;
	HAL_CAN_Transmit(&hcan1,10);
	HAL_CAN_Receive(&hcan1,CAN_FIFO0,10);
	
	


  //设置电流模式
	pTxMsg.StdId=0x601;
  pTxMsg.IDE =CAN_ID_STD;
	pTxMsg.RTR =CAN_RTR_DATA;
	pTxMsg.DLC =5;
	pTxMsg.Data[0]=0x2F;
	pTxMsg.Data[1]=0x60;
	pTxMsg.Data[2]=0x60;
	pTxMsg.Data[3]=0x00;
	pTxMsg.Data[4]=0x04;
	Timeout=10;
	hcan1.pTxMsg=&pTxMsg;
	HAL_CAN_Transmit(&hcan1,10);
	//HAL_CAN_Receive(&hcan1,CAN_FIFO0,10);
	


  //编码器位置归零
  pTxMsg.StdId=0x601;
  pTxMsg.IDE =CAN_ID_STD;
	pTxMsg.RTR =CAN_RTR_DATA;
	pTxMsg.DLC =8;
	pTxMsg.Data[0]=0x23;
	pTxMsg.Data[1]=0x64;
	pTxMsg.Data[2]=0x60;
	pTxMsg.Data[3]=0x00;
	pTxMsg.Data[4]=0x00;
	pTxMsg.Data[5]=0x00;
	pTxMsg.Data[6]=0x00;
	pTxMsg.Data[7]=0x00;
	Timeout=10;
	hcan1.pTxMsg=&pTxMsg;
	HAL_CAN_Transmit(&hcan1,10);
	HAL_CAN_Receive(&hcan1,CAN_FIFO0,10);



//????????
	pTxMsg1.StdId=0x381;
  pTxMsg1.IDE =CAN_ID_STD;
	pTxMsg1.RTR =CAN_RTR_REMOTE;
	pTxMsg1.DLC =0;
	Timeout=10;
	hcan1.pTxMsg=&pTxMsg1;
	HAL_CAN_Transmit(&hcan1,10);
	HAL_CAN_Receive(&hcan1,CAN_FIFO0,10);
	pRxMsg1=*(hcan1.pRxMsg);
	

	pTxMsg.StdId=0x301;
  pTxMsg.IDE =CAN_ID_STD;
	pTxMsg.RTR =CAN_RTR_DATA;
	pTxMsg.DLC =2;
	pTxMsg.Data[0]=00;
	pTxMsg.Data[1]=00;
	hcan1.pTxMsg=&pTxMsg;
	HAL_CAN_Transmit(&hcan1,10);
	
	
	
  //设置读取编码器PDO
	pTxMsg.StdId=0x602;
  pTxMsg.IDE =CAN_ID_STD;
	pTxMsg.RTR =CAN_RTR_DATA;
	pTxMsg.DLC =8;
	pTxMsg.Data[0]=0x23;
	pTxMsg.Data[1]=0x02;
	pTxMsg.Data[2]=0x18;
	pTxMsg.Data[3]=0x01;
	pTxMsg.Data[4]=0x82;
	pTxMsg.Data[5]=0x03;
	pTxMsg.Data[6]=0x00;
	pTxMsg.Data[7]=0x80;
	Timeout=10;
	hcan1.pTxMsg=&pTxMsg;
	HAL_CAN_Transmit(&hcan1,10);
	HAL_CAN_Receive(&hcan1,CAN_FIFO0,10);
	
	pTxMsg.StdId=0x602;
  pTxMsg.IDE =CAN_ID_STD;
	pTxMsg.RTR =CAN_RTR_DATA;
	pTxMsg.DLC =5;
	pTxMsg.Data[0]=0x2F;
	pTxMsg.Data[1]=0x02;
	pTxMsg.Data[2]=0x18;
	pTxMsg.Data[3]=0x02;
	pTxMsg.Data[4]=0xFF;
	Timeout=10;
	hcan1.pTxMsg=&pTxMsg;
	HAL_CAN_Transmit(&hcan1,10);
	HAL_CAN_Receive(&hcan1,CAN_FIFO0,10);

	pTxMsg.StdId=0x602;
  pTxMsg.IDE =CAN_ID_STD;
	pTxMsg.RTR =CAN_RTR_DATA;
	pTxMsg.DLC =5;
	pTxMsg.Data[0]=0x2F;
	pTxMsg.Data[1]=0x02;
	pTxMsg.Data[2]=0x1A;
	pTxMsg.Data[3]=0x00;
	pTxMsg.Data[4]=0x00;
	Timeout=10;
	hcan1.pTxMsg=&pTxMsg;
	HAL_CAN_Transmit(&hcan1,10);
	HAL_CAN_Receive(&hcan1,CAN_FIFO0,10);

	pTxMsg.StdId=0x602;
  pTxMsg.IDE =CAN_ID_STD;
	pTxMsg.RTR =CAN_RTR_DATA;
	pTxMsg.DLC =8;
	pTxMsg.Data[0]=0x23;
	pTxMsg.Data[1]=0x02;
	pTxMsg.Data[2]=0x1A;
	pTxMsg.Data[3]=0x01;
	pTxMsg.Data[4]=0x20;
	pTxMsg.Data[5]=0x00;
	pTxMsg.Data[6]=0x64;
	pTxMsg.Data[7]=0x60;
	Timeout=10;
	hcan1.pTxMsg=&pTxMsg;
	HAL_CAN_Transmit(&hcan1,10);
	HAL_CAN_Receive(&hcan1,CAN_FIFO0,10);

	pTxMsg.StdId=0x602;
  pTxMsg.IDE =CAN_ID_STD;
	pTxMsg.RTR =CAN_RTR_DATA;
	pTxMsg.DLC =5;
	pTxMsg.Data[0]=0x2F;
	pTxMsg.Data[1]=0x02;
	pTxMsg.Data[2]=0x1A;
	pTxMsg.Data[3]=0x00;
	pTxMsg.Data[4]=0x01;
	Timeout=10;
	hcan1.pTxMsg=&pTxMsg;
	HAL_CAN_Transmit(&hcan1,10);
	HAL_CAN_Receive(&hcan1,CAN_FIFO0,10);

	pTxMsg.StdId=0x602;
  pTxMsg.IDE =CAN_ID_STD;
	pTxMsg.RTR =CAN_RTR_DATA;
	pTxMsg.DLC =8;
	pTxMsg.Data[0]=0x23;
	pTxMsg.Data[1]=0x02;
	pTxMsg.Data[2]=0x18;
	pTxMsg.Data[3]=0x01;
	pTxMsg.Data[4]=0x82;
	pTxMsg.Data[5]=0x03;
	pTxMsg.Data[6]=0x00;
	pTxMsg.Data[7]=0x00;
	Timeout=10;
	hcan1.pTxMsg=&pTxMsg;
	HAL_CAN_Transmit(&hcan1,10);
	HAL_CAN_Receive(&hcan1,CAN_FIFO0,10);
	
		
  //设置力矩PDO
	pTxMsg.StdId=0x602;
  pTxMsg.IDE =CAN_ID_STD;
	pTxMsg.RTR =CAN_RTR_DATA;
	pTxMsg.DLC =8;
	pTxMsg.Data[0]=0x23;
	pTxMsg.Data[1]=0x01;
	pTxMsg.Data[2]=0x14;
	pTxMsg.Data[3]=0x01;
	pTxMsg.Data[4]=0x02;
	pTxMsg.Data[5]=0x03;
	pTxMsg.Data[6]=0x00;
	pTxMsg.Data[7]=0x80;
	Timeout=10;
	hcan1.pTxMsg=&pTxMsg;
	HAL_CAN_Transmit(&hcan1,10);
	HAL_CAN_Receive(&hcan1,CAN_FIFO0,10);
	
	pTxMsg.StdId=0x602;
  pTxMsg.IDE =CAN_ID_STD;
	pTxMsg.RTR =CAN_RTR_DATA;
	pTxMsg.DLC =5;
	pTxMsg.Data[0]=0x2F;
	pTxMsg.Data[1]=0x01;
	pTxMsg.Data[2]=0x14;
	pTxMsg.Data[3]=0x02;
	pTxMsg.Data[4]=0xFF;
	Timeout=10;
	hcan1.pTxMsg=&pTxMsg;
	HAL_CAN_Transmit(&hcan1,10);
	HAL_CAN_Receive(&hcan1,CAN_FIFO0,10);

	pTxMsg.StdId=0x602;
  pTxMsg.IDE =CAN_ID_STD;
	pTxMsg.RTR =CAN_RTR_DATA;
	pTxMsg.DLC =5;
	pTxMsg.Data[0]=0x2F;
	pTxMsg.Data[1]=0x01;
	pTxMsg.Data[2]=0x16;
	pTxMsg.Data[3]=0x00;
	pTxMsg.Data[4]=0x00;
	Timeout=10;
	hcan1.pTxMsg=&pTxMsg;
	HAL_CAN_Transmit(&hcan1,10);
	HAL_CAN_Receive(&hcan1,CAN_FIFO0,10);

	pTxMsg.StdId=0x602;
  pTxMsg.IDE =CAN_ID_STD;
	pTxMsg.RTR =CAN_RTR_DATA;
	pTxMsg.DLC =8;
	pTxMsg.Data[0]=0x23;
	pTxMsg.Data[1]=0x01;
	pTxMsg.Data[2]=0x16;
	pTxMsg.Data[3]=0x01;
	pTxMsg.Data[4]=0x10;
	pTxMsg.Data[5]=0x00;
	pTxMsg.Data[6]=0x71;
	pTxMsg.Data[7]=0x60;
	Timeout=10;
	hcan1.pTxMsg=&pTxMsg;
	HAL_CAN_Transmit(&hcan1,10);
	HAL_CAN_Receive(&hcan1,CAN_FIFO0,10);

	pTxMsg.StdId=0x602;
  pTxMsg.IDE =CAN_ID_STD;
	pTxMsg.RTR =CAN_RTR_DATA;
	pTxMsg.DLC =5;
	pTxMsg.Data[0]=0x2F;
	pTxMsg.Data[1]=0x01;
	pTxMsg.Data[2]=0x16;
	pTxMsg.Data[3]=0x00;
	pTxMsg.Data[4]=0x01;
	Timeout=10;
	hcan1.pTxMsg=&pTxMsg;
	HAL_CAN_Transmit(&hcan1,10);
	HAL_CAN_Receive(&hcan1,CAN_FIFO0,10);

	pTxMsg.StdId=0x602;
  pTxMsg.IDE =CAN_ID_STD;
	pTxMsg.RTR =CAN_RTR_DATA;
	pTxMsg.DLC =8;
	pTxMsg.Data[0]=0x23;
	pTxMsg.Data[1]=0x01;
	pTxMsg.Data[2]=0x14;
	pTxMsg.Data[3]=0x01;
	pTxMsg.Data[4]=0x02;
	pTxMsg.Data[5]=0x03;
	pTxMsg.Data[6]=0x00;
	pTxMsg.Data[7]=0x00;
	Timeout=10;
	hcan1.pTxMsg=&pTxMsg;
	HAL_CAN_Transmit(&hcan1,10);
	HAL_CAN_Receive(&hcan1,CAN_FIFO0,10);
		

  //设置电流模式
	pTxMsg.StdId=0x602;
  pTxMsg.IDE =CAN_ID_STD;
	pTxMsg.RTR =CAN_RTR_DATA;
	pTxMsg.DLC =5;
	pTxMsg.Data[0]=0x2F;
	pTxMsg.Data[1]=0x60;
	pTxMsg.Data[2]=0x60;
	pTxMsg.Data[3]=0x00;
	pTxMsg.Data[4]=0x04;
	Timeout=10;
	hcan1.pTxMsg=&pTxMsg;
	HAL_CAN_Transmit(&hcan1,10);
	//HAL_CAN_Receive(&hcan1,CAN_FIFO0,10);
	


  //编码器位置归零
  pTxMsg.StdId=0x602;
  pTxMsg.IDE =CAN_ID_STD;
	pTxMsg.RTR =CAN_RTR_DATA;
	pTxMsg.DLC =8;
	pTxMsg.Data[0]=0x23;
	pTxMsg.Data[1]=0x64;
	pTxMsg.Data[2]=0x60;
	pTxMsg.Data[3]=0x00;
	pTxMsg.Data[4]=0x00;
	pTxMsg.Data[5]=0x00;
	pTxMsg.Data[6]=0x00;
	pTxMsg.Data[7]=0x00;
	Timeout=10;
	hcan1.pTxMsg=&pTxMsg;
	HAL_CAN_Transmit(&hcan1,10);
	HAL_CAN_Receive(&hcan1,CAN_FIFO0,10);



  //读取编码器
	pTxMsg1.StdId=0x382;
  pTxMsg1.IDE =CAN_ID_STD;
	pTxMsg1.RTR =CAN_RTR_REMOTE;
	pTxMsg1.DLC =0;
	Timeout=10;
	hcan1.pTxMsg=&pTxMsg1;
	HAL_CAN_Transmit(&hcan1,10);
	HAL_CAN_Receive(&hcan1,CAN_FIFO0,10);
	pRxMsg1=*(hcan1.pRxMsg);
	

	pTxMsg.StdId=0x302;
  pTxMsg.IDE =CAN_ID_STD;
	pTxMsg.RTR =CAN_RTR_DATA;
	pTxMsg.DLC =2;
	pTxMsg.Data[0]=00;
	pTxMsg.Data[1]=00;
	hcan1.pTxMsg=&pTxMsg;
	HAL_CAN_Transmit(&hcan1,10);
	
	
						
}
/* USER CODE END 4 */

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
