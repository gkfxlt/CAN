#include "stm32f7xx_hal.h"

/* USER CODE BEGIN Includes */
#include "stdio.h"
#include <math.h>
#include "string.h"
#include "stm32f7xx_it.h"

extern uint32_t timeB,timeA,timeInterval;
extern uint32_t  transmitmailbox;
extern uint32_t Timeout;
extern CanTxMsgTypeDef            pTxMsg;
extern CanRxMsgTypeDef            pRxMsg;
extern CanRxMsgTypeDef            pRxMsg1;


extern double c,a;
extern float KP,KI;

extern float controlTime;
extern float positionError,positionErrorIntegral;
extern int voltageInput;

extern uint8_t positionLow[1];
extern uint8_t positionLow1[1];
extern uint8_t positionLow2[1];
extern uint8_t positionHigh[1];

extern int sensorCounter,tempCounter;
extern int sensorZero;
extern UART_HandleTypeDef huart2;
extern CAN_HandleTypeDef hcan1;

