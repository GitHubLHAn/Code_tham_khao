#include "stm32f1xx_hal.h"
#include "main.h"
/**********************************************************************************************/
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(GPIO_Pin == EXTI2_Fault_Pin)
	{
		//stMControl_Var.mState = mFAULT;
		//stMControl_Var.EXT_State = EXT_FAULT;
	}
}
/**********************************************************************************************/
void vDI_Scan(void)	//Scan the proximity sensors in operation, every 10ms
{
	//Read all DI
	DI_Objt[0].dataK = HAL_GPIO_ReadPin(DI0_GPIO_Port, DI0_Pin);
	DI_Objt[1].dataK = HAL_GPIO_ReadPin(DI1_GPIO_Port, DI1_Pin);
	DI_Objt[2].dataK = HAL_GPIO_ReadPin(DI2_GPIO_Port, DI2_Pin);
	DI_Objt[3].dataK = HAL_GPIO_ReadPin(DI3_GPIO_Port, DI3_Pin);
	DI_Objt[4].dataK = HAL_GPIO_ReadPin(DI4_GPIO_Port, DI4_Pin);
	DI_Objt[5].dataK = HAL_GPIO_ReadPin(DI5_GPIO_Port, DI5_Pin);
	DI_Objt[6].dataK = HAL_GPIO_ReadPin(DI6_GPIO_Port, DI6_Pin);
	DI_Objt[7].dataK = HAL_GPIO_ReadPin(EXTI2_Fault_GPIO_Port, EXTI2_Fault_Pin);
		
	//Event detect
	DI_Objt[0].event = eDI_EventDetect(DI_Objt[0].dataK, DI_Objt[0].dataK_1);
	DI_Objt[1].event = eDI_EventDetect(DI_Objt[1].dataK, DI_Objt[1].dataK_1);
	DI_Objt[2].event = eDI_EventDetect(DI_Objt[2].dataK, DI_Objt[2].dataK_1);
	DI_Objt[3].event = eDI_EventDetect(DI_Objt[3].dataK, DI_Objt[3].dataK_1);
	DI_Objt[4].event = eDI_EventDetect(DI_Objt[4].dataK, DI_Objt[4].dataK_1);
	DI_Objt[5].event = eDI_EventDetect(DI_Objt[5].dataK, DI_Objt[5].dataK_1);
	DI_Objt[6].event = eDI_EventDetect(DI_Objt[6].dataK, DI_Objt[6].dataK_1);
	DI_Objt[7].event = eDI_EventDetect(DI_Objt[7].dataK, DI_Objt[7].dataK_1);
	
	//Update DI
	DI_Objt[0].dataK_1 = DI_Objt[0].dataK;
	DI_Objt[1].dataK_1 = DI_Objt[1].dataK;
	DI_Objt[2].dataK_1 = DI_Objt[2].dataK;
	DI_Objt[3].dataK_1 = DI_Objt[3].dataK;
	DI_Objt[4].dataK_1 = DI_Objt[4].dataK;
	DI_Objt[5].dataK_1 = DI_Objt[5].dataK;
	DI_Objt[6].dataK_1 = DI_Objt[6].dataK;
	DI_Objt[7].dataK_1 = DI_Objt[7].dataK;
}
/**********************************************************************************************/
void vDIO_Init(void)	//Intialize DIO at start-up
{
	//Read all DI
	DI_Objt[0].dataK = HAL_GPIO_ReadPin(DI0_GPIO_Port, DI0_Pin);
	DI_Objt[1].dataK = HAL_GPIO_ReadPin(DI1_GPIO_Port, DI1_Pin);
	DI_Objt[2].dataK = HAL_GPIO_ReadPin(DI2_GPIO_Port, DI2_Pin);
	DI_Objt[3].dataK = HAL_GPIO_ReadPin(DI3_GPIO_Port, DI3_Pin);
	DI_Objt[4].dataK = HAL_GPIO_ReadPin(DI4_GPIO_Port, DI4_Pin);
	DI_Objt[5].dataK = HAL_GPIO_ReadPin(DI5_GPIO_Port, DI5_Pin);
	DI_Objt[6].dataK = HAL_GPIO_ReadPin(DI6_GPIO_Port, DI6_Pin);
	DI_Objt[7].dataK = HAL_GPIO_ReadPin(EXTI2_Fault_GPIO_Port, EXTI2_Fault_Pin);
	//Update DI
	DI_Objt[0].dataK_1 = DI_Objt[0].dataK;
	DI_Objt[1].dataK_1 = DI_Objt[1].dataK;
	DI_Objt[2].dataK_1 = DI_Objt[2].dataK;
	DI_Objt[3].dataK_1 = DI_Objt[3].dataK;
	DI_Objt[4].dataK_1 = DI_Objt[4].dataK;
	DI_Objt[5].dataK_1 = DI_Objt[5].dataK;
	DI_Objt[6].dataK_1 = DI_Objt[6].dataK;
	DI_Objt[7].dataK_1 = DI_Objt[7].dataK;
	//Reset all DO
	HAL_GPIO_WritePin(DO0_GPIO_Port, DO0_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(DO1_GPIO_Port, DO1_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(DO2_GPIO_Port, DO2_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(DO3_GPIO_Port, DO3_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(DO4_GPIO_Port, DO4_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(Bypass_GPIO_Port, Bypass_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(Reset_GPIO_Port, Reset_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(Enable_GPIO_Port, Enable_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(D_Braking_GPIO_Port, D_Braking_Pin, GPIO_PIN_RESET);
}
/**********************************************************************************************/
gpio_Event eDI_EventDetect(GPIO_PinState k_Instance, GPIO_PinState k_1_Instance) //Check the event
{
	gpio_Event temp;
	if(k_Instance == GPIO_PIN_SET && k_1_Instance == GPIO_PIN_SET)
		temp = High;
	else if(k_Instance == GPIO_PIN_SET && k_1_Instance  == GPIO_PIN_RESET)
		temp = Rising;
	else if(k_Instance == GPIO_PIN_RESET && k_1_Instance == GPIO_PIN_SET)
		temp = Falling;
	else
		temp = Low;
	return temp;
}
/**********************************************************************************************/
void GPIO_Test(void) 	//For test only
{
	uint16_t i = 0;
	for(i = 0;i<NumOfDI;i++)
	{ 
		if(DI_Objt[i].event ==Low)
		{
			HAL_GPIO_WritePin(DO0_GPIO_Port, DO0_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(DO1_GPIO_Port, DO1_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(DO2_GPIO_Port, DO2_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(DO3_GPIO_Port, DO3_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(DO4_GPIO_Port, DO4_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(Bypass_GPIO_Port, Bypass_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(Reset_GPIO_Port, Reset_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(Enable_GPIO_Port, Enable_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(D_Braking_GPIO_Port, D_Braking_Pin, GPIO_PIN_SET);
		}
	}
	if(DI_Objt[0].event == High && DI_Objt[1].event == High && DI_Objt[2].event == High && DI_Objt[3].event == High &&
			DI_Objt[4].event == High &&DI_Objt[5].event == High &&DI_Objt[6].event == High &&DI_Objt[7].event == High)
	{
		HAL_GPIO_WritePin(DO0_GPIO_Port, DO0_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(DO1_GPIO_Port, DO1_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(DO2_GPIO_Port, DO2_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(DO3_GPIO_Port, DO3_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(DO4_GPIO_Port, DO4_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(Bypass_GPIO_Port, Bypass_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(Reset_GPIO_Port, Reset_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(Enable_GPIO_Port, Enable_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(D_Braking_GPIO_Port, D_Braking_Pin, GPIO_PIN_RESET);
	}
}