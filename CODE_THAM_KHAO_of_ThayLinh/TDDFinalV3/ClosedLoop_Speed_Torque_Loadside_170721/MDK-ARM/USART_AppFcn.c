#include "main.h"
#include "math.h"

extern UART_HandleTypeDef huart4;
extern UART_HandleTypeDef huart5;
extern UART_HandleTypeDef huart2;

/*uart data*/
usartData usart2_Tx, usart2_Rx;						//ESP32 - Wifi
usartData usart4_Tx, usart4_Rx; 					//RS485
usartData usart5_Tx, usart5_Rx; 					//HMI

extern UART_HandleTypeDef huart2, huart4, huart5;
extern uint8_t First_Cycle_Flag;

/*Timer object*/
extern timer_Objt Tim_1ms[MaxTIMER];					//10ms software timer
extern timer_Objt Tim_10ms[MaxTIMER];				//10ms software timer
extern timer_Objt Tim_20ms[MaxTIMER];				//20ms software timer
extern timer_Objt Tim_100ms[MaxTIMER];				//100ms software timer
extern timer_Objt Tim_200ms[MaxTIMER];				//200ms software timer - for HMI Display
extern timer_Objt Tim_1s[MaxTIMER];					//1s software timer
extern uint16_t tim1msTick, tim10msTick, tim20msTick, tim100msTick, tim200msTick, tim1sTick;

/**********************************************************************************************/
void vUsart_DataInit(usartData *pData)
{
	uint8_t i = 0;
	pData->RxFlag = 0;
	pData->messCount = 0;
	pData->rxByte = 0;
	pData->rxPointer = 0;
	pData->STATE = USART_WAIT;
	pData->NEXTION_Flag = 0;
	pData->NEXTION_Pointer = 0;
	pData->NEXTION_RetCode = 0;
	for(i=0;i<10;i++)
	{
		pData->NEXTION_Buff[i] = 0;
	}
}
/**********************************************************************************************/
//NEXTION Command Handle
void vUsart5_rxHandle(usartData *pRxData, NEXTION *pNEXVar, machine_Control *pStateMachine)
{
	uint32_t byte1, byte2, byte3, byte4, temp; 
	
	//Disable HMI display control
	pNEXVar->UpdateEN = 0;
	
	//Handle data payload
	byte1 = pRxData->rxBuff[9];
	byte2 = pRxData->rxBuff[10];
	byte3 = pRxData->rxBuff[9];
	byte4 = pRxData->rxBuff[10];
	temp = (byte4<<24)|(byte3<<16)|(byte2<<8)|byte1;
	pRxData->messCount = temp;
	pNEXVar->messCnt = temp;
	
	byte1 = pRxData->rxBuff[13];
	byte2 = pRxData->rxBuff[14];
	byte3 = pRxData->rxBuff[15];
	byte4 = pRxData->rxBuff[16];
	temp = (byte4<<24)|(byte3<<16)|(byte2<<8)|byte1;
	pNEXVar->rxVal = temp;
	pRxData->messType = pRxData->rxBuff[7];
	switch(pRxData->messType)
	{
		case 0x51:	//RUN in LOCAL Mode Only
		{
			HAL_GPIO_TogglePin(DO4_GPIO_Port, DO4_Pin);
			if(pStateMachine->sysENABLE == 1 && pStateMachine->sysLOCAL_REMOTE == LOCAL)
			{
				pStateMachine->mState = mRUN;
			}
			break;
		}
		case 0x52: //STOP in LOCAL Mode Only
		{
			HAL_GPIO_TogglePin(DO4_GPIO_Port, DO4_Pin);
			pStateMachine->mState = mSTOP;
			break;
		}
		case 0x53: //ENABLE_DISABLE
		{
			HAL_GPIO_TogglePin(DO4_GPIO_Port, DO4_Pin);
			/*pStateMachine->sysENABLE = pNEXVar->rxVal;
			if(pStateMachine->sysENABLE!=0 && pStateMachine->sysENABLE!=1)
			{
				pStateMachine->sysENABLE = 0; //Disable system if data is imcorrected
				stMControl_Var.mState = mSTOP;
				HAL_GPIO_WritePin(Enable_GPIO_Port, Enable_Pin, GPIO_PIN_RESET);
			}
			if(pStateMachine->sysENABLE == 0)
			{
				stMControl_Var.mState = mSTOP;
				
			}
			else
			{
				HAL_GPIO_WritePin(Enable_GPIO_Port, Enable_Pin, GPIO_PIN_SET);
			}
			break;*/
		}
		case 0x54: //TORQUE_SPEED - Switch Mode when system is mSTANDSTILL only
		{
			HAL_GPIO_TogglePin(DO4_GPIO_Port, DO4_Pin);
			if(pStateMachine->mState == mSTANDSTILL)
			{
				if(pNEXVar->rxVal == 0)
				{
					pStateMachine->ctrMode = OPEN_LOOP;
				}
				else if(pNEXVar->rxVal == 1)
				{
					pStateMachine->ctrMode = TORQUE;
					
				}
				else if(pNEXVar->rxVal == 2)
				{
					pStateMachine->ctrMode = SPEED;
					
				}
			}
			break;
		}
		case 0x55: //LOCAL_REMOTE - Switch Mode when system is mSTANDSTILL only
		{
			HAL_GPIO_TogglePin(DO4_GPIO_Port, DO4_Pin);
			if(pStateMachine->mState == mSTANDSTILL)
			{
				pStateMachine->sysLOCAL_REMOTE = pNEXVar->rxVal;
				if(pStateMachine->sysLOCAL_REMOTE!=LOCAL && pStateMachine->sysLOCAL_REMOTE!=REMOTE)
				{
					pStateMachine->sysLOCAL_REMOTE = REMOTE;
				}
			}
			break;
		}
		case 0x56: //SLIDE_BAR_h0 - speed setpoint
		{
			HAL_GPIO_TogglePin(DO4_GPIO_Port, DO4_Pin);
			if(pStateMachine->sysLOCAL_REMOTE == LOCAL)
			{
				pNEXVar->page3_h0_PU_SPEED_SP = pNEXVar->rxVal;
				pNEXVar->page3_n0_PU_SPEED_SP = pNEXVar->rxVal;
			}
			break;
		}
		case 0x57: //SLIDE_BAR_h1 - Amauture current setpoint
		{
			HAL_GPIO_TogglePin(DO4_GPIO_Port, DO4_Pin);
			if(pStateMachine->sysLOCAL_REMOTE == LOCAL)
			{
				pNEXVar->page3_h1_PU_Te_SP = pNEXVar->rxVal;
				pNEXVar->page3_n1_PU_Te_SP = pNEXVar->rxVal;
			}
			break;
		}
		case 0x58: //SLIDE_BAR_h2 - Field current setpoint
		{
			HAL_GPIO_TogglePin(DO4_GPIO_Port, DO4_Pin);
			if(pStateMachine->sysLOCAL_REMOTE == LOCAL)
			{
				pNEXVar->page3_h2_PU_IF_SP = pNEXVar->rxVal;
				pNEXVar->page3_n2_PU_IF_SP = pNEXVar->rxVal;
			}
			break;
		}
		case 0x59: //FAULT_RESET
		{
			HAL_GPIO_TogglePin(DO4_GPIO_Port, DO4_Pin);
			//Send Reset Command to Power Module
			HAL_GPIO_WritePin(Reset_GPIO_Port, Reset_Pin, GPIO_PIN_SET);
			HAL_Delay(5);
			HAL_GPIO_WritePin(Reset_GPIO_Port, Reset_Pin, GPIO_PIN_RESET);
			//Reset Fault State
			pStateMachine->Ia_State = NORMAL;
			//Switch to STOP mode and Wait for New Command
			pStateMachine->mState = mSTANDSTILL;
			break;
		}
		case 0x5A: //FWD_REV
		{
			HAL_GPIO_TogglePin(DO4_GPIO_Port, DO4_Pin);
			//Message Handle
			if(pNEXVar->rxVal == 0)	//Set Direction = Forward
			{
				pStateMachine->Dir = 1;
			}
			else										//Set Direction = Reverse
			{
				pStateMachine->Dir = -1;
			}
			break;
		}
	}
	//Enable HMI display control
	pNEXVar->UpdateEN = 1;
}
/**********************************************************************************************/
void vUsart4_RxHandle(usartData *pRxData, usartData *pTxData, machine_Control *pMControl, adc_Objt *pAdc)
{
	int8_t Hbyte, Lbyte;
	float temp;
	
	pRxData->receiverAddr = pRxData->rxBuff[3]; 	//Read address
	pRxData->messType = pRxData->rxBuff[7];				//Read message type
	/* FOR TEST ONLY BEGIN */
	//pRxData->receiverAddr = 0x00;	
	//pRxData->messType = 0x01;			
	/* FOR TEST ONLY END*/
	
	if(pRxData->receiverAddr == 0x00 || pRxData->receiverAddr == MCU_ADDR)
	{
		switch(pRxData->messType)
		{
			case 0x01:  //Data request
			{
				pTxData->txBuff[0] = FrameStart;
				pTxData->txBuff[1] = 20;								//Frame length
				pTxData->txBuff[2] = 0;									//Group ID
				pTxData->txBuff[3] = 0x00; 							//Receiver: PC
				pTxData->txBuff[4] = MCU_ADDR;					//Sender: MCU		
				if(++pTxData->messCount>65000)					//messCount 
				{
					pTxData->messCount = 0;
				}
				pTxData->txBuff[5] = (uint8_t)(pTxData->messCount&0xff);				//Message count Low byte
				pTxData->txBuff[6] = (uint8_t)((pTxData->messCount>>8)&0xff);		//Message count High byte
				pTxData->txBuff[7] = 0x01;							//0x01: response to data request
				pTxData->txBuff[8] = 11;								//data length
				
				//Speed (rpm)
				temp = (fabs)(pMControl->EsSpeed);
				Hbyte = ((int)(temp) >> 8)& 0xff;
				Lbyte = (int)(temp) & 0xff; 
				pTxData->txBuff[9] = Hbyte; 
				pTxData->txBuff[10] = Lbyte; 
				//Amature current (Ampere x 100)
				temp = (fabs)(pAdc->Iak);
				Hbyte = ((int)(temp*100.0) >> 8) & 0xff;
				Lbyte = (int)(temp*100.0) & 0xff;
				pTxData->txBuff[11] = Hbyte; 
				pTxData->txBuff[12] = Lbyte; 
				//Motor status
				pTxData->txBuff[13] = pMControl->mState; 
				//Sign of Speed
				if(pMControl->EsSpeed < 0)
					pTxData->txBuff[14] = 1;
				else
					pTxData->txBuff[14] = 0;
				//Sign of Current
				if(pAdc->Iak < 0)
					pTxData->txBuff[15] = 1;
				else
					pTxData->txBuff[15] = 0;
				//Frame end
				pTxData->txBuff[19] = FrameEnd;
				//Send data via RS485 Port
				HAL_GPIO_WritePin(W_R_4_GPIO_Port, W_R_4_Pin,GPIO_PIN_SET);			 //RS485 In Transmit mode
				HAL_Delay(1);
				HAL_UART_Transmit(&huart4, pTxData->txBuff, 20, 1000);
				HAL_Delay(10);
				HAL_GPIO_WritePin(W_R_4_GPIO_Port, W_R_4_Pin,GPIO_PIN_RESET);		 //RS485 In Receive mode
				break;
			}
			case 0x02: 	//for reserve
			{
				break;
			}
		}
	}
}
/**********************************************************************************************/
void vUsart_TxHandle(adc_Objt *pADC, machine_Control *pStateMachine, 
	RAMP *pRAMP_Ia, RAMP *pRAMP_Speed, usartData *pTxData, uint8_t Port, uint8_t messType) //For Test Purpose
{	
	uint16_t temp, i;
	//Data header
	pTxData->txBuff[0] = FrameStart;
	pTxData->txBuff[1] = 60;								//Frame length
	pTxData->txBuff[2] = 0;									//Group ID
	pTxData->txBuff[3] = 0x01; 							//Receiver: ESP32
	pTxData->txBuff[4] = 0x00;							//Sender: MCU
	
	if(++pTxData->messCount>65000)
	{
		pTxData->messCount = 0;
	}
	
	pTxData->messCount = usart5_Rx.messCount; //For test data from  HMI
	
	pTxData->txBuff[5] = (uint8_t)(pTxData->messCount&0xff);				//Message count Low byte
	pTxData->txBuff[6] = (uint8_t)((pTxData->messCount>>8)&0xff);		//Message count High byte
	pTxData->txBuff[7] = messType;
	pTxData->messType = messType;
	pTxData->txBuff[8] = 51;								//Data length	
	//Data payload
	//Raw Voltage channel 0
	temp = (uint16_t)(pADC->rawVol0*100);
	pTxData->txBuff[9] = (uint8_t)((temp>>8)&0xff);		//High byte
	pTxData->txBuff[10] = (uint8_t)(temp&0xff);				//Low byte
	//Raw Voltage channel 1
	temp = (uint16_t)(pADC->rawVol1*100);
	pTxData->txBuff[11] = (uint8_t)((temp>>8)&0xff);	//High byte
	pTxData->txBuff[12] = (uint8_t)(temp&0xff);				//Low byte
	//Raw Voltage channel 6
	temp = (uint16_t)(pADC->rawVol6*100);
	pTxData->txBuff[13] = (uint8_t)((temp>>8)&0xff);	//High byte
	pTxData->txBuff[14] = (uint8_t)(temp&0xff);				//Low byte
	//Voltage channel 7
	temp = (uint16_t)(pADC->rawVol7*100);
	pTxData->txBuff[15] = (uint8_t)((temp>>8)&0xff);	//High byte
	pTxData->txBuff[16] = (uint8_t)(temp&0xff);				//Low byte
	//Raw Voltage channel 14
	temp = (uint16_t)(pADC->rawVol14*100);
	pTxData->txBuff[17] = (uint8_t)((temp>>8)&0xff);	//High byte
	pTxData->txBuff[18] = (uint8_t)(temp&0xff);				//Low byte
	
	//Filter Voltage channel 0
	temp = (uint16_t)(pADC->filterVol0*100);
	pTxData->txBuff[19] = (uint8_t)((temp>>8)&0xff);		//High byte
	pTxData->txBuff[20] = (uint8_t)(temp&0xff);				//Low byte
	//Filter Voltage channel 1
	temp = (uint16_t)(pADC->filterVol1*100);
	pTxData->txBuff[21] = (uint8_t)((temp>>8)&0xff);	//High byte
	pTxData->txBuff[22] = (uint8_t)(temp&0xff);				//Low byte
	//Filter Voltage channel 6
	temp = (uint16_t)(pADC->filterVol6*100);
	pTxData->txBuff[23] = (uint8_t)((temp>>8)&0xff);	//High byte
	pTxData->txBuff[24] = (uint8_t)(temp&0xff);				//Low byte
	//Filter Voltage channel 7
	temp = (uint16_t)(pADC->filterVol7*100);
	pTxData->txBuff[25] = (uint8_t)((temp>>8)&0xff);	//High byte
	pTxData->txBuff[26] = (uint8_t)(temp&0xff);				//Low byte
	//Filter Voltage channel 14
	temp = (uint16_t)(pADC->filterVol14*100);
	pTxData->txBuff[27] = (uint8_t)((temp>>8)&0xff);	//High byte
	pTxData->txBuff[28] = (uint8_t)(temp&0xff);				//Low byte
	
	//Input Voltage channel 0
	temp = (uint16_t)(pADC->filterVol_TorqueSP*100);
	pTxData->txBuff[29] = (uint8_t)((temp>>8)&0xff);		//High byte
	pTxData->txBuff[30] = (uint8_t)(temp&0xff);				//Low byte
	//Input Voltage channel 1
	temp = (uint16_t)(pADC->filterVol_SpeedSP*100);
	pTxData->txBuff[31] = (uint8_t)((temp>>8)&0xff);	//High byte
	pTxData->txBuff[32] = (uint8_t)(temp&0xff);				//Low byte
	//Input Voltage channel 6
	temp = (uint16_t)(pADC->filterVol_Ia*100);
	pTxData->txBuff[33] = (uint8_t)((temp>>8)&0xff);	//High byte
	pTxData->txBuff[34] = (uint8_t)(temp&0xff);				//Low byte
	//Input channel 7
	temp = (uint16_t)(pADC->filterVol_If*100);
	pTxData->txBuff[35] = (uint8_t)((temp>>8)&0xff);	//High byte
	pTxData->txBuff[36] = (uint8_t)(temp&0xff);				//Low byte
	//Input Voltage channel 14
	temp = (uint16_t)(pADC->filterVol_Vbus*100);
	pTxData->txBuff[37] = (uint8_t)((temp>>8)&0xff);	//High byte
	pTxData->txBuff[38] = (uint8_t)(temp&0xff);				//Low byte
	
	//Torque SP - Nm
	temp = (uint16_t)(pADC->TorqueSP*10);
	pTxData->txBuff[39] = (uint8_t)((temp>>8)&0xff);		//High byte
	pTxData->txBuff[40] = (uint8_t)(temp&0xff);				//Low byte
	//Speed SP - rpm
	temp = (uint16_t)(pADC->SpeedSP);
	pTxData->txBuff[41] = (uint8_t)((temp>>8)&0xff);	//High byte
	pTxData->txBuff[42] = (uint8_t)(temp&0xff);				//Low byte
	//Amture current Ia - Ampere
	temp = (uint16_t)(pADC->Iak*10);
	pTxData->txBuff[43] = (uint8_t)((temp>>8)&0xff);	//High byte
	pTxData->txBuff[44] = (uint8_t)(temp&0xff);				//Low byte
	//Field current If- Ampere
	temp = (uint16_t)(pADC->If*10);
	pTxData->txBuff[45] = (uint8_t)((temp>>8)&0xff);	//High byte
	pTxData->txBuff[46] = (uint8_t)(temp&0xff);				//Low byte
	//DC bus voltage - V
	temp = (uint16_t)(pADC->Vbus);
	pTxData->txBuff[47] = (uint8_t)((temp>>8)&0xff);	//High byte
	pTxData->txBuff[48] = (uint8_t)(temp&0xff);				//Low byte
	//Feedback Torque - Nm
	temp = (uint16_t)(pADC->Torque*10);
	pTxData->txBuff[49] = (uint8_t)((temp>>8)&0xff);	//High byte
	pTxData->txBuff[50] = (uint8_t)(temp&0xff);				//Low byte
	//Feedback Speed - rpm

	//State Machine
	pTxData->txBuff[53] = pStateMachine->mState;
	//Ia Ramp State
	pTxData->txBuff[54] = pRAMP_Ia->State;
	//Speed Ramp State
	pTxData->txBuff[55] = pRAMP_Speed->State;
	for(i=56;i<59;i++)
	{
		pTxData->txBuff[i] = 0;
	}
	pTxData->txBuff[59] = FrameEnd;
	//Send data to ECU
	switch(Port)
	{
		case ESP32_Port:
		{
			HAL_UART_Transmit(&huart2, pTxData->txBuff, 60, 1000);	
			break;
		}
		case RS485_Port:
		{
			HAL_GPIO_WritePin(W_R_4_GPIO_Port, W_R_4_Pin,GPIO_PIN_SET);			 //RS485 In Transmit mode
			HAL_Delay(1);
			//HAL_UART_Transmit(&huart4, pTxData->txBuff, 60, 1000);
			HAL_Delay(10);
			HAL_GPIO_WritePin(W_R_4_GPIO_Port, W_R_4_Pin,GPIO_PIN_RESET);			//RS485 In Receive mode
			break;
		}
		case HMI_Port:
		{
			
			break;
		}
	}	
}
/**********************************************************************************************/
void vUsart_Read(usartData *pRxData, uint8_t Port)	//Read Data from serial port
{
	int i = 0;
	//Process receive data
	switch (pRxData->STATE)
	{
		case  USART_WAIT:                             //Wait for start of new message
		{
			pRxData->rxBuff[0] = pRxData->rxByte;       //Read first byte
			if (pRxData->rxBuff[0] == 0x7A)             //Start of new message
			{
				pRxData->STATE = USART_READING;           //switch to Reading state
				pRxData->rxPointer = 1;
			}
			else				
			{
				pRxData->rxPointer = 0;                   //Reset pointer
				if(Port==HMI_Port)												//NEXTION HMI data handle
				{
					vTim_Start(&Tim_1s[1],2);								//SET timeout = 2s for HMI port
					pRxData->NEXTION_Buff[pRxData->NEXTION_Pointer] = pRxData->rxByte;
					pRxData->NEXTION_Pointer = pRxData->NEXTION_Pointer + 1;
					if(pRxData->NEXTION_Pointer == 4)
					{
						//Verify recv data
						if(pRxData->NEXTION_Buff[1] ==0xff && pRxData->NEXTION_Buff[2] ==0xff && pRxData->NEXTION_Buff[3] ==0xff)
						{
							//Turn on NEXTION recv flag
							pRxData->NEXTION_Flag = 1;	
							pRxData->NEXTION_RetCode = pRxData->NEXTION_Buff[0];
						}
						//Clear NEXTION_Buff
						for(i=0;i<4;i++)
						{
							pRxData->NEXTION_Buff[i] = 0;
						}
						pRxData->NEXTION_Pointer = 0;
					}
				}
			}
			break;
		}
		case USART_READING:
		{
			if(Port == ESP32_Port)
			{
				vTim_Start(&Tim_1s[2],2);							//SET timeout = 2s for ESP32 port
			}
			else if(Port == RS485_Port)
			{
				vTim_Start(&Tim_1s[3],2);							//SET timeout = 2s for RS485 port
			}
			else if(Port == HMI_Port)
			{
				vTim_Start(&Tim_1s[4],2);							//SET timeout = 2s for RS485 port
			}
			pRxData->rxBuff[pRxData->rxPointer] = pRxData->rxByte;  //Read data
			if (pRxData->rxPointer == 1)
			{
				pRxData->frameLength = pRxData->rxBuff[pRxData->rxPointer];
				if (pRxData->frameLength > 100 || pRxData->frameLength < 10)  //data length is violated -> Fail
				{
					pRxData->rxPointer = 0;
					pRxData->RxFlag = 0;
					for (i = 0; i < 100; i++)
					{
						pRxData->rxBuff[i] = 0;
					}
					pRxData->frameLength = 0;
					pRxData->STATE = USART_WAIT;
				}
				pRxData->rxPointer = pRxData->rxPointer + 1;        					//Increse the pointer
			}
			else if (pRxData->rxPointer == (pRxData->frameLength - 1)) 			//Check the last byte
			{
				if (pRxData->rxBuff[pRxData->rxPointer] == 0x7f)      				//Success -> Turn on the Flag and process data
				{
					//Turn on the Flag
					pRxData->RxFlag = 1;
					pRxData->STATE = USART_PENDING;
					HAL_GPIO_TogglePin(DO3_GPIO_Port, DO3_Pin);
				}
				else  //Fail -> Reset buffer, pointer and move to WAIT state
				{
					pRxData->rxPointer = 0;
					pRxData->RxFlag = 0;
					for (i = 0; i < 100; i++)
					{
						pRxData->rxBuff[i] = 0;
					}
					pRxData->frameLength = 0;
					pRxData->STATE = USART_WAIT;
				}
			}
			else
			{
				pRxData->rxPointer = pRxData->rxPointer + 1;        //Increse the pointer
				if (pRxData->rxPointer > (pRxData->frameLength - 1)) //Fail
				{
					pRxData->rxPointer = 0;
					pRxData->RxFlag = 0;
					for (i = 0; i < 100; i++)
					{
						pRxData->rxBuff[i] = 0;
					}
					pRxData->frameLength = 0;
					pRxData->STATE = USART_WAIT;
				}
			}
			break;
		}
		case USART_PENDING:   //Wait for rx data to be handled
		{
			//Do nothing
			break;
		}
	}	
}
/**********************************************************************************************/
void vUSART_RxTimeOutHandle(usartData *pRxData)
{
	uint8_t i = 0;
	pRxData->rxPointer = 0;
	pRxData->RxFlag = 0;
	for (i = 0; i < 100; i++)
	{
		pRxData->rxBuff[i] = 0;
	}
	pRxData->frameLength = 0;
	pRxData->STATE = USART_WAIT;
}
/**********************************************************************************************/
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	
  /* Prevent unused argument(s) compilation warning */
  UNUSED(huart);
  /* NOTE: This function should not be modified, when the callback is needed,
           the HAL_UART_RxCpltCallback could be implemented in the user file
   */
	if(huart->Instance == USART2)											//ESP32
	{
		vUsart_Read(&usart2_Rx, 2);
		HAL_UART_Receive_IT(&huart2, &usart2_Rx.rxByte, 1);
	}
	if(huart->Instance == UART4)											//RS485
	{
		HAL_GPIO_TogglePin(DO4_GPIO_Port, DO4_Pin);
		vUsart_Read(&usart4_Rx, 4);
		HAL_UART_Receive_IT (&huart4, &usart4_Rx.rxByte, 1);	
	}
	if(huart->Instance == UART5)											//HMI
	{
		//HAL_GPIO_TogglePin(DO4_GPIO_Port, DO4_Pin);
		vUsart_Read(&usart5_Rx, 5);
		HAL_UART_Receive_IT (&huart5, &usart5_Rx.rxByte, 1);	
	}
}
