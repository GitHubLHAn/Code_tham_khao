#include "stm32f1xx_hal_tim.h"
#include "math.h"
#include "stdlib.h"
#include "stdio.h"
#include "main.h"
/**********************************************************************************************/
//2nd Order Unity-Gain LPF
extern LPF stLPF_Var[NumOfLPF];

/*Timer object*/
extern timer_Objt Tim_1ms[MaxTIMER];					//10ms software timer
extern timer_Objt Tim_10ms[MaxTIMER];				//10ms software timer
extern timer_Objt Tim_20ms[MaxTIMER];				//20ms software timer
extern timer_Objt Tim_100ms[MaxTIMER];				//100ms software timer
extern timer_Objt Tim_200ms[MaxTIMER];				//200ms software timer - for HMI Display
extern timer_Objt Tim_1s[MaxTIMER];					//1s software timer
extern uint16_t tim1msTick, tim10msTick, tim20msTick, tim100msTick, tim200msTick, tim1sTick;
//GPIO object
extern gpio_Objt DI_Objt[NumOfDI];
/*ADC data*/
extern adc_Objt adc_Objt_var;
extern uint16_t adc_data[7];
extern ADC_HandleTypeDef hadc1;
/*DAC data*/
extern DAC_OUT stDAC_Var;
extern DAC_HandleTypeDef hdac;
/*uart data*/
extern usartData usart2_Tx, usart2_Rx;						//ESP32 - Wifi
extern usartData usart4_Tx, usart4_Rx; 					//RS485
extern usartData usart5_Tx, usart5_Rx; 					//HMI

extern UART_HandleTypeDef huart2, huart4, huart5;
extern uint8_t First_Cycle_Flag;
//PI controller
cPI stPI_Speed, stPI_Ia;						
//State machine
machine_Control stMControl_Var;		
machine_Parameter stMachine_Var;
//RAMP Generator (Ampere with Ia, and rpm with Speed)
RAMP stRamp_Ia, stRamp_Speed, stRamp_If, stRamp_Vol;			

//NEXTION HMI
extern NEXTION NEXVar;
//Quadrature Encoder
extern QEI stQEI_Var;	

/**********************************************************************************************/
void vDIO_Task(machine_Control *pMControl)
{
	if(DI_Objt[0].event==Falling&&pMControl->sysENABLE==1&&pMControl->sysLOCAL_REMOTE==REMOTE)	//START
	{
		if(stMControl_Var.chState == DOL && stMControl_Var.If_State == NORMAL && stMControl_Var.mState != mFAULT)
		{
			stMControl_Var.mState = mRUN;
			//Connect the output of conveter to motor via DO1
			HAL_GPIO_WritePin(DO1_GPIO_Port, DO1_Pin, GPIO_PIN_SET);
		}
	}
	if(DI_Objt[1].event == Falling)	//STOP
	{
		pMControl->mState = mSTOP;
	}
	
	if(DI_Objt[2].event == Falling)	//Do nothing
	{
		
	}
	if(DI_Objt[2].event == Low && pMControl->sysLOCAL_REMOTE==REMOTE)			//Speed	control
	{
		if(stMControl_Var.mState == mSTANDSTILL)
		{
			//stMControl_Var.ctrMode = SPEED;
		}
	}
	if(DI_Objt[2].event == High && pMControl->sysLOCAL_REMOTE==REMOTE)		//Torque	control
	{
		if(stMControl_Var.mState == mSTANDSTILL)
		{
			//stMControl_Var.ctrMode = TORQUE;
		}
	}
	
	if(DI_Objt[3].event == Falling)	//STOP
	{
		pMControl->mState = mSTOP;
	}
	if(DI_Objt[4].event == Falling) //External FAULT
	{
		pMControl->mState = mFAULT;
	}
	if(DI_Objt[5].event == Falling)	//For Reserve
	{
		
	}
	if(DI_Objt[6].event == Falling)	//For Reserve
	{
		
	}
	if(DI_Objt[7].event == Falling)	//For Reserve
	{
		
	}
}
/**********************************************************************************************/
void vMachineParameterConfig(machine_Parameter *pMParas)
{
	pMParas->Ia_rated = 15;					//Ampere
	pMParas->If_rated = 1;					//Ampere
	pMParas->Speed_rated = 1800.0;	//rpm
	pMParas->Power_rated = 2200;		//W
	pMParas->Va_rated = 220;				//V
	//Current to Torque Gain
	pMParas->Speed_max = rpmMAX; 		//rpm
	pMParas->kPhiConst = pMParas->Power_rated/(pMParas->Ia_rated*pMParas->Speed_rated/9.55);
	pMParas->Torque_rated = pMParas->Ia_rated * pMParas->kPhiConst;
	//Amature resistance estimation
	pMParas->Ra = (pMParas->Va_rated - (pMParas->kPhiConst*pMParas->Speed_rated/9.55))/pMParas->Ia_rated;
	pMParas->La = 0.02;							//20mH
}
/**********************************************************************************************/
void vStateMachineConfig(machine_Control *pMControl, machine_Parameter *pMParas)
{
	/*pMControl->Ia_H_Alarm = pMParas->Ia_rated * 1.2;
	pMControl->Ia_H_Fault = pMParas->Ia_rated *3.5;*/
	
	pMControl->Ia_H_Alarm = 10.0;
	pMControl->Ia_H_Fault = 15;
	
	pMControl->If_H_Alarm = pMParas->If_rated *1.2;
	pMControl->If_H_Fault = pMParas->If_rated *1.5;
	pMControl->If_L_Alarm = pMParas->If_rated *0.5;
	pMControl->If_L_Fault = pMParas->If_rated * 0.3;
	pMControl->Speed_H_Alarm = pMParas->Speed_rated * 1.1;
	pMControl->Speed_H_Fault = pMParas->Speed_rated * 1.2;
	
	pMControl->Vdc_Nom = 310.0;								//Nominal value of DC bus voltage
	pMControl->Vdc_H_Alarm = pMControl->Vdc_Nom*1.1;
	pMControl->Vdc_H_Fault = pMControl->Vdc_Nom*1.3;
	pMControl->Vdc_L_Alarm = pMControl->Vdc_Nom*0.9;
	pMControl->Vdc_L_Fault = pMControl->Vdc_Nom*0.7;
	pMControl->Vdc_Hi = pMControl->Vdc_Nom*0.9;
	pMControl->Vdc_Lo = pMControl->Vdc_Nom*0.8;
	
	pMControl->sysENABLE = 1;								//Always ENABLE
	pMControl->sysLOCAL_REMOTE = LOCAL;			//Controlled by HMI
	pMControl->mState = mSTANDSTILL;
	pMControl->Dir = 1;											//FORWARD
	pMControl->isrCnt = 0;
	pMControl->ctrMode = OPEN_LOOP;
	pMControl->EsSpeed = 0;
	pMControl->SpeedLimit = 1000;						//Limit speed <=1000rpm in Torque control mode
	pMControl->SpeedEs = 0;
	pMControl->KLimit = 0.5*DAC_MAX_Vout/(pMControl->Speed_H_Alarm - pMControl->SpeedLimit);
}
/**********************************************************************************************/
void vStateMachine(void) //State machine and speed loop
{
	//Control the charge resistor
	/*if(adc_Objt_var.Vbus < stMControl_Var.Vdc_Lo)
	{
		//Open the bypass contactor
		HAL_GPIO_WritePin(Bypass_GPIO_Port, Bypass_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(DO0_GPIO_Port, DO0_Pin, GPIO_PIN_RESET);			
		stMControl_Var.chState = CHARGE;
	}
	else if(adc_Objt_var.Vbus > stMControl_Var.Vdc_Hi)
	{
		//Close the bypass contactor
		HAL_GPIO_WritePin(Bypass_GPIO_Port, Bypass_Pin, GPIO_PIN_SET);	
		HAL_GPIO_WritePin(DO0_GPIO_Port, DO0_Pin, GPIO_PIN_SET);
		stMControl_Var.chState = DOL;														
	}*/
	//Estimate the speed
	stMControl_Var.EsSpeed = (710.0/1750.0)*(9.55/stMachine_Var.kPhiConst)*(stMControl_Var.Vcontrol/DAC_MAX_Vout)*stMControl_Var.Vdc_Nom 
	- stMachine_Var.Ra*adc_Objt_var.Iak - adc_Objt_var.dIak_dt*stMachine_Var.La;
	//Control the machine
	if(stMControl_Var.mState == mSTANDSTILL)	
	{
		//Disable the Power module
		HAL_GPIO_WritePin(Enable_GPIO_Port, Enable_Pin, GPIO_PIN_RESET);
		//Reset PI Speed Controller 10ms sampling time
		vPI_Init(&stPI_Speed, 1, 0.1, 0.01, adc_Objt_var.MaxSpeedSP, adc_Objt_var.MaxTorqueSP);	
		//Reset PI amature current controller with 0.5ms sampling time
		//vPI_Init(&stPI_Ia, 1, 0.1, 0.0005, adc_Objt_var.MaxIaSP, (stMachine_Var.Va_rated/stMControl_Var.Vdc_Nom)*DAC_MAX_Vout);	
		vPI_Init(&stPI_Ia, 1, 0.1, 0.0005, adc_Objt_var.MaxIaSP, (280.0/stMControl_Var.Vdc_Nom)*DAC_MAX_Vout);
		//Duty cyble = 0
		vDAC_Write(&stDAC_Var, DAC_DutyCycle, 0);	
		stMControl_Var.EsSpeed = 0;
		//Disconnect the Output from DC motor via DO1
		HAL_GPIO_WritePin(DO1_GPIO_Port, DO1_Pin, GPIO_PIN_RESET);
	}
	/*----------10ms - Open-Loop or Speed-Loop-----------*/
	if(stMControl_Var.isrCnt == 19)	
	{
		//Compute the Setpoint
		if(stMControl_Var.mState !=mSTOP)
		{
			switch(stMControl_Var.sysLOCAL_REMOTE)
			{
				case LOCAL:	//Operated by HMI
				{
					stMControl_Var.Te_SP = stMControl_Var.Dir*((float)(NEXVar.page3_n1_PU_Te_SP)/100.0)*10*stMachine_Var.kPhiConst;	//Nm
					stMControl_Var.Ia_SP = stMControl_Var.Te_SP/stMachine_Var.kPhiConst;	//Convert TeSP to IaSP - Ampere
					stMControl_Var.If_SP = ((float)(NEXVar.page3_n2_PU_IF_SP)/100.0)*adc_Objt_var.MaxIfSP;
					stMControl_Var.Speed_SP = stMControl_Var.Dir*((float)(NEXVar.page3_n0_PU_SPEED_SP)/100.0)*adc_Objt_var.MaxSpeedSP;	//rpm
					stMControl_Var.Vol_SP = stMControl_Var.Dir*((float)(NEXVar.page3_n0_PU_SPEED_SP)/100.0)*stMachine_Var.Va_rated;	//rpm
					break;
				}
				case REMOTE: //Operated by GPIO and Analog Inputs
				{
					stMControl_Var.Te_SP = stMControl_Var.Dir*adc_Objt_var.TorqueSP;	//Nm
					stMControl_Var.Ia_SP = stMControl_Var.Dir*stMControl_Var.Te_SP/stMachine_Var.kPhiConst;	//Ampere
					stMControl_Var.If_SP = stMachine_Var.If_rated;	//Corresponding to If_rated
					stMControl_Var.Speed_SP = stMControl_Var.Dir*adc_Objt_var.SpeedSP;	//rpm
					stMControl_Var.Vol_SP = stMControl_Var.Dir*adc_Objt_var.PU_SpeedSP*stMachine_Var.Va_rated/100.0;	//Vol
					break;
				}
			}
		}
		else
		{
			stMControl_Var.Te_SP = 0;			//Nm
			stMControl_Var.Ia_SP = 0;			//Ampere
			stMControl_Var.If_SP = stMachine_Var.If_rated;			//Ampere
			stMControl_Var.Speed_SP = 0;	//rpm
			stMControl_Var.Vol_SP = 0;		//Vol
		}
		
		//Decelerating to Standstill
		if(stMControl_Var.mState == mSTOP)	
		{
			//Enable the Power module
			HAL_GPIO_WritePin(Enable_GPIO_Port, Enable_Pin, GPIO_PIN_SET);
			//Connect the output of conveter to motor via DO1
			HAL_GPIO_WritePin(DO1_GPIO_Port, DO1_Pin, GPIO_PIN_SET);
			
			if(stMControl_Var.ctrMode == OPEN_LOOP)
			{
				//Generate reference Amature voltage
				fRampFcnGen(stMControl_Var.Vol_SP, &stRamp_Vol);
				//Send reference Voltage to the power converter via DAC
				stMControl_Var.Vcontrol = (stRamp_Vol.Output/stMControl_Var.Vdc_Nom)*DAC_MAX_Vout;
				vDAC_Write(&stDAC_Var, DAC_DutyCycle, stMControl_Var.Vcontrol);	
				if(stMControl_Var.Vol_SP == 0 && stRamp_Vol.Output == 0)
				{
					if(stMControl_Var.Ia_State == FAULT_H)
					{
						stMControl_Var.mState = mFAULT;
					}
					else
					{
						stMControl_Var.mState = mSTANDSTILL;
					}
				}
			}
			else if(stMControl_Var.ctrMode == SPEED)
			{
				//Generate reference speed
				fRampFcnGen(stMControl_Var.Speed_SP, &stRamp_Speed);
				//Run the PI Speed controller
				//fPI_Run(&stPI_Speed, stRamp_Speed.Output, stQEI_Var.rpm);
				fPI_Run(&stPI_Speed, stRamp_Speed.Output, stMControl_Var.EsSpeed);
				if(stMControl_Var.Speed_SP == 0 && stRamp_Speed.Output ==0)
				{
					if(stMControl_Var.Ia_State == FAULT_H)
					{
						stMControl_Var.mState = mFAULT;
					}
					else
					{
						stMControl_Var.mState = mSTANDSTILL;
					}
				}
				//Send reference excitation current to the power converter
				//vDAC_Write(&stDAC_Var, DAC_IfRefChannel, (stMControl_Var.If_SP/adc_Objt_var.MaxIf)*DAC_MAX_Vout);	
			}
		}	
		else if(stMControl_Var.mState == mRUN)	
		{
			//Enable the Power module
			HAL_GPIO_WritePin(Enable_GPIO_Port, Enable_Pin, GPIO_PIN_SET);
			//Connect the output of conveter to motor via DO1
			HAL_GPIO_WritePin(DO1_GPIO_Port, DO1_Pin, GPIO_PIN_SET);
			if(stMControl_Var.ctrMode == OPEN_LOOP)
			{
				//Generate reference Amature voltage
				fRampFcnGen(stMControl_Var.Vol_SP, &stRamp_Vol);
				//Send reference Voltage to the power converter via DAC
				stMControl_Var.Vcontrol = (stRamp_Vol.Output/stMControl_Var.Vdc_Nom)*DAC_MAX_Vout;
				vDAC_Write(&stDAC_Var, DAC_DutyCycle, stMControl_Var.Vcontrol);	
			}			
			else if(stMControl_Var.ctrMode == SPEED)
			{
				//Generate reference speed
				fRampFcnGen(stMControl_Var.Speed_SP, &stRamp_Speed);
				//Run the PI Speed controller
				//fPI_Run(&stPI_Speed, stRamp_Speed.Output, stQEI_Var.rpm);
				fPI_Run(&stPI_Speed, stRamp_Speed.Output, stMControl_Var.EsSpeed);
				//Send reference excitation current to the power converter
				//vDAC_Write(&stDAC_Var, DAC_IfRefChannel, (stMControl_Var.If_SP/adc_Objt_var.MaxIfSP)*DAC_MAX_Vout);	
			}
		}
	}
	/*--------0.5ms for Torque control-----------*/	
	if(stMControl_Var.mState == mSTOP)	
	{
		//Enable the Power module
		HAL_GPIO_WritePin(Enable_GPIO_Port, Enable_Pin, GPIO_PIN_SET);
		//Connect the output of conveter to motor via DO1
		HAL_GPIO_WritePin(DO1_GPIO_Port, DO1_Pin, GPIO_PIN_SET);
		if(stMControl_Var.ctrMode == TORQUE)	//Torque reference from HMI or Analog input
		{
			//Generate reference Amature current
			fRampFcnGen(stMControl_Var.Ia_SP, &stRamp_Ia);
			//Run the PI Torque controller
			fPI_Run(&stPI_Ia, stRamp_Ia.Output, adc_Objt_var.Iak);
			//Send reference Amature current to the power converter via DAC 
			if(stMControl_Var.EsSpeed > stMControl_Var.SpeedLimit)
			{
				stMControl_Var.SpeedEs = stMControl_Var.EsSpeed - stMControl_Var.SpeedLimit;
			}
			else if(stMControl_Var.EsSpeed < -stMControl_Var.SpeedLimit)
			{
				stMControl_Var.SpeedEs = stMControl_Var.EsSpeed + stMControl_Var.SpeedLimit;
			}
			else
			{
				stMControl_Var.SpeedEs = 0.0;
			}
			stMControl_Var.Vcontrol = stPI_Ia.Usk - stMControl_Var.KLimit*stMControl_Var.SpeedEs;
			vDAC_Write(&stDAC_Var, DAC_DutyCycle, stMControl_Var.Vcontrol);			
			//Send reference excitation current to the power converter
			//vDAC_Write(&stDAC_Var, DAC_IfRefChannel, (stMControl_Var.If_SP/adc_Objt_var.MaxIfSP)*DAC_MAX_Vout);		
			if(stMControl_Var.Ia_SP == 0 && stRamp_Ia.Output == 0)
			{
				if(stMControl_Var.Ia_State == FAULT_H)
				{
					stMControl_Var.mState = mFAULT;
				}
				else
				{
					stMControl_Var.mState = mSTANDSTILL;
				}
			}
		}
		else if(stMControl_Var.ctrMode == SPEED) //Torque reference from output of speed controller
		{
			//Run the PI Torque controller
			fPI_Run(&stPI_Ia, stPI_Speed.Usk/stMachine_Var.kPhiConst, adc_Objt_var.Iak);
			//Send reference Amature current to the power converter via DAC 
			stMControl_Var.Vcontrol = stPI_Ia.Usk;
			vDAC_Write(&stDAC_Var, DAC_DutyCycle, stMControl_Var.Vcontrol);			
			//Send reference excitation current to the power converter
			//vDAC_Write(&stDAC_Var, DAC_IfRefChannel, (stMControl_Var.If_SP/adc_Objt_var.MaxIfSP)*DAC_MAX_Vout);		
		}
	}
	else if(stMControl_Var.mState == mRUN)
	{
		//Enable the Power module
		HAL_GPIO_WritePin(Enable_GPIO_Port, Enable_Pin, GPIO_PIN_SET);
		//Connect the output of conveter to motor via DO1
		HAL_GPIO_WritePin(DO1_GPIO_Port, DO1_Pin, GPIO_PIN_SET);
		if(stMControl_Var.ctrMode == TORQUE)	//Torque reference from HMI or Analog input
		{
			//Generate reference Amature current
			fRampFcnGen(stMControl_Var.Ia_SP, &stRamp_Ia);
			//Run the PI Torque controller
			fPI_Run(&stPI_Ia, stRamp_Ia.Output, adc_Objt_var.Iak);
			//Send reference Amature current to the power converter via DAC
			if(stMControl_Var.EsSpeed > stMControl_Var.SpeedLimit)
			{
				stMControl_Var.SpeedEs = stMControl_Var.EsSpeed - stMControl_Var.SpeedLimit;
			}
			else if(stMControl_Var.EsSpeed < -stMControl_Var.SpeedLimit)
			{
				stMControl_Var.SpeedEs = stMControl_Var.EsSpeed + stMControl_Var.SpeedLimit;
			}
			else
			{
				stMControl_Var.SpeedEs = 0.0;
			}
			stMControl_Var.Vcontrol = stPI_Ia.Usk - stMControl_Var.KLimit*stMControl_Var.SpeedEs;
			vDAC_Write(&stDAC_Var, DAC_DutyCycle, stMControl_Var.Vcontrol);			
			//Send reference excitation current to the power converter
			//vDAC_Write(&stDAC_Var, DAC_IfRefChannel, (stMControl_Var.If_SP/adc_Objt_var.MaxIfSP)*DAC_MAX_Vout);
		}
		else if(stMControl_Var.ctrMode == SPEED) //Torque reference from output of speed controller
		{
			//Run the PI Torque controller
			fPI_Run(&stPI_Ia, stPI_Speed.Usk/stMachine_Var.kPhiConst, adc_Objt_var.Iak);
			//Send reference Amature current to the power converter via DAC 
			stMControl_Var.Vcontrol = stPI_Ia.Usk;
			vDAC_Write(&stDAC_Var, DAC_DutyCycle, stMControl_Var.Vcontrol);		
			//Send reference excitation current to the power converter
			//vDAC_Write(&stDAC_Var, DAC_IfRefChannel, (stMControl_Var.If_SP/adc_Objt_var.MaxIfSP)*DAC_MAX_Vout);
		}
	}
	else if(stMControl_Var.mState == mFAULT)
	{
		//Disable the converter
		HAL_GPIO_WritePin(Enable_GPIO_Port, Enable_Pin, GPIO_PIN_RESET);
		//Amature current reference = 0
		vDAC_Write(&stDAC_Var, DAC_DutyCycle, 0);	
		//Turn on Fault output via DO2
		HAL_GPIO_WritePin(DO2_GPIO_Port, DO2_Pin, GPIO_PIN_SET);
		//Reset Ramp Fcn Generaters
		stRamp_Speed.Output = 0; 
		stRamp_Ia.Output = 0;
		stRamp_If.Output = 0;
		stRamp_Vol.Output = 0;
		//Reset PI Speed Controller 10ms sampling time
		vPI_Init(&stPI_Speed, 0.1, 0.1, 0.01, adc_Objt_var.MaxSpeedSP, adc_Objt_var.MaxTorqueSP);	
		//Reset PI amature current controller with 0.5ms sampling time
		//vPI_Init(&stPI_Ia, 1, 0.1, 0.0005, adc_Objt_var.MaxIaSP, (stMachine_Var.Va_rated/stMControl_Var.Vdc_Nom)*DAC_MAX_Vout);	
		vPI_Init(&stPI_Ia, 1, 0.1, 0.0005, adc_Objt_var.MaxIaSP, (280.0/stMControl_Var.Vdc_Nom)*DAC_MAX_Vout);
		//Disconnect output to DC motor
		HAL_GPIO_WritePin(DO1_GPIO_Port, DO1_Pin, GPIO_PIN_RESET);
	}	
	//Handle the isr counter
	if(++stMControl_Var.isrCnt > 19)
	{
		stMControl_Var.isrCnt = 0;
	}
}
/**********************************************************************************************/
/*Safety chain handle*/
void vSafetyChainHandle(adc_Objt *pAdcObjt, machine_Control *pMControl, QEI *pQEI)
{
	//High amature current detect
	if(pMControl->Ia_State != FAULT_H)
	{
		if((fabs)(pAdcObjt->Iak) > pMControl->Ia_H_Alarm && (fabs)(pAdcObjt->Iak) < pMControl->Ia_H_Fault)	//Alarm
		{
			vTim_Start(&Tim_1s[5],5);																//SET time out = 5s
			pMControl->Ia_State = ALARM_H;
		}
		else if((fabs)(pAdcObjt->Iak) > pMControl->Ia_H_Fault)		//Over current
		{
			stMControl_Var.mState = mSTOP;
			pMControl->Ia_State  = FAULT_H;													//Ia Over Current
		}
		else if((fabs)(pAdcObjt->Iak) < pMControl->Ia_H_Alarm*0.95)
		{
			vTim_Stop(&Tim_1s[5]);
			pMControl->Ia_State = NORMAL;
		}
	}
	
	//High excitation current detect
	/*if(pAdcObjt->If > pMControl->If_H_Alarm && pAdcObjt->If < pMControl->If_H_Fault)
	{
		vTim_Start(&Tim_1s[6],5);						//SET time out = 5s
		pMControl->If_State = ALARM_H;
	}
	else if(pAdcObjt->If > pMControl->If_H_Fault)
	{
		pMControl->mState = mFAULT;
		pMControl->If_State = FAULT_H;				//If Over Current
	}
	else if(pAdcObjt->If < pMControl->If_H_Alarm*0.95)
	{
		vTim_Stop(&Tim_1s[6]);
		pMControl->If_State = NORMAL;
	}
	//Low excitation current detect
	if(pAdcObjt->If < pMControl->If_L_Alarm && pAdcObjt->If > pMControl->If_L_Fault)
	{
		vTim_Start(&Tim_1s[7],5);						//SET time out = 5s
		pMControl->If_State = ALARM_L;
	}
	else if(pAdcObjt->If <= pMControl->If_L_Fault)
	{
		pMControl->mState = mFAULT;
		pMControl->If_State = FAULT_L;			//Low excitation current
	}
	else if(pAdcObjt->If > pMControl->If_L_Alarm*1.05)
	{
		vTim_Stop(&Tim_1s[7]);
		pMControl->If_State = NORMAL;
	}
	//Low VDC detect - Only in DOL mode
	if(stMControl_Var.chState == DOL)
	{
		if(pAdcObjt->Vbus < pMControl->Vdc_L_Alarm && pAdcObjt->Vbus > pMControl->Vdc_L_Fault)
		{
			vTim_Start(&Tim_1s[8],5);				//SET time out = 5s
			pMControl->Vdc_State = ALARM_L;
		}
		else if(pAdcObjt->Vbus <= pMControl->Vdc_L_Fault)
		{
			pMControl->mState = mFAULT;
			pMControl->Vdc_State = FAULT_L;		//Low dc voltage
		}
		else if(pAdcObjt->Vbus > pMControl->Vdc_L_Alarm*1.05)
		{
			vTim_Stop(&Tim_1s[8]);
			pMControl->Vdc_State = NORMAL;
		}
	}
	
	//High VDC detect
	if(pAdcObjt->Vbus > pMControl->Vdc_H_Alarm && pAdcObjt->Vbus < pMControl->Vdc_H_Fault)
	{
		vTim_Start(&Tim_1s[9],5);							//SET time out = 5s
		pMControl->Vdc_State = ALARM_H;
	}
	else if(pAdcObjt->Vbus >= pMControl->Vdc_H_Fault)
	{
		stMControl_Var.mState = mFAULT;
		stMControl_Var.Vdc_State = FAULT_H;			//Over voltage
	}
	else if(pAdcObjt->Vbus < pMControl->Vdc_H_Alarm*0.95)
	{
		vTim_Stop(&Tim_1s[9]);
		stMControl_Var.Vdc_State = NORMAL;
	}
	//High speed detect
	if(pQEI->rpm > pMControl->Speed_H_Alarm && pQEI->rpm  < pMControl->Speed_H_Fault)
	{
		vTim_Start(&Tim_1s[10],5);						//SET time out = 5s
		pMControl->Speed_State = ALARM_H;
	}
	else if(pQEI->rpm >= pMControl->Speed_H_Fault)
	{
		pMControl->mState = mFAULT;
		pMControl->Speed_State = FAULT_H;				//Over speed
	}
	else if(pQEI->rpm < pMControl->Speed_H_Alarm*0.95)
	{
		vTim_Stop(&Tim_1s[10]);
		pMControl->Speed_State = NORMAL;
	}*/
}


