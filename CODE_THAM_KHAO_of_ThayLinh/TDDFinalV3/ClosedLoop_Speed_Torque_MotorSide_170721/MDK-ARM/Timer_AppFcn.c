#include "main.h"

/*Timer object*/
timer_Objt Tim_1ms[MaxTIMER];					//10ms software timer
timer_Objt Tim_10ms[MaxTIMER];				//10ms software timer
timer_Objt Tim_20ms[MaxTIMER];				//20ms software timer
timer_Objt Tim_100ms[MaxTIMER];				//100ms software timer
timer_Objt Tim_200ms[MaxTIMER];				//200ms software timer - for HMI Display
timer_Objt Tim_1s[MaxTIMER];					//1s software timer
uint16_t tim1msTick, tim10msTick, tim20msTick, tim100msTick, tim200msTick, tim1sTick;

/*ADC data*/
extern adc_Objt adc_Objt_var;
extern uint16_t adc_data[7];
extern ADC_HandleTypeDef hadc1;

//Quadrature Encoder
extern QEI stQEI_Var;	

//NEXTION HMI
extern NEXTION NEXVar;
//PI controller
extern cPI stPI_Speed, stPI_Ia;						
//State machine
extern machine_Control stMControl_Var;		
extern machine_Parameter stMachine_Var;
//RAMP Generator (Ampere with Ia, and rpm with Speed)
extern RAMP stRamp_Ia, stRamp_Speed, stRamp_If, stRamp_Vol;
/**********************************************************************************************/
void vTim_Start(timer_Objt *ptim, uint16_t SV)			//Start timer
{
	if(ptim->En == 0)	//if the corresponding timer is in stop mode
	{
		ptim->En = 1;
		if(SV<1)
			SV = 1;
		ptim->SV = SV-1;
		ptim->Output = 0;
		ptim->PV = 0;
	}
}
/**********************************************************************************************/
void vTim_Stop(timer_Objt *ptim)									//Reset timer
{
	ptim->En = 0;
	ptim->SV = 0;
	ptim->Output = 0;
	ptim->PV = 0;
}
/**********************************************************************************************/
void vTim_Scan(timer_Objt *pTIM)	
{
	int i = 0;
	for(i=0;i<MaxTIMER;i++)									//Scan all declared timers
	{
		if(pTIM[i].En == 1)
		{
			if(++pTIM[i].PV > pTIM[i].SV)				//Overflow
			{
				pTIM[i].PV = 0;
				pTIM[i].Output = 1;								//Turn On the Timer Output
				if(++pTIM[i].OvfCnt > 65535)
				{
					pTIM[i].OvfCnt = 0;
				}
			}		
		}
	}
}
/**********************************************************************************************/
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(htim);		
	/*USER CODE*/
	if(htim->Instance==TIM2)				//Timer 2 interrupt every 10ms - Speed Loop
	{
		vRPM_Cal(&stQEI_Var, &adc_Objt_var);
		tim10msTick = 1; 
	}
	if(htim->Instance==TIM3)				//Encoder 
	{
		 stQEI_Var.Cnt_Ovf = stQEI_Var.Cnt_Ovf+1;
		if(stQEI_Var.Cnt_Ovf > 65000)
		{
			stQEI_Var.Cnt_Ovf = 0;
		}
	}
	if(htim->Instance==TIM8)		//Timer 8 interrupt every 20ms
	{
		tim20msTick = 1; 
	}
	if(htim->Instance==TIM4)		//Timer 4 interrupt every 100ms
	{
		tim100msTick = 1; 
		//Prepare display data
		/*NEXVar.page4_SpRef = 100.0*(float)(stRamp_Speed.Output)/adc_Objt_var.MaxSpeedSP + GraphOffset;
		NEXVar.page4_SpFb = stQEI_Var.PU_rpm + GraphOffset;
		NEXVar.page4_IaFb = adc_Objt_var.PU_Ia + GraphOffset;
		if(stMControl_Var.ctrMode == SPEED)
		{
			NEXVar.page4_IaRef = 100.0*(stPI_Speed.Usk/stMachine_Var.kPhiConst)/adc_Objt_var.MaxIaSP + GraphOffset;	//% of MaxIaSP
			NEXVar.page4_SpUsk = 100.0*(stPI_Speed.Usk)/adc_Objt_var.MaxTorqueSP + GraphOffset;									//% of MaxTorqueSP
			NEXVar.page4_IaUsk = 100.0*(stPI_Ia.Usk/DAC_MAX_Vout) + GraphOffset;													//% of DAC_MAX_Vout
		}
		else if(stMControl_Var.ctrMode == TORQUE)
		{
			NEXVar.page4_IaRef  = 100.0*(stRamp_Ia.Output/adc_Objt_var.MaxIaSP)+ GraphOffset;								//% of MaxIaSP
			NEXVar.page4_SpUsk = 0 + GraphOffset;
			NEXVar.page4_IaUsk = 100.0*(stPI_Ia.Usk/DAC_MAX_Vout) + GraphOffset;													//% of DAC_MAX_Vout
		}
		else	//Open loop
		{
			NEXVar.page4_IaRef = 0 + GraphOffset;
			NEXVar.page4_SpUsk = 0 + GraphOffset;
			NEXVar.page4_IaUsk = 0 + GraphOffset;
		}
		//Send data to Scope
		vNEXTION_SendVal("page4.SpRef", NEXVar.page4_SpRef, DATA_INT);
		vNEXTION_SendVal("page4.SpFb", NEXVar.page4_SpFb, DATA_INT);
		vNEXTION_SendVal("page4.IaRef", NEXVar.page4_IaRef, DATA_INT);
		vNEXTION_SendVal("page4.IaFb", NEXVar.page4_IaFb, DATA_INT);
		vNEXTION_SendVal("page4.En", 1, DATA_INT);
		vNEXTION_SendVal("page4.SpUsk", NEXVar.page4_SpUsk, DATA_INT);
		vNEXTION_SendVal("page4.IaUsk", NEXVar.page4_IaUsk, DATA_INT);*/
	}
	if(htim->Instance==TIM5)		//Timer 5 interrupt every 1s
	{
		tim1sTick = 1;
	}
	if(htim->Instance == TIM6)	//Timer 5 - 0.5ms - Torque Loop
	{
		tim1msTick = 1;
		//ADC read
		HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_Objt_var.data, 5);
		//ADC Data Handle
		vAdc_Data_Handle(&adc_Objt_var, &stMachine_Var);
		//Machine control
		vStateMachine();
	}
}
