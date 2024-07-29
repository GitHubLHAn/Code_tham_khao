#include "main.h"

/*ADC data*/
adc_Objt adc_Objt_var;
uint16_t adc_data[7];
extern ADC_HandleTypeDef hadc1;
/*DAC data*/
DAC_OUT stDAC_Var;
extern DAC_HandleTypeDef hdac;

//2nd Order Unity-Gain LPF
extern LPF stLPF_Var[NumOfLPF];

//State machine
extern machine_Control stMControl_Var;		
extern machine_Parameter stMachine_Var;
/**********************************************************************************************/
void vAdc_DataInit(adc_Objt *pObjt, machine_Parameter *pMParas)			//Initialize adc data
{
	int i; 
	for(i=0;i<adcUsedChannel;i++)
	{
		pObjt->data[i] = 0;
	}	
	pObjt->rawVol0 = 0;
	pObjt->rawVol1 = 0;
	pObjt->rawVol6 = 0;
	pObjt->rawVol7 = 0;
	pObjt->rawVol14 = 0;
	
	pObjt->calibVol0 =1.0; 
	pObjt->calibVol1 =1.0; 
	pObjt->calibVol6 = 1.0;
	pObjt->calibVol7 = 1.0; 
	pObjt->calibVol14 = 1.0; 
	
	//Calib Gain of measured values
	pObjt->calibVbus = 1.0; 
	pObjt->calibIa = 1.0; 
	pObjt->calibIf = 1.0; 
	pObjt->calibSpeedSP = 1.0; 
	pObjt->calibTorqueSP = 1.0; 
	pObjt->Gain1 = (12.0+3.3+4.7)/12.0;
	pObjt->Gain2 = (27.0+10.0)/10.0;
	pObjt->ExVref = 5.0;
	
	pObjt->Offset_Ia = 0;
	pObjt->Offset_If = 0;
	pObjt->Offset_Vbus = 0;
	
	pObjt->IaToTorqueGain = pMParas->kPhiConst;
	pObjt->adcVref = 3.3;													//Reference voltage of the ADC
	pObjt->MaxIa = 10.0;													//Max range of amature current sensor
	pObjt->MaxIf = 10.0;														//Max range of field current sensor 
	pObjt->MaxIfSP = pMParas->If_rated;
	pObjt->MaxIaSP = pMParas->Ia_rated*2;
	pObjt->MaxSpeedSP = pMParas->Speed_rated;			//Rpm
	pObjt->MaxTorqueSP = pObjt->MaxIaSP*pMParas->kPhiConst;		//Nm
	pObjt->MaxVol_SpeedSP = 10.0; 								//Used when setpoint is analog input - V
	pObjt->MaxVol_TorqueSP = 10.0; 								//Used when setpoint is analog input - V
	pObjt->MaxVbus = 500.0;												//Vol		
	
	pObjt->IaSP = 0.0;
	pObjt->Iak = 0.0; pObjt->Iak_1 = 0.0; pObjt->dIak_dt = 0.0;
	pObjt->If = pMParas->If_rated;
	pObjt->Vbus = 0.0;
	pObjt->TorqueSP = 0.0;
	pObjt->Torque = 0.0;
	pObjt->SpeedSP = 0.0;
}
/**********************************************************************************************/
void vAdc_Data_Handle(adc_Objt *pObjt, machine_Parameter *pMParas)
{
	//Compute raw ADC voltage
	pObjt->rawVol0 = pObjt->calibVol0*(pObjt->adcVref*pObjt->data[0]) /4095.0;	
	pObjt->rawVol1 = pObjt->calibVol1*(pObjt->adcVref*pObjt->data[1]) /4095.0;
	pObjt->rawVol6 = pObjt->calibVol6*(pObjt->adcVref*pObjt->data[2]) /4095.0;
	pObjt->rawVol7 = pObjt->calibVol7*(pObjt->adcVref*pObjt->data[3]) /4095.0;	
	pObjt->rawVol14 = pObjt->calibVol14*(pObjt->adcVref*pObjt->data[4]) /4095.0;	
	
	//Compute filter data
	pObjt->filterVol0 = fLPF_Run(&stLPF_Var[0], pObjt->rawVol0);		//Corresponding to TorqueSP
	pObjt->filterVol1 = fLPF_Run(&stLPF_Var[1], pObjt->rawVol1);		//Corresponding to SpeedSP
	pObjt->filterVol6 = fLPF_Run(&stLPF_Var[2], pObjt->rawVol6);		//Corresponding to IaFB
	pObjt->filterVol7 = fLPF_Run(&stLPF_Var[3], pObjt->rawVol7);		//Corresponding to IfFB
	pObjt->filterVol14 = fLPF_Run(&stLPF_Var[4], pObjt->rawVol14);	//Corresponding to VbusFB
	
	//Compute input voltage
	pObjt->filterVol_TorqueSP = (pObjt->filterVol0 * pObjt->Gain2)*pObjt->calibTorqueSP;	//Vol
	pObjt->filterVol_SpeedSP = (pObjt->filterVol1* pObjt->Gain2)*pObjt->calibSpeedSP;			//Vol
	pObjt->filterVol_Ia = (2*pObjt->filterVol6*pObjt->Gain1 - pObjt->ExVref)*pObjt->calibIa + pObjt->Offset_Ia;
	pObjt->filterVol_If =(2*pObjt->filterVol7*pObjt->Gain1 - pObjt->ExVref)*pObjt->calibIf + pObjt->Offset_If;
	pObjt->filterVol_Vbus = (2*pObjt->filterVol14*pObjt->Gain1 - pObjt->ExVref)*pObjt->calibVbus + pObjt->Offset_Vbus;
	
	//Transform to SI Unit
	pObjt->Iak = (pObjt->filterVol_Ia/pObjt->ExVref)*pObjt->MaxIa;			//Ampere
	pObjt->Torque = pObjt->Iak * pObjt->IaToTorqueGain;									//Nm
	pObjt->If = (pObjt->filterVol_If/pObjt->ExVref)*pObjt->MaxIf;				//Ampere
	pObjt->Vbus = (pObjt->filterVol_Vbus/pObjt->ExVref)*pObjt->MaxVbus; //Vol
	
	pObjt->TorqueSP = (pObjt->filterVol_TorqueSP/pObjt->MaxVol_TorqueSP)*10*pMParas->kPhiConst;	
	if(pObjt->TorqueSP > pObjt->MaxTorqueSP)
	{
		pObjt->TorqueSP = pObjt->MaxTorqueSP;
	}
	pObjt->IaSP = pObjt->TorqueSP/pObjt->IaToTorqueGain;								//Ampere
	pObjt->SpeedSP = (pObjt->filterVol_SpeedSP/pObjt->MaxVol_SpeedSP)*pObjt->MaxSpeedSP;	//rpm
	if(pObjt->SpeedSP > pObjt->MaxSpeedSP)
	{
		pObjt->SpeedSP = pObjt->MaxSpeedSP;
	}
	//Transform to Per Unit (%)
	pObjt->PU_TorqueSP = (pObjt->TorqueSP/pObjt->MaxTorqueSP)*100;			//Percent
	pObjt->PU_IfSP = (pObjt->IfSP/pObjt->MaxIfSP)*100;									//Percent
	pObjt->PU_If = (pObjt->If/pObjt->MaxIfSP)*100;											//Percent
	pObjt->PU_SpeedSP = (pObjt->SpeedSP/pObjt->MaxSpeedSP)*100;					//Percent
	pObjt->PU_Ia = (pObjt->Iak/pObjt->MaxIaSP)*100;
	
	//Compute the differential of Ia
	pObjt->dIak_dt = (pObjt->Iak - pObjt->Iak_1)/0.0005;
	pObjt->filtered_dIak_dt = fLPF_Run(&stLPF_Var[5], pObjt->dIak_dt);
	pObjt->Iak_1 = pObjt->Iak;
}
/**********************************************************************************************/
void vDAC_Init(DAC_OUT *pAO)																				//DAC data initialize
{
	pAO->AO1 = 0;
	pAO->AO2 = 0;
	pAO->Offset1 = 2.5;
	pAO->Offset2 = 2.5;
	pAO->VO1 = 0.0;
	pAO->VO2 = 0.0;
	pAO->Range1 = 5.0;
	pAO->Range2 = 5.0;
}
/**********************************************************************************************/
void vDAC_Write(DAC_OUT *pAO, uint8_t DAC_Channel, float Vout)			//DAC write, Vout = -2.5V->2.5V
{
	switch(DAC_Channel)
	{
		case 1:
		{
			pAO->VO1 = Vout;
			pAO->AO1 = ((pAO->VO1 + pAO->Offset1)/pAO->Range1)*4095;
			HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, pAO->AO1);		
			break;
		}
		case 2:
		{
			pAO->VO2 = Vout;
			pAO->AO2 = ((pAO->VO2 + pAO->Offset2)/pAO->Range2)*4095;
			HAL_DAC_SetValue(&hdac, DAC_CHANNEL_2, DAC_ALIGN_12B_R, pAO->AO2);
			break;
		}
	}
}
/**********************************************************************************************/
void vRamp_Config(RAMP *pRamp, uint16_t Tacc_Sec, uint16_t Tdec_Sec, float inputMin, float inputMax, float Ts)
{
	pRamp->inputMax = inputMax;
	pRamp->inputMin = inputMin;
	pRamp->T_Acc = Tacc_Sec;
	pRamp->T_Dec = Tdec_Sec;
	
	//Compute the set count - 10ms resolution
	pRamp->set_AccCnt = pRamp->T_Acc/Ts;	
	pRamp->set_DecCnt = pRamp->T_Dec/Ts;
	//Compute the step size
	pRamp->delta_Acc = (float)(pRamp->inputMax - pRamp->inputMin)/pRamp->set_AccCnt;
	pRamp->delta_Dec = (float)(pRamp->inputMax - pRamp->inputMin)/pRamp->set_DecCnt;
	
	pRamp->SPk = 0;
	pRamp->Output = 0;
}
/**********************************************************************************************/
float fRampFcnGen(float SP, RAMP *pRamp)
{
	pRamp->SPk = SP;
	//Detect the state
	if(pRamp->SPk > pRamp->Output + pRamp->delta_Acc)
	{
		pRamp->State = ACC;
	}
	else if (pRamp->SPk < pRamp->Output - pRamp->delta_Dec)
	{
		pRamp->State = DEC;
	}
	else
	{
		pRamp->State = RUN;
	}
	//Generate output
	if(pRamp->State == ACC)
	{
		pRamp->Output = pRamp->Output + pRamp->delta_Acc;
		if(pRamp->Output >= pRamp->SPk)	//End the acceleration
		{
			pRamp->Output = pRamp->SPk;
			pRamp->State = RUN; 					
		}
	}
	else if(pRamp->State == DEC)
	{
		pRamp->Output = pRamp->Output - pRamp->delta_Dec;
		if(pRamp->Output <= pRamp->SPk)	//End the deceleration
		{
			pRamp->Output = pRamp->SPk;
			pRamp->State = RUN; 					
		}
	}
	else if(pRamp->State == RUN)
	{
		pRamp->Output = pRamp->SPk;
	}
	return pRamp->Output;
}