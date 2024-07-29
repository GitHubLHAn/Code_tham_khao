#include "main.h"

extern UART_HandleTypeDef huart4;
extern UART_HandleTypeDef huart5;
extern UART_HandleTypeDef huart2;

//NEXTION HMI
NEXTION NEXVar;

/**********************************************************************************************/
void vNEXTION_Init(NEXTION *pNEXVar)
{
	pNEXVar->UpdateEN = 1;
}
/**********************************************************************************************/
void vNEXTION_DisplayHandle(adc_Objt *pADC, machine_Control *pStateMachine, cPI *pSpeed, cPI *pIa, 
	RAMP *pRAMP_Ia, RAMP *pRAMP_Speed, QEI *pQEI, machine_Parameter *pMParas, NEXTION *pNEXVar)
{
	float R = 0;
	if(pNEXVar->UpdateEN == 1)
	{
		//Page number and Xfloat
		pNEXVar->page1_x0_IaFbx10 = pADC->Iak*10;
		pNEXVar->page1_x2_IExcFbx10 = pADC->If*10;
		//pNEXVar->page1_n0_SpeedFB = pQEI->rpm;
		pNEXVar->page1_n0_SpeedFB = pStateMachine->EsSpeed;
		pNEXVar->page1_n2_Vdc = pADC->Vbus;
		
		pNEXVar->page1_x1_IaSPx10 = pStateMachine->Ia_SP*10;
		pNEXVar->page1_x3_IExcSPx10 = pStateMachine->If_SP*10;
		pNEXVar->page1_n1_SpeedSP = pStateMachine->Speed_SP;
		
		//Display Ia on Gauge z0
		R = 270.0/pADC->MaxIa;
		if(pADC->Iak >= 0 && pADC->Iak < 45.0/R)	
		{
			pNEXVar->page1_z0_IaFbx10 = 315 + pADC->Iak*R;
		}
		else if(pADC->Iak >= 45.0/R)
		{
			pNEXVar->page1_z0_IaFbx10 = pADC->Iak*R - 45;
		}
		//Display IExc on Gauge z1
		R = 270.0/pADC->MaxIf;
		if(pADC->If >= 0 && pADC->If < 45.0/R)	
		{
			pNEXVar->page1_z1_IExcFbx10 = 315 + pADC->If*R;
		}
		else if(pADC->If >= 45.0/R)
		{
			pNEXVar->page1_z1_IExcFbx10 = pADC->If*R - 45;
		}
		//Display rpm on Gauge z2
		R = 270.0/pMParas->Speed_max;
		/*if(pQEI->rpm >= 0 && pQEI->rpm < 45.0/R)	
		{
			pNEXVar->page1_z2_SpeedFB = 315 + pQEI->rpm*R;
		}
		else if(pQEI->rpm>= 45.0/R)
		{
			pNEXVar->page1_z2_SpeedFB = pQEI->rpm*R - 45;
		}*/	
		if(pStateMachine->EsSpeed >= 0 && pStateMachine->EsSpeed < 45.0/R)	
		{
			pNEXVar->page1_z2_SpeedFB = 315 + pStateMachine->EsSpeed*R;
		}
		else if(pStateMachine->EsSpeed>= 45.0/R)
		{
			pNEXVar->page1_z2_SpeedFB = pStateMachine->EsSpeed*R - 45;
		}
		//Display Vdc on Gauge z3
		R = 270.0/pADC->MaxVbus;
		if(pADC->Vbus >= 0 && pADC->Vbus < 45.0/R)	
		{
			pNEXVar->page1_z3_Vdc = 315 + pADC->Vbus*R;
		}
		else if(pADC->Vbus >= 45.0/R)
		{
			pNEXVar->page1_z3_Vdc = pADC->Vbus*R - 45;
		}
		
		if(pStateMachine->Ia_State == ALARM_H)
		{
			vNEXTION_SendString("page1.t6", "IaALARM");
		}
		else if(pStateMachine->Ia_State == FAULT_H)
		{
			vNEXTION_SendString("page1.t6", "IaFAULT");
		}
		else{
			vNEXTION_SendString("page1.t6", "IaNORMAL");
		}
		/*Page 2*/
		pNEXVar->page2_n0_DI0 = HAL_GPIO_ReadPin(DI0_GPIO_Port, DI0_Pin);
		pNEXVar->page2_n1_DI1 = HAL_GPIO_ReadPin(DI1_GPIO_Port, DI1_Pin);
		pNEXVar->page2_n2_DI2 = HAL_GPIO_ReadPin(DI2_GPIO_Port, DI2_Pin);
		pNEXVar->page2_n3_DI3 = HAL_GPIO_ReadPin(DI3_GPIO_Port, DI3_Pin);
		pNEXVar->page2_n4_DI4 = HAL_GPIO_ReadPin(DI4_GPIO_Port, DI4_Pin);
		pNEXVar->page2_n5_DI5 = HAL_GPIO_ReadPin(DI5_GPIO_Port, DI5_Pin);
		pNEXVar->page2_n6_DI6 = HAL_GPIO_ReadPin(DI6_GPIO_Port, DI6_Pin);
		pNEXVar->page2_n16_FAULT = HAL_GPIO_ReadPin(EXTI2_Fault_GPIO_Port, EXTI2_Fault_Pin);
		
		pNEXVar->page2_n7_DO0 = HAL_GPIO_ReadPin(DO0_GPIO_Port, DO0_Pin);
		pNEXVar->page2_n8_DO1 = HAL_GPIO_ReadPin(DO1_GPIO_Port, DO1_Pin);
		pNEXVar->page2_n9_DO2 = HAL_GPIO_ReadPin(DO2_GPIO_Port, DO2_Pin);
		pNEXVar->page2_n10_DO3 = HAL_GPIO_ReadPin(DO3_GPIO_Port, DO3_Pin);
		pNEXVar->page2_n11_DO4 = HAL_GPIO_ReadPin(DO4_GPIO_Port, DO4_Pin);
		pNEXVar->page2_n12_DB = HAL_GPIO_ReadPin(D_Braking_GPIO_Port, D_Braking_Pin);
		pNEXVar->page2_n13_EN = HAL_GPIO_ReadPin(Enable_GPIO_Port, Enable_Pin);
		pNEXVar->page2_n14_RST = HAL_GPIO_ReadPin(Reset_GPIO_Port, Reset_Pin);
		pNEXVar->page2_n15_BYPASS = HAL_GPIO_ReadPin(Bypass_GPIO_Port, Bypass_Pin);
		pNEXVar->page2_n17_SP2 = pStateMachine->Speed_SP;		//rpm
		pNEXVar->page2_n18_RAMP2= pRAMP_Speed->Output;			//rpm
		pNEXVar->page2_n19_QEI_DIR = pQEI->Direction; 
		pNEXVar->page2_n20_QEI_CNT = pQEI->Cnt_k;
		pNEXVar->page2_x0_SP1 = pStateMachine->Ia_SP*10;		//Ampere
		pNEXVar->page2_x1_RAMP1 = pRAMP_Ia->Output*10;			//Ampere
		pNEXVar->page2_x2_LaxDia_dt = pMParas->La*pADC->filtered_dIak_dt*10;
		
		//HMI Display Control - Page 1
		vNEXTION_SendVal("page1.x0", pNEXVar->page1_x0_IaFbx10, DATA_XFLOAT);
		vNEXTION_SendVal("page1.x2", pNEXVar->page1_x2_IExcFbx10, DATA_XFLOAT);
		vNEXTION_SendVal("page1.n0", pNEXVar->page1_n0_SpeedFB, DATA_INT);
		vNEXTION_SendVal("page1.n2", pNEXVar->page1_n2_Vdc, DATA_INT);
		
		
		vNEXTION_SendVal("page1.z0", pNEXVar->page1_z0_IaFbx10, DATA_INT);
		vNEXTION_SendVal("page1.z1", pNEXVar->page1_z1_IExcFbx10, DATA_INT);
		vNEXTION_SendVal("page1.z2", pNEXVar->page1_z2_SpeedFB, DATA_INT);
		vNEXTION_SendVal("page1.z3", pNEXVar->page1_z3_Vdc, DATA_INT);
		
		vNEXTION_SendVal("page1.x1", pNEXVar->page1_x1_IaSPx10, DATA_XFLOAT);	
		vNEXTION_SendVal("page1.x3", pNEXVar->page1_x3_IExcSPx10, DATA_XFLOAT);	
		vNEXTION_SendVal("page1.n1", pNEXVar->page1_n1_SpeedSP, DATA_INT);	
		
		if(pStateMachine->chState == CHARGE)
		{
			vNEXTION_SendString("page1.t14", "CHARGE");
		}
		else if(pStateMachine->chState == DOL)
		{
			vNEXTION_SendString("page1.t14", "DOL");
		}
		
		if(pStateMachine->mState == mSTANDSTILL)
		{
			vNEXTION_SendString("page1.t10", "IDLE");
		}
		else if(pStateMachine->mState == mSTOP)
		{
			vNEXTION_SendString("page1.t10", "STOP");
		}
		else if(pStateMachine->mState == mRUN)
		{
			vNEXTION_SendString("page1.t10", "RUN");
		}
		else if(pStateMachine->mState == mFAULT)
		{
			vNEXTION_SendString("page1.t10", "FAULT");
		}
		
		if(pStateMachine->ctrMode == OPEN_LOOP)
		{
			vNEXTION_SendString("page1.t8", "OPENLOOP");
			vNEXTION_SendString("page3.t0", "Va(%)");
			if(pStateMachine->sysLOCAL_REMOTE == REMOTE)
			{
				vNEXTION_SendString("page3.bt1", "OPENLOOP");
				vNEXTION_SendVal("page3.bt1", 0, DATA_INT);	
			}
		}
		else if(pStateMachine->ctrMode == TORQUE)
		{
			vNEXTION_SendString("page1.t8", "TORQUE");
			vNEXTION_SendString("page3.t0", "Spd_SP(%)");
			if(pStateMachine->sysLOCAL_REMOTE == REMOTE)
			{
				vNEXTION_SendString("page3.bt1", "TORQUE");
				vNEXTION_SendVal("page3.bt1", 1, DATA_INT);
			}
		}
		else if(pStateMachine->ctrMode == SPEED)
		{
			vNEXTION_SendString("page1.t8", "SPEED");
			vNEXTION_SendString("page3.t0", "Spd_SP(%)");
			if(pStateMachine->sysLOCAL_REMOTE == REMOTE)
			{
				vNEXTION_SendString("page3.bt1", "SPEED");
				vNEXTION_SendVal("page3.bt1", 2, DATA_INT);
			}
		}
		
		if(pStateMachine->sysLOCAL_REMOTE == LOCAL)
		{
			vNEXTION_SendString("page1.t7", "LOCAL");
		}
		else if(pStateMachine->sysLOCAL_REMOTE == REMOTE)
		{
			vNEXTION_SendString("page1.t7", "REMOTE");
		}
		
		if(pQEI->Direction == 0)				//Forward
		{
			vNEXTION_SendString("page1.t9", "FWD");
		}
		else
		{
			vNEXTION_SendString("page1.t9", "REV");
		}
		
		//HMI Display Control - Page 2
		vNEXTION_SendVal("page2.n0", pNEXVar->page2_n0_DI0, DATA_INT);
		vNEXTION_SendVal("page2.n1", pNEXVar->page2_n1_DI1, DATA_INT);
		vNEXTION_SendVal("page2.n2", pNEXVar->page2_n2_DI2, DATA_INT);
		vNEXTION_SendVal("page2.n3", pNEXVar->page2_n3_DI3, DATA_INT);
		vNEXTION_SendVal("page2.n4", pNEXVar->page2_n4_DI4, DATA_INT);
		vNEXTION_SendVal("page2.n5", pNEXVar->page2_n5_DI5, DATA_INT);
		vNEXTION_SendVal("page2.n6", pNEXVar->page2_n6_DI6, DATA_INT);
		vNEXTION_SendVal("page2.n16", pNEXVar->page2_n16_FAULT, DATA_INT);

		vNEXTION_SendVal("page2.n7", pNEXVar->page2_n7_DO0, DATA_INT);
		vNEXTION_SendVal("page2.n8", pNEXVar->page2_n8_DO1, DATA_INT);
		vNEXTION_SendVal("page2.n9", pNEXVar->page2_n9_DO2, DATA_INT);
		vNEXTION_SendVal("page2.n10", pNEXVar->page2_n10_DO3, DATA_INT);
		vNEXTION_SendVal("page2.n11", pNEXVar->page2_n11_DO4, DATA_INT);
		
		vNEXTION_SendVal("page2.n12", pNEXVar->page2_n12_DB, DATA_INT);
		vNEXTION_SendVal("page2.n13", pNEXVar->page2_n13_EN, DATA_INT);
		vNEXTION_SendVal("page2.n14", pNEXVar->page2_n14_RST, DATA_INT);
		vNEXTION_SendVal("page2.n15", pNEXVar->page2_n15_BYPASS, DATA_INT);
		
		vNEXTION_SendVal("page2.n17", pNEXVar->page2_n17_SP2, DATA_INT);
		vNEXTION_SendVal("page2.n18", pNEXVar->page2_n18_RAMP2, DATA_INT);
		vNEXTION_SendVal("page2.n19", pNEXVar->page2_n19_QEI_DIR, DATA_INT);
		vNEXTION_SendVal("page2.n20", pNEXVar->page2_n20_QEI_CNT, DATA_INT);
		
		vNEXTION_SendVal("page2.x0", pNEXVar->page2_x0_SP1, DATA_XFLOAT);
		vNEXTION_SendVal("page2.x1", pNEXVar->page2_x1_RAMP1, DATA_XFLOAT);
		vNEXTION_SendVal("page2.x2", pNEXVar->page2_x2_LaxDia_dt, DATA_XFLOAT);
		//HMI Display Control - Page 3
		vNEXTION_SendVal("page3.n0", pNEXVar->page3_n0_PU_SPEED_SP, DATA_INT);
		vNEXTION_SendVal("page3.n1", pNEXVar->page3_n1_PU_Te_SP, DATA_INT);
		vNEXTION_SendVal("page3.n2", pNEXVar->page3_n2_PU_IF_SP, DATA_INT);
		vNEXTION_SendVal("page3.h0", pNEXVar->page3_h0_PU_SPEED_SP, DATA_INT);
		vNEXTION_SendVal("page3.h1", pNEXVar->page3_h1_PU_Te_SP, DATA_INT);
		vNEXTION_SendVal("page3.h2", pNEXVar->page3_h2_PU_IF_SP, DATA_INT);
		
		vNEXTION_SendVal("page3.n3", pQEI->PU_rpm, DATA_INT);
		vNEXTION_SendVal("page3.n4", pADC->PU_Ia, DATA_INT);
		vNEXTION_SendVal("page3.n5", pADC->PU_If, DATA_INT);
	}
}
/**********************************************************************************************/
void vNEXTION_RecvRetNumHandle(uint8_t RetNum)
{	
	HAL_GPIO_WritePin(W_R_4_GPIO_Port, W_R_4_Pin,GPIO_PIN_SET);			 //RS485 In Transmit mode
	HAL_Delay(1);
	//HAL_UART_Transmit(&huart4, &RetNum, 1, 100);
	HAL_Delay(1);
	HAL_GPIO_WritePin(W_R_4_GPIO_Port, W_R_4_Pin,GPIO_PIN_RESET);			//RS485 In Receive mode
}
/**********************************************************************************************/
/*
 * Send command to Nextion.
 *
 * @param cmd - the string of command.
 */
void vNEXTION_SendCmd(char *cmd, uint8_t val)
{ 
	char buf[50];   
	uint8_t hmi_EndCmd[3] = {0xff, 0xff, 0xff};
  int len = sprintf(buf, "%s=%d",cmd,val);
	HAL_UART_Transmit(&huart5, (uint8_t *)buf, len, 1000);
	HAL_UART_Transmit(&huart5, hmi_EndCmd, 3, 1000);
}
/**********************************************************************************************/
void vNEXTION_SendPCO(char *ID, int32_t Val)
{
	char buf[50];   
	uint8_t hmi_EndCmd[3] = {0xff, 0xff, 0xff};
  int len = sprintf(buf, "%s.pco=%d",ID,Val);
	HAL_UART_Transmit(&huart5, (uint8_t *)buf, len, 1000);
	HAL_UART_Transmit(&huart5, hmi_EndCmd, 3, 1000);
}
/**********************************************************************************************/
void vNEXTION_SendVal(char *ID, int Val, uint8_t dataType)
{
	char buf[50], buf1[20];
	uint8_t hmi_EndCmd[3] = {0xff, 0xff, 0xff};
	
	int len = sprintf(buf, "%s.val=%d", ID, Val);
	
	HAL_UART_Transmit(&huart5, (uint8_t *)buf, len, 1000);				//Send data
	HAL_UART_Transmit(&huart5, hmi_EndCmd,3, 1000);								//Send End Command
	if(dataType == DATA_XFLOAT)	//x10 data
	{
		if(Val >= 0 && Val < 100)
		{
			len = sprintf(buf1, "%s.vvs0=%d", ID, 1);									//Number of Digit in the Left
			HAL_UART_Transmit(&huart5, (uint8_t *)buf1, len, 1000);
			HAL_UART_Transmit(&huart5, hmi_EndCmd,3, 1000);
		}
		else if(100<= Val && Val < 1000)
		{
			len = sprintf(buf1, "%s.vvs0=%d", ID, 2);									//Number of Digit in the Left
			HAL_UART_Transmit(&huart5, (uint8_t *)buf1, len, 1000);
			HAL_UART_Transmit(&huart5, hmi_EndCmd,3, 1000);
		}
		else if(Val >=1000 && Val < 10000)
		{
			len = sprintf(buf1, "%s.vvs0=%d", ID, 3);									//Number of Digit in the Left			
			HAL_UART_Transmit(&huart5, (uint8_t *)buf1, len, 1000);
			HAL_UART_Transmit(&huart5, hmi_EndCmd,3, 1000);
		}
		else if(Val >=10000 && Val <100000)
		{
			len = sprintf(buf1, "%s.vvs0=%d", ID, 4);									//Number of Digit in the Left			
			HAL_UART_Transmit(&huart5, (uint8_t *)buf1, len, 1000);
			HAL_UART_Transmit(&huart5, hmi_EndCmd,3, 1000);
		}
	}
}
/**********************************************************************************************/
void vNEXTION_SendString(char *ID, char *string)
{
	char buf[50];
	uint8_t hmi_EndCmd[3] = {0xff, 0xff, 0xff};
	int len = sprintf(buf, "%s.txt=\"%s\"", ID, string);
	HAL_UART_Transmit(&huart5, (uint8_t *)buf, len, 1000);
	HAL_UART_Transmit(&huart5, hmi_EndCmd, 3, 100);
}