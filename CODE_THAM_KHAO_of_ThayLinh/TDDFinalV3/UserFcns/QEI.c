#include "main.h"
//Quadrature Encoder
QEI stQEI_Var;	

extern TIM_HandleTypeDef htim3;
/**********************************************************************************************/
void vQEI_Config(QEI *pQEI, int ppr, int QEImode, float Ts) 
{
	//mode = 1, 2, or 4 (x1, x2, x4)
	//ppr: pulse per round
	//Ts: sampling time (second)
	pQEI->FirstCnt = 0;				//To ignore the first resulst
	pQEI->Cnt_k = 0; 
	pQEI->Cnt_k_1 = 0;
	pQEI->Cnt_Ovf = 0;
	pQEI->ppr = ppr*QEImode;		
	pQEI->PU_rpm = 0;
	pQEI->rpm = 0;
	pQEI->Ts = Ts;						//10ms
	pQEI->Direction = 0;			//Real Direction
}
/**********************************************************************************************/
void vRPM_Cal(QEI *pQEI, adc_Objt *pADC)
{
	//Detect the direction of the QEI	
	pQEI->Direction = __HAL_TIM_IS_TIM_COUNTING_DOWN(&htim3);
	//Read QEI Counter
	pQEI->Cnt_k = __HAL_TIM_GET_COUNTER(&htim3);
	//Compute the rotation speed (rpm)
	if(pQEI->Direction == 0)	//Up counting - RUN Forward
	{
		if(pQEI->FirstCnt == 1)
		{
			if(pQEI->Cnt_Ovf == 0)
			{
				pQEI->rpm = ((float)(pQEI->Cnt_k-pQEI->Cnt_k_1)/pQEI->Ts)*(60.0/(float)(pQEI->ppr)); 
			}
			else
			{
				//Auto-reload value of QEI Timer must be equal to ppr of the encoder
				pQEI->rpm = (pQEI->ppr - pQEI->Cnt_k_1) + pQEI->Cnt_k + pQEI->ppr*(pQEI->Cnt_Ovf-1);
				pQEI->rpm = pQEI->rpm * (60.0/(float)(pQEI->ppr)); 
				//Reset overflow counter
				pQEI->Cnt_Ovf  = 0;
			}
			pQEI->PU_rpm = ((float)(pQEI->rpm)/pADC->MaxSpeedSP)*100;	//Percent 
		}
		else	//Ignore results in first cycle
		{
			pQEI->FirstCnt = 1;
			pQEI->rpm = 0;
			pQEI->PU_rpm = 0;
		}
	}
	else if(pQEI->Direction == 1)	//Down-counting - RUN Reverse
	{
		if(pQEI->FirstCnt == 1)
		{
			if(pQEI->Cnt_Ovf == 0)
			{
				pQEI->rpm = ((float)(pQEI->Cnt_k_1-pQEI->Cnt_k)/pQEI->Ts)*(60.0/(float)(pQEI->ppr)); 
			}
			else
			{
				//Auto-reload value of QEI Timer must be equal to ppr of the encoder
				pQEI->rpm = (pQEI->ppr - pQEI->Cnt_k) + pQEI->Cnt_k_1 + pQEI->ppr*(pQEI->Cnt_Ovf-1);
				pQEI->rpm = -pQEI->rpm * (60.0/(float)(pQEI->ppr)); 
				//Reset overflow counter
				pQEI->Cnt_Ovf  = 0;
			}
			pQEI->PU_rpm = ((float)(pQEI->rpm)/pADC->MaxSpeedSP)*100;	//Percent 
		}
		else	//Ignore results in first cycle
		{
			pQEI->FirstCnt = 1;
			pQEI->rpm = 0;
			pQEI->PU_rpm = 0;
		}
	}
	pQEI->Cnt_k_1 = pQEI->Cnt_k;
}