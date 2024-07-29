#include "main.h"
/**********************************************************************************************/
void vPI_Init(cPI *pPI, float KI, float KP, float Ts, float MaxSP, float Usat)
{
	pPI->Uk_1 = 0; pPI->Uk = 0; pPI->Usk_1 = 0;
	pPI->Ek = 0; pPI->Ek_1 = 0; 
	pPI->Esk = 0; pPI->Esk_1 = 0; 
	
	pPI->Umax = Usat; pPI->Umin = -Usat;
	//Sampling time (sec)
	pPI->Ts = Ts;	
	//pPI->Kp = KP; 
	pPI->Kp = 0.5*pPI->Umax/MaxSP;
	pPI->Ks = pPI->Kp;
	//Integral time (sec)
	pPI->KI = KI; 
	if(KI <=0)
	{
		pPI->Mode = P;		//Proportional only
		pPI->TI = 0;
		pPI->D = 0;
	}
	else
	{
		pPI->Mode = P_I;	//Proportional-Integral
		pPI->TI = pPI->Kp/KI;										
		pPI->D = 1 - pPI->Ts/pPI->TI;
	}
}
/**********************************************************************************************/
float fPI_Run(cPI *pPI, float Ref, float Fb)
{
	pPI->Ek = Ref - Fb;
	switch(pPI->Mode)
	{
		case P:
		{
			pPI->Uk = pPI->Kp*pPI->Ek;
			if(pPI->Uk > pPI->Umax)
					pPI->Usk = pPI->Umax;
			else if(pPI->Uk < pPI->Umin)
					pPI->Usk = pPI->Umin;
			else 
					pPI->Usk = pPI->Uk;
			break;
		}
		case P_I:
		{
			pPI->Esk_1 = pPI->Ek_1 + (1/pPI->Ks)*(pPI->Usk_1-pPI->Uk_1);
			pPI->Uk = pPI->Usk_1 +pPI->Kp*(pPI->Ek-pPI->D*pPI->Esk_1);

			if(pPI->Uk > pPI->Umax)
					pPI->Usk = pPI->Umax;
			else if(pPI->Uk < pPI->Umin)
					pPI->Usk = pPI->Umin;
			else 
					pPI->Usk = pPI->Uk;
			//update data
			pPI->Ek_1 = pPI->Ek;
			pPI->Uk_1 = pPI->Uk;
			pPI->Usk_1 = pPI->Usk;
			break;
		}
	}
	return pPI->Usk;
}

