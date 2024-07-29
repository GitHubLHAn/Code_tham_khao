/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
/**********************************************************************************************/
//Analog output
typedef struct 
{
	float Range1, Range2;				//Max range of the output voltage
	float Offset1, Offset2;			
	float VO1, VO2;							//Output voltage: -5V -> 5V
	uint16_t AO1, AO2;					//Value to be sent to the DAC
}DAC_OUT;
/**********************************************************************************************/
//Quadrature Encoder
typedef struct
{
	int16_t ppr;								//Pulse per round of the Encoder
	float Ts;										//Sampling time (sec)
	int16_t Cnt_k, Cnt_k_1;			//Value of counter at time instance k and k_1
	uint16_t Cnt_Ovf; 					//Overflow counter
	int16_t rpm;								//Round per minute of the motor - need to be calculated
	int8_t Direction;						//Direction of the motor
	int8_t PU_rpm;
	int8_t FirstCnt;			
}QEI;
/**********************************************************************************************/
typedef enum
{
	ACC, 
	RUN, 
	DEC
}RAMP_STATE;
/**********************************************************************************************/
//Ramp Fcn generator
typedef struct
{
	float SPk;												//Current and past value of setpoint
	float inputMax, inputMin;
	uint16_t T_Acc, T_Dec;						//Ramp up and down time (s)
	float delta_Acc, delta_Dec;				//step size in ACC and DEC mode
	uint16_t set_AccCnt, set_DecCnt;	//Set count - 10ms Resolution
	RAMP_STATE State;
	float Output;
}RAMP;

/**********************************************************************************************/
//PI Controller
typedef struct
{
	float Kp, KI, TI;								//Controller parameters	
	float Ks, D;										//Ks: Anti-windup Gain; D = 1-Ts/TI
	float Umax, Umin, Usk, Usk_1, Uk, Uk_1;	//Control signal
	float Ek, Ek_1, Esk, Esk_1;			//Error
	float Ts;												//Sampling time
	uint8_t Mode;										//Mode = 0: P, Mode = 1: PI
}cPI;
/**********************************************************************************************/
//Second-Order Unity-Gain Low Pass Filter
typedef struct
{
	float Ts;							//Sampling time
	float Fc; 						//Conrner frequency (Hz)
	float a1, a2, b1, b2;	//Parameters of the filter
	float alpha;
	float yk, yk_1, yk_2, uk, uk_1;
}LPF;
/**********************************************************************************************/
//Analog input
typedef struct 
{
	uint16_t data[5];	
	float calibVol0, calibVol1, calibVol6, calibVol7, calibVol14;	
	float rawVol0, rawVol1, rawVol6, rawVol7, rawVol14;
	float filterVol0, filterVol1, filterVol6, filterVol7, filterVol14;
	float Gain1, Gain2, ExVref;
	float Offset_Ia, Offset_If, Offset_Vbus;			//Zero point of corresponding channel
	float filterVol_Vbus, filterVol_Ia, filterVol_If, filterVol_SpeedSP, filterVol_TorqueSP;
	
	float Vbus, If, Torque;
	float Iak, Iak_1, dIak_dt, filtered_dIak_dt;
	float PU_Vbus, PU_Torque, PU_If, PU_Ia;
	float IfSP, IaSP, SpeedSP, TorqueSP;
	float MaxVol_TorqueSP, MaxVol_SpeedSP;
	
	float calibIa, calibIf, calibSpeedSP, calibTorqueSP, calibVbus;
	float MaxVbus, MaxIa, MaxIf, MaxTorque; 					//Max range of Sensors
	float MaxIfSP, MaxIaSP, MaxSpeedSP, MaxTorqueSP;						//Max range of reference
	float PU_IfSP, PU_IaSP, PU_SpeedSP, PU_TorqueSP;	//PU is computed based on Max range of reference
	
	float IaToTorqueGain;
	float adcVref;
}adc_Objt;
/**********************************************************************************************/
typedef enum  
{
	Low,
	High, 
	Rising, 
	Falling
}gpio_Event;
/**********************************************************************************************/
typedef enum
{
	mSTANDSTILL,
	mSTOP,  
	mRUN, 
	mFAULT
}machine_State;
/**********************************************************************************************/
typedef enum
{
	OPEN_LOOP,
	TORQUE, 
	SPEED
}control_Mode;
/**********************************************************************************************/
typedef enum
{
	CHARGE, 		//Charging the DC bus Capacitor via Resistor	
	DOL					//Direct OnLine
}charge_State;
/**********************************************************************************************/
typedef enum
{
	NORMAL, 		
	ALARM_L,
	ALARM_H,
	FAULT_L,
	FAULT_H, 
	EXT_FAULT
}OP_State;		//Operation state: NORMAL, ALARM, FAULT
/**********************************************************************************************/
//Machine control
typedef struct
{
	control_Mode ctrMode;				//cMode = SPEED or Torque
	machine_State mState;				//State = STOP, RUN, or FAULT
	charge_State chState;				//cState = CHARGE or DOL
	uint8_t sysENABLE, sysLOCAL_REMOTE;
	
	float If_H_Alarm, If_H_Fault;
	float If_L_Alarm, If_L_Fault;
	float Ia_H_Alarm, Ia_H_Fault;
	float Speed_H_Alarm, Speed_H_Fault;
	float Vdc_H_Alarm, Vdc_H_Fault , Vdc_L_Alarm, Vdc_L_Fault;
	
	OP_State If_State, Ia_State, Vdc_State, Speed_State, EXT_State;
	float Vdc_Nom;							//Nominal value of DC bus voltage
	float Vdc_Hi, Vdc_Lo;				//High and Low Level of the DC bus voltage, handle the charge state
	float Speed_SP, Te_SP, Ia_SP, If_SP, Vol_SP;
	int Dir;										//1: Forward, -1: Reverse
	int REVERSE_ON;							//1: REVERSE, 0: NORMAL
	int isrCnt;
	float EsSpeed;							//Estimation of speed
	float SpeedLimit, KLimit;		//Used to limit speed in Torque Mode
	float SpeedEs;							//SpeedEs is the output of saturation
	float Vcontrol;							//Final control signal which is send to PWM module
}machine_Control;
/**********************************************************************************************/
//Machine parameters
typedef struct
{
	float Ia_rated, If_rated, Va_rated;
	float Speed_rated, Speed_max;
	float La, Ra;
	float kPhiConst;
	float Power_rated, Torque_rated;
}machine_Parameter;
/**********************************************************************************************/
//GPIO
typedef struct 
{
	GPIO_PinState dataK, dataK_1;	
	gpio_Event event;
}gpio_Objt;
/**********************************************************************************************/
//Timer 
typedef struct 
{
	uint16_t En;	//Enable the timer
	uint16_t SV;	//Set value
	uint16_t PV; 	//Present value
	uint16_t Output;
	uint16_t OvfCnt;
}timer_Objt;	
/**********************************************************************************************/
//usart data
typedef struct
{
	//Buffer
	uint8_t rxBuff[100];
	uint8_t txBuff[100];
	uint8_t NEXTION_Buff[10];
	//Header
	uint8_t frameLength, groupID, senderAddr, receiverAddr, messType, dataLength;
	uint16_t messCount;
	//Data payload
	
	//State flow 
	uint8_t STATE;
	uint8_t RxFlag;
	uint8_t rxByte, rxPointer;
	uint8_t NEXTION_Pointer, NEXTION_Flag, NEXTION_RetCode;	
	uint16_t timeOut; 
}usartData;
/**********************************************************************************************/
typedef struct
{
	/*Page 1*/
	int page1_z0_IaFbx10, page1_z1_IExcFbx10, page1_z2_SpeedFB, page1_z3_Vdc;
	int page1_x0_IaFbx10, page1_x2_IExcFbx10, page1_n0_SpeedFB, page1_n2_Vdc;
	int page1_x1_IaSPx10, page1_x3_IExcSPx10, page1_n1_SpeedSP;
	
	char page1_t14_ChargeState[20];
	char page1_t8_ControlMode[20];
	char page1_t10_MachineState[20];
	char page1_t6_ErrorMess[50];
	char page1_t7_LOCAL_REMOTE[50];
	
	/*Page 2*/
	uint16_t page2_n0_DI0, page2_n1_DI1, page2_n2_DI2, page2_n3_DI3;
	uint16_t page2_n4_DI4, page2_n5_DI5, page2_n6_DI6, page2_n16_FAULT;
	uint16_t page2_n7_DO0, page2_n8_DO1, page2_n9_DO2, page2_n10_DO3, page2_n11_DO4;
	uint16_t page2_n12_DB, page2_n13_EN, page2_n14_RST, page2_n15_BYPASS;
	uint16_t page2_n17_SP2, page2_n18_RAMP2, page2_x0_SP1, page2_x1_RAMP1;
	uint16_t page2_n19_QEI_DIR, page2_n20_QEI_CNT;
	int page2_x2_LaxDia_dt;
	/*Page 3*/
	uint16_t page3_n0_PU_SPEED_SP, page3_n1_PU_Te_SP, page3_n2_PU_IF_SP;
	uint16_t page3_h0_PU_SPEED_SP, page3_h1_PU_Te_SP, page3_h2_PU_IF_SP;
	uint16_t page3_n3_PU_SPEED_FB, page3_n4_PU_Te_FB, page3_n5_PU_IF_FB;
	/*Data payload*/
	uint32_t messCnt, rxVal;
	uint8_t UpdateEN;
}NEXTION;
/**********************************************************************************************/
/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
void Error_Handler(void);

/* USER CODE BEGIN EFP */
/*Safety chain handle*/
void vSafetyChainHandle(adc_Objt *pAdcObjt, machine_Control *pMControl, QEI *pQEI);
void vStateMachine(void);
void vStateMachineConfig(machine_Control *pMControl, machine_Parameter *pMParas);
void vMachineParameterConfig(machine_Parameter *pMParas);

void vLPF_Init(LPF *pLPF, uint16_t ConnerFrqHz, float samplingTime);	//Initialize LPF 
float fLPF_Run(LPF *pLPF, float input);																//Run LPF 

void vDIO_Init(void);			//Scan all input and setup output at startup
void vDI_Scan(void);			//Scan the proximity sensors in operation, every 10ms

gpio_Event eDI_EventDetect(GPIO_PinState k_Instance, GPIO_PinState k_1_Instance); //Check the even
void vDIO_Task(machine_Control *pMControl);

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);			//Timer interrupt handle

void vTim_Start(timer_Objt *ptim, uint16_t SV);				//Start timer 10ms
void vTim_Stop(timer_Objt *ptim);											//Reset timer 10ms
void vTim_Scan(timer_Objt *pTIM);
//Response to received message
void vUsart2_rxHandle(usartData *pRxData);													
void vUsart4_RxHandle(usartData *pRxData, usartData *pTxData, machine_Control *pMControl, adc_Objt *pAdc);
void vUsart5_rxHandle(usartData *pRxData, NEXTION *pNEXVar, machine_Control *pStateMachine);

void vUsart_TxHandle(adc_Objt *pADC, machine_Control *pStateMachine, 
	RAMP *pRAMP_Ia, RAMP *pRAMP_Speed, usartData *pTxData, uint8_t Port, uint8_t messType); //For Test Purpose

void vUsart2_TxHandle(adc_Objt *pADC, machine_Control *pStateMachine, 
	RAMP *pRAMP1, RAMP *pRAMP2, usartData *pTxData, uint8_t messType);//Process and send data to ESP32
void vUsart4_TxHandle(adc_Objt *pADC, machine_Control *pStateMachine, 
	RAMP *pRAMP1, RAMP *pRAMP2, usartData *pTxData, uint8_t messType);//Process and send data to RS485
void vUsart5_TxHandle(adc_Objt *pADC, machine_Control *pStateMachine, 
	RAMP *pRAMP1, RAMP *pRAMP2, usartData *pTxData, uint8_t messType);//Process and send data to HMI		

void vUsart_DataInit(usartData *pData);
void vUsart_Read(usartData *pRxData, uint8_t Port);									//Read Data from ESP32
void vUSART_RxTimeOutHandle(usartData *pRxData);

void vGPIO_Test(void); 	//For test only
void vStateMachineLoop(void);																				//State machine

void vPI_Init(cPI *pPI, float KI, float KP, float Ts, float MaxSP, float Usat);	//PI Controller with Anti-Windup
float fPI_Run(cPI *pPI, float Ref, float Fb);

void vAdc_Data_Handle(adc_Objt *pObjt, machine_Parameter *pMParas);
void vAdc_DataInit(adc_Objt *pObjt, machine_Parameter *pMParas);			//Initialize adc data

void vDAC_Init(DAC_OUT *pAO);																				//DAC data initialize
void vDAC_Write(DAC_OUT *pAO, uint8_t DAC_Channel, float Vout);			//DAC write, Vout = -2.5V->2.5V

//Config the Ramp Fcn genertor
void vRamp_Config(RAMP *pRamp, uint16_t Tacc_Sec, uint16_t Tdec_Sec, float inputMin, float inputMax, float Ts);
float fRampFcnGen(float SP, RAMP *pRamp);														//Ramp Fcn generator

//NEXTION HMI interface
void vNEXTION_Init(NEXTION *pNEXVar);
void vNEXTION_SendVal(char *ID, int32_t Val, uint8_t dataType);
void vNEXTION_SendString(char *ID, char *string);
void vNEXTION_SendCmd(char *cmd, uint8_t val);
void vNEXTION_SendPCO(char *ID, int32_t val);
void vNEXTION_RecvRetNumHandle(uint8_t RetNum);
void vNEXTION_DisplayHandle(adc_Objt *pADC, machine_Control *pStateMachine, cPI *pSpeed, cPI *pIa,
	RAMP *pRAMP_Ia, RAMP *pRAMP_Speed, QEI *pQEI, machine_Parameter *pMParas, NEXTION *pNEXVar);

//Quadrature Encoder
void vQEI_Config(QEI *pQEI);
void vRPM_Cal(QEI *pQEI, adc_Objt *pADC);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define DO4_Pin GPIO_PIN_15
#define DO4_GPIO_Port GPIOC
#define DO3_Pin GPIO_PIN_0
#define DO3_GPIO_Port GPIOC
#define DO2_Pin GPIO_PIN_1
#define DO2_GPIO_Port GPIOC
#define DO1_Pin GPIO_PIN_2
#define DO1_GPIO_Port GPIOC
#define DO0_Pin GPIO_PIN_3
#define DO0_GPIO_Port GPIOC
#define Bypass_Pin GPIO_PIN_5
#define Bypass_GPIO_Port GPIOC
#define Reset_Pin GPIO_PIN_0
#define Reset_GPIO_Port GPIOB
#define Enable_Pin GPIO_PIN_1
#define Enable_GPIO_Port GPIOB
#define EXTI2_Fault_Pin GPIO_PIN_2
#define EXTI2_Fault_GPIO_Port GPIOB
#define EXTI2_Fault_EXTI_IRQn EXTI2_IRQn
#define LDAC_Pin GPIO_PIN_12
#define LDAC_GPIO_Port GPIOB
#define TIM3_QEI_A_Pin GPIO_PIN_6
#define TIM3_QEI_A_GPIO_Port GPIOC
#define TIM3_QEI_B_Pin GPIO_PIN_7
#define TIM3_QEI_B_GPIO_Port GPIOC
#define QEI_Z_Pin GPIO_PIN_8
#define QEI_Z_GPIO_Port GPIOC
#define D_Braking_Pin GPIO_PIN_9
#define D_Braking_GPIO_Port GPIOC
#define W_R_4_Pin GPIO_PIN_15
#define W_R_4_GPIO_Port GPIOA
#define DI0_Pin GPIO_PIN_3
#define DI0_GPIO_Port GPIOB
#define DI1_Pin GPIO_PIN_4
#define DI1_GPIO_Port GPIOB
#define DI2_Pin GPIO_PIN_5
#define DI2_GPIO_Port GPIOB
#define DI3_Pin GPIO_PIN_6
#define DI3_GPIO_Port GPIOB
#define DI4_Pin GPIO_PIN_7
#define DI4_GPIO_Port GPIOB
#define DI5_Pin GPIO_PIN_8
#define DI5_GPIO_Port GPIOB
#define DI6_Pin GPIO_PIN_9
#define DI6_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */
#define MaxTIMER 20
#define NumOfDI 8
#define NumOfDO 5
#define adcUsedChannel 7
#define adcMAX 4096

#define FrameStart 0x7A
#define FrameEnd 0x7F
#define RS485_Write 1
#define RS485_Read 0

#define USART_PENDING 2
#define USART_READING 1
#define USART_WAIT 0
#define PI 3.1416
#define NumOfLPF 20

#define DAC_DutyCycle 1
#define DAC_IfRefChannel 2
#define DAC_MAX_Vout 2.5
#define LOCAL 1							//Local means HMI is used for control
#define REMOTE 0						//Remote means GPIO Port is used for control

#define MCU_ADDR 0x01				//Address of MCU in RS485 network

//NEXTION Return Data
#define NEX_RET_CMD_FINISHED            	0x01
#define NEX_RET_INVALID_CMD             	0x00
#define NEX_RET_INVALID_COMPONENT_ID    	0x02
#define NEX_RET_INVALID_PAGE_ID         	0x03
#define NEX_RET_INVALID_PICTURE_ID      	0x04
#define NEX_RET_INVALID_FONT_ID         	0x05
#define NEX_RET_INVALID_BAUD            	0x11
#define NEX_RET_INVALID_VARIABLE        	0x1A
#define NEX_RET_INVALID_OPERATION       	0x1B
#define NEX_RET_INVALID_ASSIGNMENT       	0x1C
#define NEX_RET_INVALID_EEPROM_OPERATION 	0x1D
#define NEX_RET_INVALID_PARA_QTY				 	0x1E
#define NEX_RET_TOO_LONG_VAR_NAME				 	0x23 
#define NEX_RET_TOO_BUFF_OVERFLOW				 	0x24 

#define RED 63488
#define YELLOW 65504
#define GREEN 2016
#define PURPLE 53594

#define GraphOffset 100
#define DATA_INT 0
#define DATA_XFLOAT 1

#define ESP32_Port 2
#define RS485_Port 4
#define HMI_Port 5

#define IaMAX 50
#define TeMAX 50
#define rpmMAX 2000

#define FORWARD 1
#define REVERSE -1

#define P 0
#define P_I 1
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
