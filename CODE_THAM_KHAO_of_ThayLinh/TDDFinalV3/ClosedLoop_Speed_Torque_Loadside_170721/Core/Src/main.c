/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

CAN_HandleTypeDef hcan;

CRC_HandleTypeDef hcrc;

DAC_HandleTypeDef hdac;

I2C_HandleTypeDef hi2c2;

IWDG_HandleTypeDef hiwdg;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;
TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim8;

UART_HandleTypeDef huart4;
UART_HandleTypeDef huart5;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
/*uart data*/
extern usartData usart2_Tx, usart2_Rx;		//ESP32 - Wifi
extern usartData usart4_Tx, usart4_Rx; 		//RS485
extern usartData usart5_Tx, usart5_Rx; 		//HMI
extern timer_Objt Tim_1ms[MaxTIMER];					//1ms software timer
extern timer_Objt Tim_10ms[MaxTIMER];				//10ms software timer
extern timer_Objt Tim_20ms[MaxTIMER];				//20ms software timer
extern timer_Objt Tim_100ms[MaxTIMER];			//100ms software timer
extern timer_Objt Tim_200ms[MaxTIMER];			//200ms software timer
extern timer_Objt Tim_1s[MaxTIMER];						//1s software timer
extern uint16_t tim1msTick, tim10msTick, tim20msTick, tim100msTick, tim200msTick, tim1sTick;
/*ADC data*/
extern adc_Objt adc_Objt_var;
extern uint16_t adc_data[7];
/*DAC data*/
extern DAC_OUT stDAC_Var;
//2nd Order Unity-Gain LPF
extern LPF stLPF_Var[NumOfLPF];
//PI speed controller
extern cPI stPI_Speed, stPI_Ia;		
//State machine
extern machine_Control stMControl_Var;
extern machine_Parameter stMachine_Var;
//RAMP Generator (Ampere with Ia, and rpm with Speed)
extern RAMP stRamp_Ia, stRamp_If, stRamp_Speed, stRamp_Vol;		
//Quadrature Encoder
extern QEI stQEI_Var;									
//NEXTION HMI
extern NEXTION NEXVar;
uint8_t FirstCycle = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_DAC_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_UART4_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_CRC_Init(void);
static void MX_IWDG_Init(void);
static void MX_I2C2_Init(void);
static void MX_UART5_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM5_Init(void);
static void MX_TIM8_Init(void);
static void MX_CAN_Init(void);
static void MX_TIM6_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	static uint32_t Cnt = 0;
	uint8_t i = 0;
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_DAC_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_UART4_Init();
  MX_USART2_UART_Init();
  MX_CRC_Init();
  MX_IWDG_Init();
  MX_I2C2_Init();
  MX_UART5_Init();
  MX_TIM4_Init();
  MX_TIM5_Init();
  MX_TIM8_Init();
  MX_CAN_Init();
  MX_TIM6_Init();
  /* USER CODE BEGIN 2 */
	IWDG->KR = 0xAAAA; // Writing 0xAAAA in the Key register prevents watchdog reset
	IWDG->KR = 0xCCCC; // Start the independent watchdog timer	
	
	HAL_GPIO_WritePin(W_R_4_GPIO_Port, W_R_4_Pin,GPIO_PIN_RESET);			//RS485 In Receive mode
	HAL_UART_Receive_IT(&huart2, &usart2_Rx.rxByte,1);								//Init usart2 interrupt
	HAL_UART_Receive_IT(&huart4, &usart4_Rx.rxByte,1);								//Init usart4 interrupt
	HAL_UART_Receive_IT (&huart5, &usart5_Rx.rxByte, 1);							//Init usart5 interrupt
	
	HAL_TIM_Base_Start_IT(&htim2);																		//Enable Timer interrupts - 10ms
	HAL_TIM_Base_Start_IT(&htim3);																		//Enable Timer interrupts - ENCODER Mode
	HAL_TIM_Base_Start_IT(&htim4);																		//Enable Timer interrupts - 100ms
	HAL_TIM_Base_Start_IT(&htim5);																		//Enable Timer interrupts - 1s
	HAL_TIM_Base_Start_IT(&htim6);																		//Enable Timer interrupts - 1ms
	HAL_TIM_Base_Start_IT(&htim8);																		//Enable Timer interrupts - 20ms
	HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);										//Start Encoder 
	
	//Calibrate the ADC
	HAL_ADCEx_Calibration_Start(&hadc1);	
	//Initialize DC motor parameters
	vMachineParameterConfig(&stMachine_Var);
	//State machine Initialize
	vStateMachineConfig(&stMControl_Var, &stMachine_Var);
	//Initialize adc data	
	vAdc_DataInit(&adc_Objt_var, &stMachine_Var);					
	//Start DAC
	HAL_DAC_Start(&hdac, DAC_CHANNEL_1);	
	HAL_DAC_Start(&hdac, DAC_CHANNEL_2);
	//Initialize DAC data
	vDAC_Init(&stDAC_Var);
	//Generate Excitation current reference	
	vDAC_Write(&stDAC_Var, DAC_IfRefChannel, (stMachine_Var.Va_rated/stMControl_Var.Vdc_Nom)*DAC_MAX_Vout);	
	//Torque current reference = 0
	vDAC_Write(&stDAC_Var, DAC_DutyCycle, 0);	
	//Timers initialize
	vTim_Start(&Tim_1ms[0],1);					//Create 1ms time tick
	vTim_Start(&Tim_10ms[0],1);					//Create 10ms time tick
	vTim_Start(&Tim_20ms[0],1);					//Create 20ms time tick
	vTim_Start(&Tim_100ms[0],1);				//Create 100ms time tick
	vTim_Start(&Tim_200ms[0],1);				//Create 100ms time tick
	vTim_Start(&Tim_1s[0],1);						//Create 1s time tick 

	tim1msTick = 0;
	tim10msTick = 0;
	tim20msTick = 0;
	tim100msTick = 0;
	tim200msTick = 0;
	tim1sTick = 0;
	//Low pass filter initialize
	vLPF_Init(&stLPF_Var[0], 100, 0.0005);		//LPF0 - For TorqueSP filter
	vLPF_Init(&stLPF_Var[1], 100, 0.0005);		//LPF1 - For SpeedSP filter
	vLPF_Init(&stLPF_Var[2], 100, 0.0005);		//LPF2 - For Ia filter
	vLPF_Init(&stLPF_Var[3], 100, 0.0005);		//LPF3 - For If filter
	vLPF_Init(&stLPF_Var[4], 10, 0.0005);			//LPF4 - For Vbus filter
	vLPF_Init(&stLPF_Var[5], 50, 0.0005);			//LPF5 - For dia/dt filter
	vLPF_Init(&stLPF_Var[6], 10, 0.0005);			//LPF6 - For EsSpeed filter	
	//GPIO Initialize
	vDIO_Init();
	//Serial ports initialize
	vUsart_DataInit(&usart2_Rx);
	vUsart_DataInit(&usart4_Rx);
	vUsart_DataInit(&usart5_Rx);
	//Config PI Speed Controller 10ms sampling time
	vPI_Init(&stPI_Speed, 1, 0.1, 0.01, adc_Objt_var.MaxSpeedSP, adc_Objt_var.MaxTorqueSP);
	//Config PI amature current controller with 0.5ms sampling time
	vPI_Init(&stPI_Ia, 1, 0.1, 0.0005, adc_Objt_var.MaxIaSP, (stMachine_Var.Va_rated/stMControl_Var.Vdc_Nom)*DAC_MAX_Vout);			
	//Config Quadrature Encoder
	vQEI_Config(&stQEI_Var);
	//Config Ramp function generator
	vRamp_Config(&stRamp_Ia, 10, 10, 0, stMachine_Var.Ia_rated, 0.0005);
	vRamp_Config(&stRamp_If, 10, 10, 0, stMachine_Var.If_rated, 0.01);
	vRamp_Config(&stRamp_Speed, 10, 10, 0, stMachine_Var.Speed_rated, 0.01);
	vRamp_Config(&stRamp_Vol, 10, 10, 0, stMachine_Var.Va_rated, 0.01);
	//Config Nextion HMI
	vNEXTION_Init(&NEXVar);	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		//Scan the software timer
		if(tim1msTick ==1)
		{
			tim1msTick = 0;
			vTim_Scan(Tim_1ms);
		}		
		if(tim10msTick ==1)
		{
			tim10msTick = 0;
			vTim_Scan(Tim_10ms);
		}
		
		if(tim20msTick ==1)
		{
			tim20msTick = 0;
			vTim_Scan(Tim_20ms);
		}
		
		if(tim100msTick == 1)
		{
			tim100msTick = 0;
			vTim_Scan(Tim_100ms);			
		}
		
		if(tim200msTick == 1)
		{
			tim200msTick = 0;
			vTim_Scan(Tim_200ms);			
		}
		if(tim1sTick == 1)
		{
			tim1sTick = 0;
			vTim_Scan(Tim_1s);			
		}
		
		//Run Regular Timer Task
		if(Tim_1ms[0].En==1 && Tim_1ms[0].Output==1)			//1ms event
		{
			if(FirstCycle == 1)
			{
				//vSafetyChainHandle(&adc_Objt_var, &stMControl_Var, &stQEI_Var);
			}			
			//Reset timer
			Tim_1ms[0].Output = 0;
		}
		if(Tim_10ms[0].En==1 && Tim_10ms[0].Output==1)		//10ms event
		{
			if(FirstCycle==1)
			{				
				//GPIO Task
				vDI_Scan();
				vDIO_Task(&stMControl_Var);
			}
			//Reset timer
			Tim_10ms[0].Output = 0;
		}
		if(Tim_20ms[0].En==1 && Tim_20ms[0].Output==1)		//20ms event
		{
			
			//Reset timer
			Tim_20ms[0].Output = 0;
		}
		if(Tim_100ms[0].En==1 && Tim_100ms[0].Output==1)	//100ms event
		{	
			if(Tim_100ms[0].OvfCnt == 5)
			{
				tim200msTick = 1;
				Tim_100ms[0].OvfCnt = 0;
			}
			
			if(FirstCycle==1)
			{
				/*Communication task*/
				//Send data to ESP32
				//vUsart_TxHandle(&adc_Objt_var,&stMControl_Var, &stRamp_Ia, &stRamp_Speed, &usart2_Tx, 2, 0x31);
				
			}
			//Reset timer
			Tim_100ms[0].Output = 0;
		}	
		
		if(Tim_200ms[0].En==1 && Tim_200ms[0].Output==1)	//200ms event
		{		
			if(FirstCycle==1)
			{
				/*Send data to HMI*/
				vNEXTION_DisplayHandle(&adc_Objt_var, &stMControl_Var, &stPI_Speed, &stPI_Ia,
				&stRamp_Ia, &stRamp_Speed, &stQEI_Var, &stMachine_Var, &NEXVar);		
				
				//vUsart4_RxHandle(&usart4_Rx,&usart4_Tx,&stMControl_Var,&adc_Objt_var);
				//HAL_GPIO_TogglePin(DO3_GPIO_Port, DO3_Pin);
			}				
			//Reset timer
			Tim_200ms[0].Output = 0;
		}
		
		if(Tim_1s[0].En==1 && Tim_1s[0].Output==1)				//1s event
		{			
			if(FirstCycle == 0)
			{
				if(++Cnt>2)
				{
					//First cycle task - Once time only
					//Close the bypass contactor
					HAL_GPIO_WritePin(Bypass_GPIO_Port, Bypass_Pin, GPIO_PIN_SET);	
					HAL_GPIO_WritePin(DO0_GPIO_Port, DO0_Pin, GPIO_PIN_SET);
					stMControl_Var.chState = DOL;
					
					FirstCycle = 1;
					Cnt = 0;
				}
			}						
			//Reset timer
			Tim_1s[0].Output = 0;
		}
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		//ESP32 ports scan
		if(usart2_Rx.RxFlag == 1)
		{
			
			usart2_Rx.RxFlag = 0;
			usart2_Rx.STATE = USART_WAIT;
		}
		//RS485 ports scan
		if(usart4_Rx.RxFlag == 1)
		{
			/*Send data via RS485*/
			vUsart4_RxHandle(&usart4_Rx,&usart4_Tx,&stMControl_Var,&adc_Objt_var);
			usart4_Rx.RxFlag = 0;
			usart4_Rx.STATE = USART_WAIT;
		}
		//HMI ports scan
		if(usart5_Rx.RxFlag == 1)
		{
			vUsart5_rxHandle(&usart5_Rx, &NEXVar, &stMControl_Var);
			usart5_Rx.RxFlag = 0;
			usart5_Rx.STATE = USART_WAIT;
		}
		if(usart5_Rx.NEXTION_Flag == 1)
		{
			vNEXTION_RecvRetNumHandle(usart5_Rx.NEXTION_RetCode);
			usart5_Rx.NEXTION_Flag = 0;
		}		
		
		//Time out handle
		if(Tim_1s[1].En==1 && Tim_1s[1].Output==1)				//HMI port timeout
		{
			//Clear NEXTIO_Buff
			for(i=0;i<4;i++)
			{
				usart5_Rx.NEXTION_Buff[i] = 0;
			}
			usart5_Rx.NEXTION_Pointer = 0;
			usart5_Rx.NEXTION_Flag = 0;
			usart5_Rx.NEXTION_RetCode = 0;
			vTim_Stop(&Tim_1s[1]);
		}
		
		if(Tim_1s[2].En==1 && Tim_1s[2].Output==1)				//ESP32 Port time out
		{
			vUSART_RxTimeOutHandle(&usart2_Rx);
			vTim_Stop(&Tim_1s[2]);
		}
		
		if(Tim_1s[3].En==1 && Tim_1s[3].Output==1)				//RS485 Port time out
		{
			vUSART_RxTimeOutHandle(&usart4_Rx);
			vTim_Stop(&Tim_1s[3]);
		}
		if(Tim_1s[4].En==1 && Tim_1s[4].Output==1)				//HMI Time out (Button, Gauge ... )
		{
			vUSART_RxTimeOutHandle(&usart5_Rx);
			vTim_Stop(&Tim_1s[4]);
		}
		if(Tim_1s[5].En==1 && Tim_1s[5].Output==1)				//Amature current is too high
		{
			stMControl_Var.mState = mSTOP;
			stMControl_Var.Ia_State = FAULT_H;							//Ia Over Current
			vTim_Stop(&Tim_1s[5]);
			
		}
		if(Tim_1s[6].En==1 && Tim_1s[6].Output==1)				//Excitation current is too High
		{
			stMControl_Var.mState = mFAULT;
			stMControl_Var.If_State = FAULT_H;								
			vTim_Stop(&Tim_1s[6]);
		}
		if(Tim_1s[7].En==1 && Tim_1s[7].Output==1)				//Excitation current is too Low
		{
			stMControl_Var.mState = mFAULT;
			stMControl_Var.If_State = FAULT_L;								
			vTim_Stop(&Tim_1s[7]);
		}
		if(Tim_1s[8].En==1 && Tim_1s[8].Output==1)				//Low Vdc
		{
			stMControl_Var.mState = mFAULT;
			stMControl_Var.Vdc_State = FAULT_L;								
			vTim_Stop(&Tim_1s[8]);
		}
		if(Tim_1s[9].En==1 && Tim_1s[9].Output==1)				//High Vdc
		{
			stMControl_Var.mState = mFAULT;
			stMControl_Var.Vdc_State = FAULT_H;								
			vTim_Stop(&Tim_1s[9]);
		}
		if(Tim_1s[10].En==1 && Tim_1s[10].Output==1)			//High Speed
		{
			stMControl_Var.mState = mFAULT;
			stMControl_Var.Speed_State = FAULT_H;								
			vTim_Stop(&Tim_1s[10]);
		}
		IWDG->KR = 0xAAAA; // Writing 0xAAAA in the Key register prevents watchdog reset
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL8;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV4;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 5;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_7;
  sConfig.Rank = ADC_REGULAR_RANK_4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_14;
  sConfig.Rank = ADC_REGULAR_RANK_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief CAN Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN_Init(void)
{

  /* USER CODE BEGIN CAN_Init 0 */

  /* USER CODE END CAN_Init 0 */

  /* USER CODE BEGIN CAN_Init 1 */

  /* USER CODE END CAN_Init 1 */
  hcan.Instance = CAN1;
  hcan.Init.Prescaler = 16;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_3TQ;
  hcan.Init.TimeSeg2 = CAN_BS2_5TQ;
  hcan.Init.TimeTriggeredMode = DISABLE;
  hcan.Init.AutoBusOff = ENABLE;
  hcan.Init.AutoWakeUp = ENABLE;
  hcan.Init.AutoRetransmission = DISABLE;
  hcan.Init.ReceiveFifoLocked = DISABLE;
  hcan.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN_Init 2 */

  /* USER CODE END CAN_Init 2 */

}

/**
  * @brief CRC Initialization Function
  * @param None
  * @retval None
  */
static void MX_CRC_Init(void)
{

  /* USER CODE BEGIN CRC_Init 0 */

  /* USER CODE END CRC_Init 0 */

  /* USER CODE BEGIN CRC_Init 1 */

  /* USER CODE END CRC_Init 1 */
  hcrc.Instance = CRC;
  if (HAL_CRC_Init(&hcrc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CRC_Init 2 */

  /* USER CODE END CRC_Init 2 */

}

/**
  * @brief DAC Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC_Init(void)
{

  /* USER CODE BEGIN DAC_Init 0 */

  /* USER CODE END DAC_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC_Init 1 */

  /* USER CODE END DAC_Init 1 */
  /** DAC Initialization
  */
  hdac.Instance = DAC;
  if (HAL_DAC_Init(&hdac) != HAL_OK)
  {
    Error_Handler();
  }
  /** DAC channel OUT1 config
  */
  sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  if (HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /** DAC channel OUT2 config
  */
  if (HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC_Init 2 */

  /* USER CODE END DAC_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 100000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_ENABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief IWDG Initialization Function
  * @param None
  * @retval None
  */
static void MX_IWDG_Init(void)
{

  /* USER CODE BEGIN IWDG_Init 0 */

  /* USER CODE END IWDG_Init 0 */

  /* USER CODE BEGIN IWDG_Init 1 */

  /* USER CODE END IWDG_Init 1 */
  hiwdg.Instance = IWDG;
  hiwdg.Init.Prescaler = IWDG_PRESCALER_256;
  hiwdg.Init.Reload = 312;
  if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN IWDG_Init 2 */

  /* USER CODE END IWDG_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 9;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 63999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 1439;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 10;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 10;
  if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 99;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 63999;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 999;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 63999;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 0;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 31999;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief TIM8 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM8_Init(void)
{

  /* USER CODE BEGIN TIM8_Init 0 */

  /* USER CODE END TIM8_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM8_Init 1 */

  /* USER CODE END TIM8_Init 1 */
  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 19;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 63999;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim8, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM8_Init 2 */

  /* USER CODE END TIM8_Init 2 */

}

/**
  * @brief UART4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART4_Init(void)
{

  /* USER CODE BEGIN UART4_Init 0 */

  /* USER CODE END UART4_Init 0 */

  /* USER CODE BEGIN UART4_Init 1 */

  /* USER CODE END UART4_Init 1 */
  huart4.Instance = UART4;
  huart4.Init.BaudRate = 19200;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART4_Init 2 */

  /* USER CODE END UART4_Init 2 */

}

/**
  * @brief UART5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART5_Init(void)
{

  /* USER CODE BEGIN UART5_Init 0 */

  /* USER CODE END UART5_Init 0 */

  /* USER CODE BEGIN UART5_Init 1 */

  /* USER CODE END UART5_Init 1 */
  huart5.Instance = UART5;
  huart5.Init.BaudRate = 9600;
  huart5.Init.WordLength = UART_WORDLENGTH_8B;
  huart5.Init.StopBits = UART_STOPBITS_1;
  huart5.Init.Parity = UART_PARITY_NONE;
  huart5.Init.Mode = UART_MODE_TX_RX;
  huart5.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart5.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart5) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART5_Init 2 */

  /* USER CODE END UART5_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, DO4_Pin|DO3_Pin|DO2_Pin|DO1_Pin
                          |DO0_Pin|Bypass_Pin|D_Braking_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, Reset_Pin|Enable_Pin|LDAC_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(W_R_4_GPIO_Port, W_R_4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : DO4_Pin DO3_Pin DO2_Pin DO1_Pin
                           DO0_Pin Bypass_Pin D_Braking_Pin */
  GPIO_InitStruct.Pin = DO4_Pin|DO3_Pin|DO2_Pin|DO1_Pin
                          |DO0_Pin|Bypass_Pin|D_Braking_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : Reset_Pin Enable_Pin LDAC_Pin */
  GPIO_InitStruct.Pin = Reset_Pin|Enable_Pin|LDAC_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : EXTI2_Fault_Pin */
  GPIO_InitStruct.Pin = EXTI2_Fault_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(EXTI2_Fault_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : QEI_Z_Pin */
  GPIO_InitStruct.Pin = QEI_Z_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(QEI_Z_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : W_R_4_Pin */
  GPIO_InitStruct.Pin = W_R_4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(W_R_4_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : DI0_Pin DI1_Pin DI2_Pin DI3_Pin
                           DI4_Pin DI5_Pin DI6_Pin */
  GPIO_InitStruct.Pin = DI0_Pin|DI1_Pin|DI2_Pin|DI3_Pin
                          |DI4_Pin|DI5_Pin|DI6_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);

}

/* USER CODE BEGIN 4 */
#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */

/**
  * @brief  Retargets the C library printf function to the USART5.
  * @param  None
  * @retval None
  */
PUTCHAR_PROTOTYPE
{
  HAL_UART_Transmit(&huart5, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
  return ch;
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
