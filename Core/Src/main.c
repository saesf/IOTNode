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
#include <stdlib.h>
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#include "string.h"
#include "wifi.h"
#include "config.h"
#include "sensors.h"
#include "lsm6dsl_reg.h"
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
DFSDM_Filter_HandleTypeDef hdfsdm1_filter0;
DFSDM_Filter_HandleTypeDef hdfsdm1_filter1;
DFSDM_Channel_HandleTypeDef hdfsdm1_channel1;
DFSDM_Channel_HandleTypeDef hdfsdm1_channel2;
DMA_HandleTypeDef hdma_dfsdm1_flt0;
DMA_HandleTypeDef hdma_dfsdm1_flt1;

I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;
DMA_HandleTypeDef hdma_i2c1_rx;
DMA_HandleTypeDef hdma_i2c1_tx;

SPI_HandleTypeDef hspi3;
DMA_HandleTypeDef hdma_spi3_tx;
DMA_HandleTypeDef hdma_spi3_rx;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart1_tx;

DMA_HandleTypeDef hdma_memtomem_dma2_channel3;
/* USER CODE BEGIN PV */
uint8_t *payloadBuffers[BUFFER_Q_SIZE];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_SPI3_Init(void);
static void MX_I2C2_Init(void);
static void MX_DFSDM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */
#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif

PUTCHAR_PROTOTYPE
{
  HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
  return ch;
}
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint8_t connected[]={'K','O','\n','\r'};
uint8_t already[]={'l','A','e','r','d'};
int wifiState=0;
const char dummyCommand[DUMMY_LENGTH]={
		 0x0a,0x0a,0x0a,0x0a,0x0a,0x0a,0x0a,0x0a,0x0a,0x0a
		,0x0a,0x0a,0x0a,0x0a,0x0a,0x0a,0x0a,0x0a,0x0a,0x0a
		,0x0a,0x0a,0x0a,0x0a,0x0a,0x0a,0x0a,0x0a,0x0a,0x0a
		,0x0a,0x0a,0x0a,0x0a,0x0a,0x0a,0x0a,0x0a,0x0a,0x0a
		,0x0a,0x0a,0x0a,0x0a,0x0a,0x0a,0x0a,0x0a,0x0a,0x0a
		,0x0a,0x0a,0x0a,0x0a,0x0a,0x0a,0x0a,0x0a,0x0a,0x0a
		,0x0a,0x0a,0x0a,0x0a,0x0a,0x0a,0x0a,0x0a,0x0a,0x0a
		,0x0a,0x0a,0x0a,0x0a,0x0a,0x0a,0x0a,0x0a,0x0a,0x0a
		,0x0a,0x0a,0x0a,0x0a,0x0a,0x0a,0x0a,0x0a,0x0a,0x0a
		,0x0a,0x0a,0x0a,0x0a,0x0a,0x0a,0x0a,0x0a,0x0a,0x0a
};

int CountDigit(int num)
{
	if(num<10) return 1;
	if(num<100) return 2;
	if(num<1000) return 3;
	if(num<10000) return 4;
	return 5;
}
uint8_t UARTTXCompleteFlag=0;

void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
	if (hi2c->Instance==hi2c1.Instance)
	lsdm6dslDmaCpltFlg=2;
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance==huart1.Instance)
		UARTTXCompleteFlag=1;

  /* NOTE : This function should not be modified, when the callback is needed,
            the HAL_UART_TxCpltCallback can be implemented in the user file.
   */
}

void HAL_UART_TxHalfCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance==huart1.Instance)
		UARTTXCompleteFlag=1;

  /* NOTE : This function should not be modified, when the callback is needed,
            the HAL_UART_TxCpltCallback can be implemented in the user file.
   */
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if(GPIO_Pin == GPIO_PIN_1)
  {
	  if(HAL_GPIO_ReadPin(WIFI_INT_GPIO_Port,WIFI_INT_Pin)==1)
	  {
		  wifiReady=1;
	  }
	  else if(HAL_GPIO_ReadPin(WIFI_INT_GPIO_Port,WIFI_INT_Pin)==0)
	  {
		  DMAReceiveCompleted=1;
	  }
  }
  else if(GPIO_Pin == GPIO_PIN_3)
  {
	  lsdm6dslDataReadyFlg=1;
  }
}

/*
 * microphone dma callback
 * */
uint32_t micDma0HlfCmplt=0;
uint32_t micDma1HlfCmplt=0;
uint32_t micDma0FullCmplt=0;
uint32_t micDma1FullCmplt=0;
uint32_t lastWifiEventTimeStamp=0;
uint8_t lsdm6dslDataReadyFlg=0;

char *sensorBuffer[BUFFER_Q_SIZE];
uint8_t memTomemTransferCompleteFlag=1;
char *micdata0;
char *micdata1;
void HAL_DFSDM_FilterRegConvHalfCpltCallback(DFSDM_Filter_HandleTypeDef *hdfsdm_filter)
{
	/* Prevent unused argument(s) compilation warning */
//	UNUSED(hdfsdm_filter);
	if(hdfsdm_filter==&hdfsdm1_filter0)
	{
		micDma0HlfCmplt=1;
//		enqueue(&sensorBufferQueue, 0);
	}
	else if(hdfsdm_filter==&hdfsdm1_filter1)
	{
		micDma1HlfCmplt=1;
//		enqueue(&sensorBufferQueue, 2);
	}
}

void HAL_DFSDM_FilterRegConvCpltCallback(DFSDM_Filter_HandleTypeDef *hdfsdm_filter)
{
	if(hdfsdm_filter==&hdfsdm1_filter0)
	{
		micDma0FullCmplt=1;
//		enqueue(&sensorBufferQueue, 1);
	}
	else if(hdfsdm_filter==&hdfsdm1_filter1)
	{
		micDma1FullCmplt=1;
//		enqueue(&sensorBufferQueue, 3);
	}
}

int SendDummy()
{
	uint8_t index=0;
	HAL_GPIO_WritePin(WIFI_CS_GPIO_Port,WIFI_CS_Pin,GPIO_PIN_RESET);
	for(index=0;index<128;index+=2)
	{
	  HAL_SPI_TransmitReceive(&hspi3,(uint8_t *)dummyCommand,(uint8_t *)payloadBuffers[2]+index,2/2,1);
	  if(*(payloadBuffers[2]+index)=='>' && *(payloadBuffers[2]+index+1)==' ')
		  break;
	}
	HAL_GPIO_WritePin(WIFI_CS_GPIO_Port,WIFI_CS_Pin,GPIO_PIN_SET);
	while(wifiReady==0);
	if(*(payloadBuffers[2]+index)=='>' && *(payloadBuffers[2]+index+1)==' '){
		return 0;
	}
	else
		return 1;
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

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
  MX_SPI3_Init();
  MX_I2C2_Init();
  MX_DFSDM1_Init();
  MX_TIM2_Init();
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
    unsigned char emptyBufferIndex=255, fullBufferIndex=255, sensorBufferIndex=255, lsm6dslBufferIndex=255;
	micdata0=malloc((PAYLOAD_LEN-4)*2);
	micdata1=malloc((PAYLOAD_LEN-4)*2);

	int len = CountDigit(PAYLOAD_LEN)+4;
	char header[len];
	sprintf(header,"S3=%d\r", PAYLOAD_LEN);
	payload=malloc(PAYLOAD_LEN+len);
	payloadLen=PAYLOAD_LEN+len;
	memcpy(payload, header, len);
	Qinit(&emptyBufferQueue);
	Qinit(&fullBufferQueue);
	for(int i=0;i<BUFFER_Q_SIZE;i++)
	{
		payloadBuffers[i]=malloc(PAYLOAD_LEN+len);
		memset(payloadBuffers[i],'a',PAYLOAD_LEN+len);
		memcpy(payloadBuffers[i], header, len);
		enqueue(&emptyBufferQueue, i);
	}

	Qinit(&sensorBufferQueue);
	sensorBuffer[0]=&micdata0[0];
	sensorBuffer[1]=&micdata0[PAYLOAD_LEN-4];
	sensorBuffer[2]=&micdata1[0];
	sensorBuffer[3]=&micdata1[PAYLOAD_LEN-4];
	sensorBuffer[4]=&payload[0];

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  //wait for the module that is ready for communication
  WifiInit();


  //init mems
  HAL_TIM_Base_Start(&htim2);

  MagnetoMeterInitialization(LIS3MDL_LP_1kHz, LIS3MDL_16_GAUSS, LIS3MDL_CONTINUOUS_MODE, 1);
  while(AccelGyroMeterInitialization()!=0)
	  HAL_Delay(500);

  uint32_t start_ts=0;

  	  /* Start DFSDM conversions */
	if( HAL_OK != HAL_DFSDM_FilterRegularStart_DMA(&hdfsdm1_filter0, (int32_t *)micdata0, (PAYLOAD_LEN-4)*2/4) )
	{
	  Error_Handler();
	}
	if( HAL_OK != HAL_DFSDM_FilterRegularStart_DMA(&hdfsdm1_filter1, (int32_t *)micdata1, (PAYLOAD_LEN-4)*2/4) )
	{
	  Error_Handler();
	}



  while (1)
  {
	if(micDma0HlfCmplt==1)
		{
			micDma0HlfCmplt=0;
			enqueue(&sensorBufferQueue, 0);
		}
		if(micDma1HlfCmplt==1)
		{
			micDma1HlfCmplt=0;
			enqueue(&sensorBufferQueue, 2);
		}
		if(micDma0FullCmplt==1)
		{
			micDma0FullCmplt=0;
			enqueue(&sensorBufferQueue, 1);
		}
		if(micDma1FullCmplt==1)
		{
			micDma1FullCmplt=0;
			enqueue(&sensorBufferQueue, 3);
		}

		if(lsdm6dslDataReadyFlg==1 && lsdm6dslDmaCpltFlg==1)
		{
			start_ts=htim2.Instance->CNT;
			if(lsm6dslBufferIndex!=255){
				enqueue(&fullBufferQueue, lsm6dslBufferIndex);
				lsm6dslBufferIndex=255;
			}
			if(dequeue(&emptyBufferQueue, &lsm6dslBufferIndex)==1)
			{
				lsdm6dslDataReadyFlg=0;
				lsdm6dslDmaCpltFlg=0;
				HAL_I2C_Mem_Read_DMA(&SENSOR_BUS, LSM6DSL_I2C_ADD_R, LSM6DSL_FIFO_DATA_OUT_L, I2C_MEMADD_SIZE_8BIT, (uint8_t *)(payloadBuffers[lsm6dslBufferIndex]+len+4), LSDM6DSL_FIFO_THRESHOLD*2);

				*(payloadBuffers[lsm6dslBufferIndex]+8) = LSM6DSL_ID;
				*(payloadBuffers[lsm6dslBufferIndex]+9) = LSM6DSL_ID;
				*(payloadBuffers[lsm6dslBufferIndex]+10) = LSM6DSL_ID;
				*(payloadBuffers[lsm6dslBufferIndex]+11) = LSM6DSL_ID;
			}
			else
			{
				printf("error: there is no empty buffer\n\r");
			}
		}
		if(lsdm6dslDmaCpltFlg==2)
		{
			lsdm6dslDmaCpltFlg=1;
//			printf("%d: %d\t%d\t%d\t%d\t%d\t%d\n\r",(htim2.Instance->CNT-start_ts)*10, (int16_t)((int16_t)(*(payloadBuffers[lsm6dslBufferIndex]+len+4+1)<<8)|*(payloadBuffers[lsm6dslBufferIndex]+len+4+0))
//																				 , (int16_t)((int16_t)(*(payloadBuffers[lsm6dslBufferIndex]+len+4+3)<<8)|*(payloadBuffers[lsm6dslBufferIndex]+len+4+2))
//																				 , (int16_t)((int16_t)(*(payloadBuffers[lsm6dslBufferIndex]+len+4+5)<<8)|*(payloadBuffers[lsm6dslBufferIndex]+len+4+4))
//																				 , (int16_t)((int16_t)(*(payloadBuffers[lsm6dslBufferIndex]+len+4+7)<<8)|*(payloadBuffers[lsm6dslBufferIndex]+len+4+6))
//																				 , (int16_t)((int16_t)(*(payloadBuffers[lsm6dslBufferIndex]+len+4+9)<<8)|*(payloadBuffers[lsm6dslBufferIndex]+len+4+8))
//																				 , (int16_t)((int16_t)(*(payloadBuffers[lsm6dslBufferIndex]+len+4+11)<<8)|*(payloadBuffers[lsm6dslBufferIndex]+len+4+10)));
			if(lsm6dslBufferIndex!=255){
				enqueue(&fullBufferQueue, lsm6dslBufferIndex);
				lsm6dslBufferIndex=255;
			}
		}

	  if(memTomemTransferCompleteFlag==1)
	  {
		  if(emptyBufferIndex!=255){
			  enqueue(&fullBufferQueue, emptyBufferIndex);
			  emptyBufferIndex=255;
		  }
		  if(dequeue(&sensorBufferQueue, &sensorBufferIndex)==1)
		  {
			  // if(index==4) //you should compensate for the transfer size
			  memTomemTransferCompleteFlag=0;
			  if(dequeue(&emptyBufferQueue, &emptyBufferIndex)==1)
			  {
				  if(HAL_OK != HAL_DMA_Start_IT(&hdma_memtomem_dma2_channel3, (uint32_t)sensorBuffer[sensorBufferIndex],
				  			  					  (uint32_t)(payloadBuffers[emptyBufferIndex]+len+4), PAYLOAD_LEN-4))
					{
					  printf("error on micdata0[0] DMA transfer\n\r");
					  enqueue(&emptyBufferQueue, emptyBufferIndex);
					}
				  *(payloadBuffers[emptyBufferIndex]+8) = sensorBufferIndex;
				  *(payloadBuffers[emptyBufferIndex]+9) = sensorBufferIndex;
				  *(payloadBuffers[emptyBufferIndex]+10) = sensorBufferIndex;
				  *(payloadBuffers[emptyBufferIndex]+11) = sensorBufferIndex;
			  }
			  else{
				  printf("error: there is not empty buffer\n\r");
				  memTomemTransferCompleteFlag=1;
			  }
		  }
	  }



		  if((htim2.Instance->CNT-lastWifiEventTimeStamp)>500 && wifiState==1 && wifiReady==1)
		  {
			  if(dequeue(&fullBufferQueue, &fullBufferIndex)==1)
			  {
				lastWifiEventTimeStamp=htim2.Instance->CNT;
				wifiReady=0;
				SendPayload((char *)payloadBuffers[fullBufferIndex], PAYLOAD_LEN+len);
				wifiState=2;
			  }
		  }
		  else if(wifiState==2 && DMATransferCompleted==1)
		  {
			DMATransferCompleted=0;
			HAL_GPIO_WritePin(WIFI_CS_GPIO_Port,WIFI_CS_Pin,GPIO_PIN_SET);
			wifiState=3;
			if(fullBufferIndex!=255){
				enqueue(&emptyBufferQueue, fullBufferIndex);
				fullBufferIndex=255;
			}
		  }
		  else if(wifiState==3 && wifiReady==1)
		  {
			wifiReady=0;
			SendPayload((char *)dummyCommand, DUMMY_LENGTH);
			wifiState=4;
		  }
		  else if(wifiState==4 && DMATransferCompleted==1)
		  {
			DMATransferCompleted=0;
			HAL_GPIO_WritePin(WIFI_CS_GPIO_Port,WIFI_CS_Pin,GPIO_PIN_SET);
			wifiState=1;
		  }
		  if(htim2.Instance->CNT<lastWifiEventTimeStamp)
			  lastWifiEventTimeStamp=htim2.Instance->CNT;
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
//  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI | RCC_OSCILLATORTYPE_LSI | RCC_OSCILLATORTYPE_HSE | RCC_OSCILLATORTYPE_LSE | RCC_OSCILLATORTYPE_MSI;
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI | RCC_OSCILLATORTYPE_LSI | RCC_OSCILLATORTYPE_HSE | RCC_OSCILLATORTYPE_LSE | RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 40;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_I2C1
                              |RCC_PERIPHCLK_I2C2|RCC_PERIPHCLK_DFSDM1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
  PeriphClkInit.I2c2ClockSelection = RCC_I2C2CLKSOURCE_PCLK1;
  PeriphClkInit.Dfsdm1ClockSelection = RCC_DFSDM1CLKSOURCE_PCLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief DFSDM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_DFSDM1_Init(void)
{

  /* USER CODE BEGIN DFSDM1_Init 0 */
  /* USER CODE END DFSDM1_Init 0 */

  /* USER CODE BEGIN DFSDM1_Init 1 */
  /* USER CODE END DFSDM1_Init 1 */
  hdfsdm1_filter0.Instance = DFSDM1_Filter0;
  hdfsdm1_filter0.Init.RegularParam.Trigger = DFSDM_FILTER_SW_TRIGGER;
  hdfsdm1_filter0.Init.RegularParam.FastMode = ENABLE;
  hdfsdm1_filter0.Init.RegularParam.DmaMode = ENABLE;
  hdfsdm1_filter0.Init.FilterParam.SincOrder = DFSDM_FILTER_SINC4_ORDER;
  hdfsdm1_filter0.Init.FilterParam.Oversampling = 128;
  hdfsdm1_filter0.Init.FilterParam.IntOversampling = 1;
  if (HAL_DFSDM_FilterInit(&hdfsdm1_filter0) != HAL_OK)
  {
    Error_Handler();
  }
  hdfsdm1_filter1.Instance = DFSDM1_Filter1;
  hdfsdm1_filter1.Init.RegularParam.Trigger = DFSDM_FILTER_SW_TRIGGER;
  hdfsdm1_filter1.Init.RegularParam.FastMode = ENABLE;
  hdfsdm1_filter1.Init.RegularParam.DmaMode = ENABLE;
  hdfsdm1_filter1.Init.FilterParam.SincOrder = DFSDM_FILTER_SINC4_ORDER;
  hdfsdm1_filter1.Init.FilterParam.Oversampling = 128;
  hdfsdm1_filter1.Init.FilterParam.IntOversampling = 1;
  if (HAL_DFSDM_FilterInit(&hdfsdm1_filter1) != HAL_OK)
  {
    Error_Handler();
  }
  hdfsdm1_channel1.Instance = DFSDM1_Channel1;
  hdfsdm1_channel1.Init.OutputClock.Activation = ENABLE;
  hdfsdm1_channel1.Init.OutputClock.Selection = DFSDM_CHANNEL_OUTPUT_CLOCK_SYSTEM;
  hdfsdm1_channel1.Init.OutputClock.Divider = 39;
  hdfsdm1_channel1.Init.Input.Multiplexer = DFSDM_CHANNEL_EXTERNAL_INPUTS;
  hdfsdm1_channel1.Init.Input.DataPacking = DFSDM_CHANNEL_STANDARD_MODE;
  hdfsdm1_channel1.Init.Input.Pins = DFSDM_CHANNEL_FOLLOWING_CHANNEL_PINS;
  hdfsdm1_channel1.Init.SerialInterface.Type = DFSDM_CHANNEL_SPI_RISING;
  hdfsdm1_channel1.Init.SerialInterface.SpiClock = DFSDM_CHANNEL_SPI_CLOCK_INTERNAL;
  hdfsdm1_channel1.Init.Awd.FilterOrder = DFSDM_CHANNEL_FASTSINC_ORDER;
  hdfsdm1_channel1.Init.Awd.Oversampling = 1;
  hdfsdm1_channel1.Init.Offset = 0;
  hdfsdm1_channel1.Init.RightBitShift = 0x02;
  if (HAL_DFSDM_ChannelInit(&hdfsdm1_channel1) != HAL_OK)
  {
    Error_Handler();
  }
  hdfsdm1_channel2.Instance = DFSDM1_Channel2;
  hdfsdm1_channel2.Init.OutputClock.Activation = ENABLE;
  hdfsdm1_channel2.Init.OutputClock.Selection = DFSDM_CHANNEL_OUTPUT_CLOCK_SYSTEM;
  hdfsdm1_channel2.Init.OutputClock.Divider = 39;
  hdfsdm1_channel2.Init.Input.Multiplexer = DFSDM_CHANNEL_EXTERNAL_INPUTS;
  hdfsdm1_channel2.Init.Input.DataPacking = DFSDM_CHANNEL_STANDARD_MODE;
  hdfsdm1_channel2.Init.Input.Pins = DFSDM_CHANNEL_SAME_CHANNEL_PINS;
  hdfsdm1_channel2.Init.SerialInterface.Type = DFSDM_CHANNEL_SPI_FALLING;
  hdfsdm1_channel2.Init.SerialInterface.SpiClock = DFSDM_CHANNEL_SPI_CLOCK_INTERNAL;
  hdfsdm1_channel2.Init.Awd.FilterOrder = DFSDM_CHANNEL_FASTSINC_ORDER;
  hdfsdm1_channel2.Init.Awd.Oversampling = 1;
  hdfsdm1_channel2.Init.Offset = 0;
  hdfsdm1_channel2.Init.RightBitShift = 0x02;
  if (HAL_DFSDM_ChannelInit(&hdfsdm1_channel2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_DFSDM_FilterConfigRegChannel(&hdfsdm1_filter0, DFSDM_CHANNEL_1, DFSDM_CONTINUOUS_CONV_ON) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_DFSDM_FilterConfigRegChannel(&hdfsdm1_filter1, DFSDM_CHANNEL_2, DFSDM_CONTINUOUS_CONV_ON) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DFSDM1_Init 2 */

  /* USER CODE END DFSDM1_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x00702991;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

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
  hi2c2.Init.Timing = 0x00702991;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief SPI3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI3_Init(void)
{

  /* USER CODE BEGIN SPI3_Init 0 */

  /* USER CODE END SPI3_Init 0 */

  /* USER CODE BEGIN SPI3_Init 1 */

  /* USER CODE END SPI3_Init 1 */
  /* SPI3 parameter configuration*/
  hspi3.Instance = SPI3;
  hspi3.Init.Mode = SPI_MODE_MASTER;
  hspi3.Init.Direction = SPI_DIRECTION_2LINES;
  hspi3.Init.DataSize = SPI_DATASIZE_16BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi3.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi3.Init.NSS = SPI_NSS_SOFT;
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 7;
  hspi3.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi3.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI3_Init 2 */

  /* USER CODE END SPI3_Init 2 */

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
  htim2.Init.Prescaler = 800;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
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
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 576000;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * Enable DMA controller clock
  * Configure DMA for memory to memory transfers
  *   hdma_memtomem_dma2_channel3
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* Configure DMA request hdma_memtomem_dma2_channel3 on DMA2_Channel3 */
  hdma_memtomem_dma2_channel3.Instance = DMA2_Channel3;
  hdma_memtomem_dma2_channel3.Init.Request = DMA_REQUEST_0;
  hdma_memtomem_dma2_channel3.Init.Direction = DMA_MEMORY_TO_MEMORY;
  hdma_memtomem_dma2_channel3.Init.PeriphInc = DMA_PINC_ENABLE;
  hdma_memtomem_dma2_channel3.Init.MemInc = DMA_MINC_ENABLE;
  hdma_memtomem_dma2_channel3.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
  hdma_memtomem_dma2_channel3.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
  hdma_memtomem_dma2_channel3.Init.Mode = DMA_NORMAL;
  hdma_memtomem_dma2_channel3.Init.Priority = DMA_PRIORITY_LOW;
  if (HAL_DMA_Init(&hdma_memtomem_dma2_channel3) != HAL_OK)
  {
    Error_Handler( );
  }

  /* DMA interrupt init */
  /* DMA1_Channel4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel4_IRQn);
  /* DMA1_Channel5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);
  /* DMA1_Channel6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel6_IRQn);
  /* DMA1_Channel7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel7_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel7_IRQn);
  /* DMA2_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Channel1_IRQn);
  /* DMA2_Channel2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Channel2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Channel2_IRQn);
  /* DMA2_Channel3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Channel3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Channel3_IRQn);
  /* DMA2_Channel6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Channel6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Channel6_IRQn);
  /* DMA2_Channel7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Channel7_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Channel7_IRQn);

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, WIFI_RESET_Pin|WIFI_CS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(WIFI_BOOT_GPIO_Port, WIFI_BOOT_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, TOF_RESET_Pin|LED_BLUE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : XNUCLEO_INT1_Pin */
  GPIO_InitStruct.Pin = XNUCLEO_INT1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(XNUCLEO_INT1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : WIFI_RESET_Pin */
  GPIO_InitStruct.Pin = WIFI_RESET_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(WIFI_RESET_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PE12 PE13 PE14 PE15 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF10_QUADSPI;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : WIFI_BOOT_Pin */
  GPIO_InitStruct.Pin = WIFI_BOOT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(WIFI_BOOT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LSDM6DSL_INT1_EXTI11_Pin */
  GPIO_InitStruct.Pin = LSDM6DSL_INT1_EXTI11_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(LSDM6DSL_INT1_EXTI11_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : TOF_RESET_Pin LED_BLUE_Pin */
  GPIO_InitStruct.Pin = TOF_RESET_Pin|LED_BLUE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : LIS3MDL_DRDY_EXTI8_Pin */
  GPIO_InitStruct.Pin = LIS3MDL_DRDY_EXTI8_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(LIS3MDL_DRDY_EXTI8_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : WIFI_CS_Pin */
  GPIO_InitStruct.Pin = WIFI_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(WIFI_CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : WIFI_INT_Pin */
  GPIO_InitStruct.Pin = WIFI_INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(WIFI_INT_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  HAL_NVIC_SetPriority(EXTI3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */

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
