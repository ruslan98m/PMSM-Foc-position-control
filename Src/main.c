/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "can.h"
#include "dac.h"
#include "dma.h"
#include "spi.h"
#include "tim.h"
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "fmath.h"
#include "DWT.h"
#include "current_functions.h"
#include "PID_controller.h"
#include "motor_calibration.h"

#define ARM_MATH_CM4
#include "arm_math.h"
#include "usbd_cdc_if.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define PI_2_3 (float)PI*2.f/3.f
#define MAX_PWM TIM1->ARR
#define HALF_PWM (uint16_t)(MAX_PWM/2)

#define SHUNT_RES 0.005f
#define AMPL_GAIN 10
#define ENCODER_FID (float)(2*PI/524288.f)

#define EN_GATE_PIN GPIO_PIN_10
#define GAIN_PIN GPIO_PIN_7

#define PWM_FREQ 10000
#define SAMPLE_TIME (float)1/PWM_FREQ
#define ADC_2_VOLT (float)(1/66.32f)
#define ADC_2_CUR (float)(3.3/4095/SHUNT_RES/AMPL_GAIN)
	
#define __LED_D1_ON HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, 1)
#define __LED_D2_ON HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, 1)
#define __LED_D1_OFF HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, 0)
#define __LED_D2_OFF HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, 0)
#define __DAC_SET_V(signal) HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, (uint16_t)(signal*4095/10 + 2048))


//#define USB_COM
#define CAN_COM
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

uint16_t ind = 0;
uint32_t encoder_data;
float tm = 0;
float freq = 0.02f;



cur_dq_t signal;	
cur_dq_t space_voltage_vector;
cur_abc_t phase_voltage;

servo_t motor;
cur_dq_t space_current_vector;
cur_dq_t rotor_current_vector;

pi_t current_controller_d;
pi_t current_controller_q;
pi_t velocity_controller;
pi_t position_controller;
pll_t velocity_pll;

volatile adc_res_t adc_res[2];
uint8_t can_Tx_arr[8];
uint8_t usb_Tx_arr[11];
uint8_t can_Rx_arr[8];

uint16_t enc_buf[2];
uint8_t byte;
uint8_t a = 0;
CAN_TxHeaderTypeDef pHeader; //declare a specific header for message transmittions
uint32_t TxMailbox;
uint8_t a,r; //declare byte to be transmitted //declare a receive byte
CAN_FilterTypeDef sFilterConfig;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
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
  MX_CAN1_Init();
  MX_TIM1_Init();
  MX_USB_DEVICE_Init();
  MX_ADC2_Init();
  MX_DAC_Init();
  MX_TIM8_Init();
  MX_SPI1_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
	
	
	DWT_start_timer();
	
	HAL_ADC_Start(&hadc2);
	HAL_ADCEx_MultiModeStart_DMA(&hadc1, (uint32_t *)adc_res, 6);

	htim1.Instance->EGR |= TIM_EGR_UG;
	htim8.Instance->EGR |= TIM_EGR_UG;

	htim8.Instance->CCER = TIM_CCER_CC1E | TIM_CCER_CC4E;
	htim8.Instance->BDTR |= TIM_BDTR_MOE;

	__HAL_TIM_ENABLE(&htim8);

	//Timer 1 complementary mode start
	htim1.Instance->CCER = TIM_CCER_CC1E | TIM_CCER_CC1NE |
	TIM_CCER_CC2E | TIM_CCER_CC2NE |
	TIM_CCER_CC3E | TIM_CCER_CC3NE;
	htim1.Instance->BDTR |= TIM_BDTR_MOE;

	__HAL_TIM_ENABLE(&htim1);

	HAL_GPIO_WritePin(GPIOC, EN_GATE_PIN, 1);
	HAL_DAC_Start(&hdac, DAC1_CHANNEL_1);
	HAL_TIM_Base_Start_IT(&htim3);
	
	__HAL_SPI_ENABLE(&hspi1);
	
	//HAL_GPIO_WritePin(GPIOC, GAIN_PIN, 1);
	
	motor.pole_pairs = 10;
	motor.target_angle = 0.f;
	
	current_controller_d.Ki = 6;
	current_controller_d.Kp = 0.3f;
	current_controller_d.I_max = 20;
	current_controller_d.u_max = motor.voltage;
	current_controller_d.freq = PWM_FREQ;

	current_controller_q = current_controller_d;
	
	velocity_controller.Kp = 6;
	velocity_controller.Ki = 3;
	velocity_controller.freq = PWM_FREQ;
	
	position_controller.Ki = 0;
	position_controller.Kp = 8;
	position_controller.freq = PWM_FREQ;
	
	
	velocity_controller.I_max = 10;
	position_controller.I_max = 6;
	
	velocity_controller.u_max = 20;
	position_controller.u_max = 20;
	
	velocity_pll.controller.Kp = 50;
	velocity_pll.controller.Ki = 100;
	velocity_pll.controller.u_max = 10000;
	velocity_pll.controller.I_max = 10000;
	velocity_pll.controller.I_last = 0;
	velocity_pll.freq = PWM_FREQ;
	velocity_pll.controller.freq = velocity_pll.freq;
	velocity_pll.theta = 0;
	
	
	pHeader.DLC=8; //give message size of 1 byte
	pHeader.IDE=CAN_ID_STD; //set identifier to standard
	pHeader.RTR=0; //set data type to remote transmission request?
	pHeader.StdId=0x244; //define a standard identifier, used for message identification by filters (switch this for the other microcontroller)
	
	
		//filter one (stack light blink)
	sFilterConfig.FilterFIFOAssignment=CAN_FILTER_FIFO0; //set fifo assignment
	sFilterConfig.FilterIdHigh=0x000; 
	sFilterConfig.FilterIdLow=0x000; 
	sFilterConfig.FilterMaskIdHigh=0x0000;
	sFilterConfig.FilterMaskIdLow=0x0000;
	sFilterConfig.FilterScale=CAN_FILTERSCALE_32BIT; //set filter scale
	sFilterConfig.FilterActivation=ENABLE;  
	sFilterConfig.FilterBank = 0;
	sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
	sFilterConfig.SlaveStartFilterBank = 14;
	
	HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig); //configure CAN filter
	HAL_CAN_Start(&hcan1);
	
	HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING); //enable interrupts
	
	TIM3->ARR = 8400;
	
	
	motor_calibrate(&motor);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {


		//__LED_D1_ON;
		//__LED_D2_ON;
	
		
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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

  /** Configure the main internal regulator output voltage 
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

void DMA2_Stream0_IRQHandler()
{
//	tm += 2*PI*freq/PWM_FREQ;;
//	if(tm>2*PI)
//		tm -= 2*PI;
	__LED_D2_ON;
	volatile adc_res_t *res = (DMA2->LISR & DMA_LISR_TCIF0) ? adc_res + 1 : adc_res;
	motor.phase_current.a = ((res->cur_a_1 + res->cur_a_2)*0.5f - 2048)*ADC_2_CUR;
	motor.phase_current.b = ((res->cur_b_1 + res->cur_b_2)*0.5f - 2048)*ADC_2_CUR;
	motor.phase_current.c = - motor.phase_current.a - motor.phase_current.b;
	
	motor.voltage = (float)((res->volt_1 + res->volt_2)*0.5f*ADC_2_VOLT);
	
	current_controller_d.u_max = motor.voltage*ISQRT3;
	current_controller_q.u_max = motor.voltage*ISQRT3;
	current_controller_d.I_max = 0.8f*current_controller_d.u_max;
	current_controller_q.I_max = 0.8f*current_controller_q.u_max;

	
	space_current_vector = clark_transform(&motor.phase_current);
	rotor_current_vector = park_transform(&space_current_vector, motor.theta);
	
	motor.current_theta = _atan2_(rotor_current_vector.d, rotor_current_vector.q);
	motor.current = sqrtf(SQ(space_current_vector.d) + SQ(space_current_vector.q));
	
	//get encoder value from bytes array
	encoder_data = (((uint32_t)enc_buf[1] << 16 | (uint32_t)enc_buf[0]) >> 8) & ((1 << 19) - 1);
	
	motor.angle = encoder_data*ENCODER_FID;
	pll_update(&velocity_pll, motor.angle);
	motor.velocity = velocity_pll.velocity;
	motor.theta = fmodf(motor.angle*motor.pole_pairs, 2*PI);

	pi_update(&current_controller_d, - rotor_current_vector.d);
	pi_update(&current_controller_q,  velocity_controller.u - rotor_current_vector.q);
	pi_update(&velocity_controller, position_controller.u - motor.velocity);
	pi_update(&position_controller,  motor.target_angle - velocity_pll.theta);
	
	signal.q = current_controller_q.u;
	signal.d = current_controller_d.u;
	
	motor.rotor_vector.d = current_controller_d.u;
	motor.rotor_vector.q = current_controller_q.u;
	
	space_voltage_vector = inv_park_transform(&signal, motor.theta);
	phase_voltage = inv_clark_transform(&space_voltage_vector);

	//TIM8->CCR1 = CCR1;
	//TIM1->CCR1 = (uint16_t)(k*sinf(tm) + HALF_PWM);
	//TIM1->CCR2 = (uint16_t)(k*sinf(tm+PI_2_3) + HALF_PWM);
	//TIM1->CCR3 = (uint16_t)(k*sinf(tm+2*PI_2_3) + HALF_PWM);
	
	TIM1->CCR1 = phase_voltage.b*HALF_PWM/motor.voltage + HALF_PWM;
	TIM1->CCR2 = phase_voltage.c*HALF_PWM/motor.voltage + HALF_PWM;
	TIM1->CCR3 = phase_voltage.a*HALF_PWM/motor.voltage + HALF_PWM;

	
	HAL_SPI_Receive(&hspi1, (uint8_t *)enc_buf, 2, 50);
	
	DMA2->LIFCR = DMA_LIFCR_CTCIF0 | DMA_LIFCR_CHTIF0;
	__LED_D2_OFF;
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	
	if(htim->Instance == TIM3){
		
	#ifdef CAN_COM	
		memcpy(can_Tx_arr, &motor.angle, 4);
		memcpy(can_Tx_arr + 4, &motor.current, 4);
		HAL_CAN_AddTxMessage(&hcan1, &pHeader, (uint8_t *)can_Tx_arr, &TxMailbox);  //function to add message for transmition
	#endif //USB_COM
		
	#ifdef USB_COM
		usb_Tx_arr[0] = 0xff;
		memcpy(usb_Tx_arr+1, &motor.angle, 4);
		memcpy(usb_Tx_arr+5, &motor.rotor_vector.q, 4);
		CDC_Transmit_FS((uint8_t *)usb_Tx_arr, 13);
	#endif //USB_COM
	}
	
}
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	__LED_D1_ON;
  CAN_RxHeaderTypeDef msgHeader;
  uint32_t msgId = 0;
  
  HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &msgHeader, can_Rx_arr);
  motor.target_angle = *(float *)can_Rx_arr ;	
  if (msgHeader.IDE == CAN_ID_EXT)
  {
    msgId = msgHeader.ExtId;
  }
  else
  {
    msgId = msgHeader.StdId;
  }
	__LED_D1_OFF;
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
    printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
