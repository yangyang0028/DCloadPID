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
#include "adc.h"
#include "dac.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define max(a, b)			(a>b? a:b)
#define min(a, b)			(a<b? a:b)
#define range(x, a, b)		(min(max(x, a), b))
#define exchange(a, b, tmp) (tmp=a, a=b, b=tmp)
#define myabs(x)			((x<0)? -x:x)

#define MIN_LIMIT 0

#define MAX_LIMIT 4000

typedef struct{
	float limit;		//输出限幅
	float target;		//目标输出量
	float feedback;		//实际输出量
	float Kp;
	float Ki;
	float Kd;
	float e_0;			//当前误差
	float e_1;			//上一次误差
	float sum_e;
}PID;


float pid_calc(PID *pid){
	float out;
	float ep, ei, ed;

	pid->e_0 = pid->target - pid->feedback;

	ep = pid->e_0;
	ei = pid->sum_e += pid->e_0;
	ed = pid->e_0  - pid->e_1;


	out = pid->Kp*ep + pid->Ki*ei + pid->Kd*ed;

	out = range(out, -pid->limit, pid->limit);

	pid->e_1 = pid->e_0;

	return out;
}

PID pid = {
	4000,	//输出限幅
	0,	//目标控制量
	0,	//反馈控制量

	0.006,	//Kp
	0.0055,	//Ki
	0.4,	//Kd

	0, 0, 0	//e
};
float	inc_val=0;
float	control_val=0;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
#define LEN_i 500
uint16_t lv_I[LEN_i];
uint16_t I_i=0;
uint32_t num_i=0;
uint16_t ping=0;
int ans=0;
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint16_t num=0;
uint32_t AD_DMA[2];


uint16_t pingjunzhi(){
	num_i-=lv_I[I_i];
	return (uint16_t)(num_i/LEN_i);
}

int fputc(int ch, FILE *f)
{
  HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, 0xffff);
  return ch;
}

int fgetc(FILE *f)
{
  uint8_t ch = 0;
  HAL_UART_Receive(&huart2, &ch, 1, 0xffff);
  return ch;
}



uint16_t read_num(){
	uint16_t num=0;
	if(HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_6)==1){
		num+=1;
	}
	if(HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_4)==1){
		num+=2;
	}
	if(HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_2)==1){
		num+=4;
	}
	if(HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_0)==1){
		num+=8;
	}
	return num;
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
  MX_ADC1_Init();
  MX_DAC_Init();
  MX_TIM3_Init();
  MX_USART2_UART_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
	HAL_TIM_Base_Start_IT(&htim3);
	HAL_TIM_Base_Start_IT(&htim2);
	HAL_ADCEx_Calibration_Start(&hadc1); 
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)    //定时器中断回调
{		
		if(htim==&htim3){
			HAL_ADC_Start_DMA(&hadc1, AD_DMA, 2);
			if(num==1){
				ans=450;
			}else if(num==2){
				ans=820;
			}else if(num==3){
				ans=1190;
			}else if(num==4){
				ans=1545;
			}else if(num==5){
				ans=1900;
			}else if(num==6){
				ans=2273;
			}else if(num==7){
				ans=2640;
			}else if(num==8){
				ans=3000;
			}else if(num==9){
				ans=3285;
			}else if(num==0){
				ans=3510;
			}
////				ans=450;
////				ans=820;
////			ans=1190;
////				ans=1545;
////			ans=1900;
////				ans=2273;
////			ans=2640;
////				ans=3000;
////				ans=3285;
//				ans=3510;
				pid.target=ans;
				inc_val = pid_calc(&pid);
				control_val = inc_val;
				control_val = range(control_val, MIN_LIMIT, MAX_LIMIT);
				HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, control_val);

		//		num=read_num();
		//		printf("n0.val=%d%c%c%c", (int)num, 0xff, 0xff, 0xff);
			
				lv_I[I_i]=AD_DMA[0];
				num_i+=AD_DMA[0];
				I_i++;
				if(I_i>=LEN_i){
					I_i=0;
				}
				ping=pingjunzhi();
				pid.feedback=ping;
		//		HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, 420);//1
		//		HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, 750);//2
		//		HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, 1100);//3
		//		HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, 1470);//4
		//    HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, 1845);//5
		//    HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, 2230);//6
		//			HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, 2640);//7
		//			HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, 3045);//8
		//			HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, 3045);
				HAL_DAC_Start(&hdac,DAC_CHANNEL_1);
		}
		if(htim==&htim2){
				num=read_num();
				printf("n0.val=%d%c%c%c", (int)control_val, 0xff, 0xff, 0xff);
				printf("add 1,0,%d%c%c%c", (int)control_val/21, 0xff, 0xff, 0xff);
				printf("n2.val=%d%c%c%c", (int)ping, 0xff, 0xff, 0xff);
				printf("add 5,0,%d%c%c%c", (int)ping/21, 0xff, 0xff, 0xff);
				printf("n1.val=%d%c%c%c", (int)ans, 0xff, 0xff, 0xff);

		}
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
