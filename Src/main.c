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
#include "i2c.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include "../../../Hardware/MPU6050/mpu6050.h"
#include "../../../Hardware/MPU6050/delay.h"
#include "../../../Hardware/MPU6050/eMPL/inv_mpu.h"
#include "../../../Hardware/MPU6050/eMPL/inv_mpu_dmp_motion_driver.h" 
#include "stm32f1xx_it.h"
#include <math.h>
#include <stdlib.h>

#include "PID1.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
//int fputc(int ch, FILE *f)
//{
//    uint8_t temp[1]={ch};
//    HAL_UART_Transmit(&huart1, temp, 1, HAL_MAX_DELAY);
//    return ch;
//}
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
float pitch,roll,yaw; 		//欧拉角
short aacx,aacy,aacz;		//加速度传感器原始数据
short gyrox,gyroy,gyroz;	//陀螺仪原始数据
short temp;					//温度
extern float SetAngle1;
extern float SetAngle2;
unsigned char data1[8]={'A','0','0','B','0','0','0','0'};  //气阀1
unsigned char data2[8]={'A','0','1','B','0','0','0','0'};  //气阀2
unsigned char data3[8]={'A','0','2','B','0','0','0','0'};  //气阀3
unsigned char data4[8]={'A','0','3','B','0','0','0','0'};  //气阀4
int a, b, c, d;
extern int i, k;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#define USART_REC_LEN 200
uint8_t		Res;
uint8_t 	USART1_RX_BUF[USART_REC_LEN];     //接收缓冲,最大USART_REC_LEN个字节.（一般给200，接收数据量大就增加）
uint16_t  USART1_RX_STA;       			 				//接收状态标记
float voltage_out1=0;
float voltage_out2=0;

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
	//开启接收中断
	HAL_UART_Receive_IT(&huart1, &Res, 1);
	//发送初始化完成提示
	HAL_UART_Transmit_IT(&huart1,"USART1_Ready!",sizeof("USART1_Ready!"));
	//等待发送完成
	while(huart1.gState != HAL_UART_STATE_READY){};
		if(USART1_RX_STA & 0x8000)
		{
			HAL_UART_Transmit_IT(&huart1,"Get_Dat:",sizeof("Get_Dat:"));
			//等待发送完成
			while(huart1.gState != HAL_UART_STATE_READY){};
			//发送接收到的字节
			HAL_UART_Transmit_IT(&huart1,USART1_RX_BUF,USART1_RX_STA&0x7FFF);
			while(huart1.gState != HAL_UART_STATE_READY){};
			
//			if(USART1_RX_BUF[0]== 'A')
//					HAL_GPIO_WritePin(LED0_GPIO_Port,LED0_Pin,GPIO_PIN_RESET);
//			if(USART1_RX_BUF[0]== 'B')
//					HAL_GPIO_WritePin(LED1_GPIO_Port,LED1_Pin,GPIO_PIN_RESET);
//			if(USART1_RX_BUF[0]== 'C')
//			{
//					HAL_GPIO_WritePin(LED0_GPIO_Port,LED0_Pin,GPIO_PIN_SET);
//					HAL_GPIO_WritePin(LED1_GPIO_Port,LED1_Pin,GPIO_PIN_SET);
//			}
			USART1_RX_STA=0;
		}
		HAL_Delay(100);

//  /* USER CODE BEGIN 1 */
//  pid1_struct pid_roll_structure;
//	pid1_struct pid_pitch_structure;
//  /* USER CODE END 1 */

//  /* MCU Configuration--------------------------------------------------------*/

//  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
//  HAL_Init();

//  /* USER CODE BEGIN Init */
//  PID1_init(&pid_roll_structure, &pid_pitch_structure);
//  /* USER CODE END Init */

//  /* Configure the system clock */
//  SystemClock_Config();

//  /* USER CODE BEGIN SysInit */

//  /* USER CODE END SysInit */

//  /* Initialize all configured peripherals */
//  MX_GPIO_Init();
//  MX_I2C1_Init();
//  MX_USART1_UART_Init();
//  MX_USART2_UART_Init();
//  /* USER CODE BEGIN 2 */
//	while(MPU_Init());					//初始化MPU6050
//	printf("%s\r\n","Mpu6050 Initializing");
//	while(mpu_dmp_init())
//	{
//		delay_ms(200);
//		printf("%s\r\n","Mpu6050 Init Wrong!");
//	}
//	printf("%s\r\n","Mpu6050 Init OK!");
//  
//  /* USER CODE END 2 */

//  /* Infinite loop */
//  /* USER CODE BEGIN WHILE */
//  while (1)
//  {
//    /* USER CODE END WHILE */

//    /* USER CODE BEGIN 3 */
//		if(k == 1)
//		{
//		printf("here\r\n");	
//		delay_ms(100);
//		k = 0;
//		while(abs(SetAngle1-pitch)>0.5 || abs(SetAngle2-roll)>0.5)
//		{
//		while(mpu_dmp_get_data(&pitch,&roll,&yaw))
//		{
//			printf("pitch=%f  roll=%f  \r\n", pitch, roll);
//			voltage_out1 = PID1_out(SetAngle1, pitch, &pid_pitch_structure);
//			voltage_out2 = PID1_out(SetAngle2, roll, &pid_roll_structure);
//			printf("SetAngle1=%f  SetAngle2=%f  \r\n", SetAngle1, SetAngle2);
//			printf("voltageout1=%f  voltageout2=%f  \r\n", voltage_out1, voltage_out2);
////			temp=MPU_Get_Temperature();								//得到温度值
////			MPU_Get_Accelerometer(&aacx,&aacy,&aacz);	//得到加速度传感器数据
////			MPU_Get_Gyroscope(&gyrox,&gyroy,&gyroz);	//得到陀螺仪数据
////			printf("三轴角度：%f-%f-%f\r\n",pitch,roll,yaw);
////			printf("三轴加速度：%d-%d-%d\r\n",aacx,aacy,aacz);
////			printf("三轴角角度：%d-%d-%d\r\n",gyrox,gyroy,gyroz);
//		}
//		delay_ms(100);
//		if(voltage_out1 < 0)
//		{
//			voltage_out1 = (-voltage_out1) * 1000;
//      a = voltage_out1 / 1000;
//			b = (voltage_out1 - a * 1000)/100;
//			c = (voltage_out1 - a * 1000 - b * 100);
//			d = voltage_out1 - a * 1000 - b * 100 - c * 10;
//			data1[4]=a+'0';
//	    data1[5]=b+'0';
//	    data1[6]=c+'0';
//	    data1[7]=d+'0';	
//			for(i=4;i<8;i++)
//			{
//				data2[i] = 0 + '0';
//			}
//		}
//		else if(voltage_out1 > 0)
//		{
//      a = voltage_out1 / 1000;
//			b = (voltage_out1 - a * 1000)/100;
//			c = (voltage_out1 - a * 1000 - b * 100);
//			d = voltage_out1 - a * 1000 - b * 100 - c * 10;
//			data2[4]=a+'0';
//	    data2[5]=b+'0';
//	    data2[6]=c+'0';
//	    data2[7]=d+'0';	
//			for(i=4;i<8;i++)
//			{
//				data1[i] = 0 + '0';
//			}			
//		
//		}
//		if(voltage_out2 < 0)
//		{
//			voltage_out2 = (-voltage_out2) * 1000;
//      a = voltage_out2 / 1000;
//			b = (voltage_out2 - a * 1000)/100;
//			c = (voltage_out2 - a * 1000 - b * 100);
//			d = voltage_out2 - a * 1000 - b * 100 - c * 10;
//			data3[4]=a+'0';
//	    data3[5]=b+'0';
//	    data3[6]=c+'0';
//	    data3[7]=d+'0';	
//			for(i=4;i<8;i++)
//			{
//				data4[i] = 0 + '0';
//			}
//		}
//		else if(voltage_out2 > 0)
//		{
//      a = voltage_out2 / 1000;
//			b = (voltage_out2 - a * 1000)/100;
//			c = (voltage_out2 - a * 1000 - b * 100);
//			d = voltage_out2 - a * 1000 - b * 100 - c * 10;
//			data4[4]=a+'0';
//	    data4[5]=b+'0';
//	    data4[6]=c+'0';
//	    data4[7]=d+'0';	
//			for(i=4;i<8;i++)
//			{
//				data3[i] = 0 + '0';
//			}			
//		
//		}
//	  HAL_UART_Transmit(&huart2,data1 ,8,10);       //串口2发送数据给气阀控制板
//	  HAL_Delay(10);
//	  HAL_UART_Transmit(&huart2,data2 ,8,10);
//	  HAL_Delay(10);
//	  HAL_UART_Transmit(&huart2,data3 ,8,10);
//	  HAL_Delay(10);
//	  HAL_UART_Transmit(&huart2,data4 ,8,10);
//		
//		HAL_UART_Transmit(&huart1,data1 ,8,10);       //串口1发送数据
//	  HAL_UART_Transmit(&huart1,data2 ,8,10);
//	  HAL_UART_Transmit(&huart1,data3 ,8,10);
//	  HAL_UART_Transmit(&huart1,data4 ,8,10);
//		printf("\r\n");
//		HAL_Delay(10);
//  }
//  }
//}
//  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

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
}

/* USER CODE BEGIN 4 */

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    /* 判断是哪个串口触发的中断 */
    if(huart ->Instance == USART1)
    {
        		//处理接受到的数据
				if((USART1_RX_STA&0x8000)==0)//接收未完成
				{
					//读取接收到的数据
					if(Res==0x0D)
					{
						USART1_RX_STA|=0x8000;
						HAL_UART_Receive_IT(&huart1, &Res, 1);
					}
					else
					{
						USART1_RX_BUF[USART1_RX_STA&0X3FFF]=Res ;
						USART1_RX_STA++;
						if(USART1_RX_STA>(USART_REC_LEN-1))
							USART1_RX_STA=0;
					}		 
				}
				//等待下一次接收中断
				HAL_UART_Receive_IT(huart,&Res,1);
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
