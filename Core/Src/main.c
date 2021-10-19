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
#include "cmsis_os.h"
#include "can.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "bsp_can.h"
//#include "can_receive.h"
//#include "stdio.h"
//#include "pid.h"
//#include "Filtering.h"
//#include "bsp_rc.h"
#include "bsp_imu.h"
#include "remote_control.h"
//#include "Debug_task.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
//#define M6020_MOTOR_SPEED_PID_MAX_OUT 28000.0f
//#define M6020_MOTOR_SPEED_PID_MAX_IOUT 15000.0f //15000.0f
//#define M6020_MOTOR_ANGLE_PID_MAX_OUT 30.0f
//#define M6020_MOTOR_ANGLE_PID_MAX_IOUT 3.0f
//#define MAX_PID_SHOOT_OUT 13000.0f
//#define MAX_PID_SHOOT_IOUT 2000.0f

//#define lingmindu_y 1
//#define lingmindu_p 0.4
//#define SHOOT_START_FLAG 1
//#define SHOOT_STOP_FLAG 3
//#define REMOTE_CONTROL 2
//#define SHOOT1_CURRENT_DERECTION 1
//#define SHOOT2_CURRENT_DERECTION -1
//#define SHOOT_SPEED 10000
//#define SHOOT_STOP 0
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
//const motor_measure_t * moto;
//pid_type_def motor_speed_pid;
//pid_type_def motor_angle_pid;
//pid_type_def motor_speed_pid_p;
//pid_type_def motor_angle_pid_p;
//pid_type_def shoot1_pid_ref,shoot2_pid_ref;
//float My_PID_Speed[3] = {300,1.2,700};//30 1.9 500		
//float My_PID_Angle[3] = {2.5,0,10};//0.4 5
//float My_PID_Speed_p[3] = {200,0.05,200};
//float My_PID_Angle_p[3] = {10,0,5};
//fp32 shoot1[3]={10,0,0};
//fp32 shoot2[3]={10,0,0};

//motor_measure_t yaw_data;
//motor_measure_t pitch_data;
//motor_measure_t shoot1_motor_data;
//motor_measure_t shoot2_motor_data;
//extern motor_measure_t motor[8];
//gimbal_data_t wt;
//ADRC_t gyro_ADRC;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

//uint16_t motor_flag=1;

//float Goal_Speed,Angle=0,Angle_p=0;
//extern RC_ctrl_t rc_ctrl;
////const RC_ctrl_t *shoot_ctrl;
////const RC_ctrl_t *gimbal_ctrl;

//int16_t yaw_control;
//int16_t pitch_control;

//float set1=0,set2=0;
//int16_t num=0;
//float TD2_gyro,TD2_gyro_p;
//float give_shoot1_current;
//float give_shoot2_current;
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
  MX_CAN1_Init();
  MX_CAN2_Init();
  MX_TIM2_Init();
  MX_USART1_UART_Init();
  MX_DMA_Init();
  MX_UART5_Init();
  MX_TIM3_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
	HAL_TIM_Base_Start_IT(&htim3);
	can_filter_init();
  remote_control_init();
	imu_init();
	
//	PID_init(&motor_speed_pid, PID_POSITION,My_PID_Speed,M6020_MOTOR_SPEED_PID_MAX_OUT,M6020_MOTOR_SPEED_PID_MAX_IOUT);
//	PID_init(&motor_angle_pid, PID_POSITION,My_PID_Angle,M6020_MOTOR_ANGLE_PID_MAX_OUT,M6020_MOTOR_ANGLE_PID_MAX_IOUT);
//	PID_init(&motor_speed_pid_p,PID_POSITION,My_PID_Speed_p,M6020_MOTOR_SPEED_PID_MAX_OUT,M6020_MOTOR_SPEED_PID_MAX_IOUT);
//	PID_init(&motor_angle_pid_p,PID_POSITION,My_PID_Angle_p,M6020_MOTOR_SPEED_PID_MAX_OUT,M6020_MOTOR_SPEED_PID_MAX_IOUT);
//  PID_init(&shoot1_pid_ref,PID_POSITION,shoot1,MAX_PID_SHOOT_OUT,MAX_PID_SHOOT_IOUT);
//  PID_init(&shoot2_pid_ref,PID_POSITION,shoot2,MAX_PID_SHOOT_OUT,MAX_PID_SHOOT_IOUT);
//	gyro_ADRC.TD2.h = 0.01;
//	gyro_ADRC.TD2.delta = 200;
//	Angle=motor_chassis[4].angle;
//	Angle_p=motor_chassis[5].angle;
  /* USER CODE END 2 */

  /* Call init function for freertos objects (in freertos.c) */
  MX_FREERTOS_Init();
  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
//		get_imu_value(&wt);
//		
//		yaw_data= *get_yaw_gimbal_motor_measure_point();
//		pitch_data= *get_pitch_gimbal_motor_measure_point();
//		shoot1_motor_data = *get_shoot_motor_measure_point(0);
//	  shoot2_motor_data = *get_shoot_motor_measure_point(1);
//		
//		yaw_control=(float)rc_ctrl.rc.ch[2];
//		pitch_control=(float)rc_ctrl.rc.ch[3];

//		Angle=Angle-yaw_control*lingmindu_y/660.0;
//		Angle_p=Angle_p-pitch_control*lingmindu_p/660.0;
//		
//		if(Angle>=180)
//		{
//			Angle=-360+Angle;
//			
//		}
//		else if(Angle<-180)
//		{
//			Angle=360+Angle;
//		}
//		
//		if(Angle_p>=25)
//		{
//			Angle_p=25;
//		}
//		else if(Angle_p<-10)
//		{
//			Angle_p=-10;
//		}
		
		
		
		
//		//调试
//		if(motor_flag==1)
//		{
////			set1=5;
//			set2=10;
//		}else{
////			set1=30;
//			set2=-10;
//		}
//		
		
//		TD2_gyro = FAST_TD2(&gyro_ADRC,motor[4].speed_rpm);
//		PID_calc_angle(&motor_angle_pid,motor[4].angle,set1);
//		PID_calc_speed(&motor_speed_pid,TD2_gyro,motor_angle_pid.out);
//		TD2_gyro_p = FAST_TD2(&gyro_ADRC,motor[5].speed_rpm);
//		PID_calc_angle(&motor_angle_pid_p,motor[5].angle,set2);
//		PID_calc_speed(&motor_speed_pid_p,TD2_gyro_p,motor_speed_pid_p.out);
//  	CAN_cmd_gimbal(0,motor_speed_pid_p.out,0,0);
//		printf("%f,%f,%f,%f\n",motor_angle_pid.out,set1,TD2_gyro,motor_chassis[4].angle);
//		printf("%f,%f,%f,%f\n",motor_angle_pid_p.out,set2,TD2_gyro_p,motor_chassis[5].angle);
//		TD2_gyro = FAST_TD2(&gyro_ADRC,motor_chassis[4].speed_rpm);
//		TD2_gyro_p = FAST_TD2(&gyro_ADRC,motor_chassis[5].speed_rpm);
//		PID_calc_angle(&motor_angle_pid,motor_chassis[4].angle,Angle);
//		PID_calc_speed(&motor_speed_pid,TD2_gyro,motor_angle_pid.out);
//		PID_calc_angle(&motor_angle_pid_p,motor_chassis[5].angle,Angle_p);
//		PID_calc_speed(&motor_speed_pid_p,TD2_gyro_p,motor_angle_pid_p.out);
//  	CAN_cmd_gimbal(motor_speed_pid.out,motor_speed_pid_p.out,0,0);			//5678
//		CAN_cmd_gimbal(2000,4000,0,0);	




//	if(rc_ctrl.rc.s[0]==2)//右下档——无力
//	{
//		CAN_cmd_gimbal(0,0,0,0);	
//	}
//	if(rc_ctrl.rc.s[0]==3)//右中挡——遥控控制
//	{
//		   TD2_gyro = FAST_TD2(&gyro_ADRC,wt.v_yaw/10.0);
//		   TD2_gyro_p = FAST_TD2(&gyro_ADRC,wt.v_pitch/10.0);
//		   PID_calc_angle(&motor_angle_pid,wt.p_yaw,Angle);
//		   PID_calc_speed(&motor_speed_pid,TD2_gyro,motor_angle_pid.out);
//		   PID_calc_angle(&motor_angle_pid_p,wt.p_pitch,Angle_p);
//		   PID_calc_speed(&motor_speed_pid_p,TD2_gyro_p,motor_angle_pid_p.out);
//		   CAN_cmd_gimbal(-motor_speed_pid.out,motor_speed_pid_p.out,0,0);			//5678
//		   U1Printf("%f,%f\n",TD2_gyro,wt.v_yaw/10.0);
//		
//		switch(rc_ctrl.rc.s[1])
//		{
//			case REMOTE_CONTROL://2

//       break;
//			case SHOOT_START_FLAG://1
//				give_shoot1_current = SHOOT1_CURRENT_DERECTION*PID_calc_speed(&shoot1_pid_ref,motor[6].speed_rpm,SHOOT_SPEED);
//				give_shoot2_current = SHOOT2_CURRENT_DERECTION*PID_calc_speed(&shoot2_pid_ref,-motor[7].speed_rpm,SHOOT_SPEED);
//			  CAN_cmd_shoot(give_shoot1_current,give_shoot2_current);
//				break;
//			case SHOOT_STOP_FLAG://3
//				give_shoot1_current = PID_calc_speed(&shoot1_pid_ref,motor[6].speed_rpm,SHOOT_STOP);
//				give_shoot2_current = PID_calc_speed(&shoot2_pid_ref,motor[7].speed_rpm,SHOOT_STOP);
//			  CAN_cmd_shoot(give_shoot1_current,give_shoot2_current);
//			break;
//		}
//		
//	}  
//	
//	HAL_Delay(5);
}
		
		

		


    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */


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
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

//void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
//{
//		if(htim==&htim3)
//		{
//			num++;
//			if(num==4000)
//			{
//				motor_flag=-motor_flag;		
//				num=0;
//			}
//		}
//}

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
