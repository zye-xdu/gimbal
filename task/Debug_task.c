#include "Debug_task.h"


/********** 变量定义区 **********/

float set1=0,set2=0;
int16_t num=0;
uint16_t motor_flag=1;

/********** 变量定义区 **********/



/******* 调试任务入口函数 *******/

void Debug_task(void const * argument){
	for(;;)
	{
		   HAL_GPIO_TogglePin(LED2_GPIO_Port,LED2_Pin);
		if(motor_flag==1)
		{
//			set1=5;
			set2=10;
		}else{
//			set1=30;
			set2=-10;
		} 	
		
   U1Printf("%f,%f\n",TD2_gyro,wt.v_yaw/10.0);
		
//		 printf("%d,%f\n",chassis_motor_measure[0]->speed_rpm,set);
//		 printf("%d,%d,%f\n",chassis_motor_measure[1]->speed_rpm,chassis_motor_measure[1]->given_current,set);
//		 printf("%d,%d,%f\n",chassis_motor_measure[2]->speed_rpm,chassis_motor_measure[2]->given_current,set);
//		 printf("%d,%d,%f\n",chassis_motor_measure[3]->speed_rpm,chassis_motor_measure[3]->given_current,set);
			osDelay(5);
		
	} 
	
}


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
		if(htim==&htim3)
		{
			num++;
			if(num==4000)
			{
				motor_flag=-motor_flag;		
				num=0;
			}
		}
}

