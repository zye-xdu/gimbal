#include "Shoot_task.h"

/********** 变量定义区 **********/



/********** 变量定义区 **********/



/******* 调试任务入口函数 *******/
void Shoot_task(void const * argument){
	while(1){
		HAL_GPIO_TogglePin(LED1_GPIO_Port,LED1_Pin);
		osDelay(1); 
	  PID_init(&shoot1_pid_ref,PID_POSITION,shoot1,MAX_PID_SHOOT_OUT,MAX_PID_SHOOT_IOUT);
    PID_init(&shoot2_pid_ref,PID_POSITION,shoot2,MAX_PID_SHOOT_OUT,MAX_PID_SHOOT_IOUT);
		
		shoot1_motor_data = *get_shoot_motor_measure_point(0);
		shoot2_motor_data = *get_shoot_motor_measure_point(1);
		
	if(rc_ctrl.rc.s[0]==3)//右中挡——遥控控制
	{		
		switch(rc_ctrl.rc.s[1])
		{
			case REMOTE_CONTROL://2

       break;
			case SHOOT_START_FLAG://1
				give_shoot1_current = SHOOT1_CURRENT_DERECTION*PID_calc_speed(&shoot1_pid_ref,motor[6].speed_rpm,SHOOT_SPEED);
				give_shoot2_current = SHOOT2_CURRENT_DERECTION*PID_calc_speed(&shoot2_pid_ref,-motor[7].speed_rpm,SHOOT_SPEED);
			  CAN_cmd_shoot(give_shoot1_current,give_shoot2_current);
				break;
			case SHOOT_STOP_FLAG://3
				give_shoot1_current = PID_calc_speed(&shoot1_pid_ref,motor[6].speed_rpm,SHOOT_STOP);
				give_shoot2_current = PID_calc_speed(&shoot2_pid_ref,motor[7].speed_rpm,SHOOT_STOP);
			  CAN_cmd_shoot(give_shoot1_current,give_shoot2_current);
			break;
		}
	}
		
	osDelay(5);
	}
}
