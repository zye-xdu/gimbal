#include "Gimbal_task.h"


/********** ���������� **********/

//extern motor_measure_t motor_chassis[7];
//motor_measure_t yaw_data;
//motor_measure_t pitch_data;
//gimbal_data_t wt;
//ADRC_t gyro_ADRC;
//extern RC_ctrl_t rc_ctrl;

float Goal_Speed,Angle=0,Angle_p=0;
int16_t yaw_control;
int16_t pitch_control;
float TD2_gyro,TD2_gyro_p;


///********** ���������� **********/


///********** ���������� **********/
void Gimbal_task(void const * argument){
	for(;;){
  //�����뺯���ĵ�
   HAL_GPIO_TogglePin(LED3_GPIO_Port,LED3_Pin);
	 osDelay(1); 
	//pid��ʼ��
	 PID_init(&motor_speed_pid, PID_POSITION,My_PID_Speed,M6020_MOTOR_SPEED_PID_MAX_OUT,M6020_MOTOR_SPEED_PID_MAX_IOUT);
	 PID_init(&motor_angle_pid, PID_POSITION,My_PID_Angle,M6020_MOTOR_ANGLE_PID_MAX_OUT,M6020_MOTOR_ANGLE_PID_MAX_IOUT);
	 PID_init(&motor_speed_pid_p,PID_POSITION,My_PID_Speed_p,M6020_MOTOR_SPEED_PID_MAX_OUT,M6020_MOTOR_SPEED_PID_MAX_IOUT);
	 PID_init(&motor_angle_pid_p,PID_POSITION,My_PID_Angle_p,M6020_MOTOR_SPEED_PID_MAX_OUT,M6020_MOTOR_SPEED_PID_MAX_IOUT);
		//�����ǻ�ֵ
		get_imu_value(&wt);
		//����ֵ�˲�
		TD2_gyro = wt.v_yaw/10.0;//FAST_TD2(&gyro_ADRC,wt.v_yaw/10.0);
		TD2_gyro_p = wt.v_pitch/10.0;//FAST_TD2(&gyro_ADRC,wt.v_pitch/10.0);
		//ң��ָ��
		yaw_data= *get_yaw_gimbal_motor_measure_point();
		pitch_data= *get_pitch_gimbal_motor_measure_point();
    //ң��ͨ��
		yaw_control=(float)rc_ctrl.rc.ch[2];
		pitch_control=(float)rc_ctrl.rc.ch[3];
   //ң�ؿ��ƽǶ�Ŀ��ֵ
		Angle=Angle-yaw_control*lingmindu_y/660.0;
		Angle_p=Angle_p-pitch_control*lingmindu_p/660.0;
		//yawѭ���޷�
		if(Angle>=180)
		{
			Angle=-360+Angle;
			
		}
		else if(Angle<-180)
		{
			Angle=360+Angle;
		}
		//pitch�޷�
		if(Angle_p>=25)
		{
			Angle_p=25;
		}
		else if(Angle_p<-10)
		{
			Angle_p=-10;
		}
	
		//����״̬�������µ�
	if(rc_ctrl.rc.s[0]==2)
	{
		CAN_cmd_gimbal(0,0,0,0);	
	}
	//ң�ؿ��ơ������е�
	if(rc_ctrl.rc.s[0]==3)
	 {

		   //pid���㡪������
		   PID_calc_angle(&motor_angle_pid,wt.p_yaw,Angle);
		   PID_calc_speed(&motor_speed_pid,TD2_gyro,motor_angle_pid.out);
		   PID_calc_angle(&motor_angle_pid_p,wt.p_pitch,Angle_p);
		   PID_calc_speed(&motor_speed_pid_p,TD2_gyro_p,motor_angle_pid_p.out);
		   //can����
		  CAN_cmd_gimbal(-motor_speed_pid.out,motor_speed_pid_p.out,0,0);		

	 }
	 osDelay(5);
 }
}



/********** ���������� **********/