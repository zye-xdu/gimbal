#ifndef __GIMBAL_TASK_H
#define __GIMBAL_TASK_H

#include "cmsis_os.h"
#include "remote_control.h"
#include "struct_typedef.h"
#include "CAN_receive.h"
#include "pid.h"
#include "bsp_imu.h"
#include "Filtering.h"

//PID��ֵ
#define M6020_MOTOR_SPEED_PID_MAX_OUT 28000.0
#define M6020_MOTOR_SPEED_PID_MAX_IOUT 15000.0f
#define M6020_MOTOR_ANGLE_PID_MAX_OUT 30.0f
#define M6020_MOTOR_ANGLE_PID_MAX_IOUT 3.0f
//ң��������
#define lingmindu_y 1
#define lingmindu_p 0.7
//pid�ṹ��
pid_type_def motor_speed_pid;
pid_type_def motor_angle_pid;
pid_type_def motor_speed_pid_p;
pid_type_def motor_angle_pid_p;
//pid����ֵ
float My_PID_Speed[3] = {500,0.5,0};//300,1.2,700		
float My_PID_Angle[3] = {2.5,0,10};//2.5,0,10
float My_PID_Speed_p[3] = {100,0,0};//200,0.05,200
float My_PID_Angle_p[3] = {10,0,5};
//��̨�������
motor_measure_t yaw_data;
motor_measure_t pitch_data;
//ң��ָ��
extern RC_ctrl_t rc_ctrl;
//������
gimbal_data_t wt;
//�˲��ṹ��
ADRC_t gyro_ADRC;
extern void get_imu_value(gimbal_data_t *data);
#endif
