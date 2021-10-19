#ifndef __SHOOT_TASK_H
#define __SHOOT_TASK_H

#include "cmsis_os.h"
#include "struct_typedef.h"
#include "pid.h"
#include "CAN_receive.h"
#include "remote_control.h"

#define MAX_PID_SHOOT_OUT 13000.0f
#define MAX_PID_SHOOT_IOUT 2000.0f

#define SHOOT_START_FLAG 1
#define SHOOT_STOP_FLAG 3
#define REMOTE_CONTROL 2
#define SHOOT1_CURRENT_DERECTION 1
#define SHOOT2_CURRENT_DERECTION -1
#define SHOOT_SPEED 10000
#define SHOOT_STOP 0

pid_type_def shoot1_pid_ref,shoot2_pid_ref;
fp32 shoot1[3]={10,0,0};
fp32 shoot2[3]={10,0,0};
motor_measure_t shoot1_motor_data;
motor_measure_t shoot2_motor_data;

float give_shoot1_current;
float give_shoot2_current;

extern motor_measure_t motor[8];
extern RC_ctrl_t rc_ctrl;

#endif
