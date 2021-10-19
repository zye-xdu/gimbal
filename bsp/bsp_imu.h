#ifndef BSP_IMU_H
#define BSP_IMU_H

#include "struct_typedef.h"


#define IMU_BUFF_SIZE       100
#define IMU_PACK_LENGTH     11

typedef struct
{
    fp32    yaw;
    fp32    pitch;
    fp32    row;
} imu_value_t;

typedef struct
{
    imu_value_t angle;
    imu_value_t rate;
} imu_t;

typedef struct
{
		fp32 p_yaw;
		fp32 p_pitch;
		fp32 v_yaw;
		fp32 v_pitch;
}gimbal_data_t;
extern void imu_init(void);
extern void get_imu_value(gimbal_data_t *data);

#endif

