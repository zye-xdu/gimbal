
#ifndef FILTERING_H_
#define FILTERING_H_

#include "main.h"

#include <math.h>
#include "stdint.h"
typedef struct
{
	float out;
	float out_dt;
	
	//TD2微分跟踪器参数
	float h;			//跟踪步长
	float delta;	//跟踪速度

}TD2_t;


typedef struct
{
	//TD2微分跟踪器
	TD2_t TD2;
	
	//ESO
	float ESO_z[3];			//状态变量
	float ESO_beta[3];	//补偿系数
	float ESO_alfa1;
	float ESO_alfa2;
	float ESO_delta;	//ESO跟踪系数
	float ESO_h;
	float ESO_b;		//观测补偿增益
	
	
	//非线性PD控制器
	float PD_belta1,PD_belta2;
	float PD_alpha1,PD_alpha2,PD_delta;
	
	
	float last_u,now_u;
	

}ADRC_t;


float FAST_TD2(ADRC_t *ADRC,float input);
extern ADRC_t gyro_ADRC;
#endif /* FILTERING_H_ */
