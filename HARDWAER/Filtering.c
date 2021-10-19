#include "Filtering.h"
#include "user_lib.h"


//TD2
//滤波之后的数据储存在ADRC结构体中的TD结构体中的TD2_out和TD2_out_dt

float out_temp,out_dt_temp;

float FAST_TD2(ADRC_t *ADRC,float input)
{
	//获取跟踪器参数
	float TD_h = ADRC->TD2.h,TD_delta = ADRC->TD2.delta;  
	
	//输出的滤波数据
	//out(k) = out(k-1) + t * out_dt;
	ADRC->TD2.out = out_temp + TD_h * out_dt_temp;
	
	//
	float err = out_temp - input;
	 ADRC->TD2.h = 0.01;
   ADRC->TD2.delta = 15;
	//
	float d = TD_delta * TD_h;
	float d0 = TD_h * d;
	float y = err + TD_h * out_dt_temp;
	float a0 = sqrt(d * d + 8 * TD_delta * fabs(y));
	float a;
	
	if(fabs(y) > d0)
     a = out_dt_temp + sign(y)*(a0-d)/2;
	else
     a = out_dt_temp + y / TD_h;
	
	float fst;
	if(fabs(a)>d)
			fst=-TD_delta*sign(a); 
	else
			fst=-TD_delta*a/d;
	
	ADRC->TD2.out_dt = out_dt_temp + TD_h * fst;
	
	out_temp 		= ADRC->TD2.out;
	out_dt_temp = ADRC->TD2.out_dt;
	
	return ADRC->TD2.out;

}
