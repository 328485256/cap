#ifndef __DATE_PROCESSING_H
#define __DATE_PROCESSING_H	 
							  
////////////////////////////////////////////////////////////////////////////////// 	 




#include "stdint.h"
/*定义结构体*/
volatile typedef struct
{
	float temp;
 	float setpoint;       //设定值
	float Kp;     //比例系数
	float Kp2;     //比例系数
	float Kd;      //积分系数
	float Ki;    //微分系数
	
	float lasterror;     //前一拍偏差   
	float preerror;     //前两拍偏差
	float result; //输出值
}PID;//比上面少了个积分值，因为相减掉了

void PID_int(PID*pid);//PID参数初始化
void PID_power_cap_init(PID *vPID);
void PID_power_out_init(PID *vPID);
void PIDRegulation(volatile  PID *vPID, float processValue);
uint16_t Average_Value(uint16_t* date,int Length);
double Standard_Deviation(uint16_t* date,int Length);
double AGV_Remove_the_Ross_Error(uint16_t* date,int Length);
float kalmanFilter_A(float inData,float*prevData) ;
#endif