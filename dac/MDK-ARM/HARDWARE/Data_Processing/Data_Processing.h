#ifndef __DATE_PROCESSING_H
#define __DATE_PROCESSING_H	 
							  
////////////////////////////////////////////////////////////////////////////////// 	 




#include "stdint.h"
/*����ṹ��*/
volatile typedef struct
{
	float temp;
 	float setpoint;       //�趨ֵ
	float Kp;     //����ϵ��
	float Kp2;     //����ϵ��
	float Kd;      //����ϵ��
	float Ki;    //΢��ϵ��
	
	float lasterror;     //ǰһ��ƫ��   
	float preerror;     //ǰ����ƫ��
	float result; //���ֵ
}PID;//���������˸�����ֵ����Ϊ�������

void PID_int(PID*pid);//PID������ʼ��
void PID_power_cap_init(PID *vPID);
void PID_power_out_init(PID *vPID);
void PIDRegulation(volatile  PID *vPID, float processValue);
uint16_t Average_Value(uint16_t* date,int Length);
double Standard_Deviation(uint16_t* date,int Length);
double AGV_Remove_the_Ross_Error(uint16_t* date,int Length);
float kalmanFilter_A(float inData,float*prevData) ;
#endif