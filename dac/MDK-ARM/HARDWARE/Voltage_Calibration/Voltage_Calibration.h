#ifndef __VOLTAGE_CALIBRATION_H
#define __VOLTAGE_CALIBRATION_H
#include "main.h"
u16 Uout_Convert(float Uout);//��Ҫ����ĵ�ѹֵͨ����ʽת��Ϊ����ʽͨ��������Զ����Ժ����������ñ����ϳ�
void Automatic_Voltage_Acquisition();	//������Դ�����ѹ������dac����Ŀ��Ƶ�ѹһһ��Ӧ�����Դ��ڵ���ʽ���		�������������Ҫ�ܾ�����ɢ��				  
void Automatic_Current_Acquisition();//������Դ����������ʹ��������ص�ѹһһ��Ӧ�����Դ��ڵ���ʽ���		�������������Ҫ�ܾ�����ɢ��




#endif