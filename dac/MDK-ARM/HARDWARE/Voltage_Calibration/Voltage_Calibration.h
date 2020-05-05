#ifndef __VOLTAGE_CALIBRATION_H
#define __VOLTAGE_CALIBRATION_H
#include "main.h"
u16 Uout_Convert(float Uout);//将要输出的电压值通过公式转化为，公式通过下面的自动测试函数，可以用表格拟合出
void Automatic_Voltage_Acquisition();	//测量电源输出电压，并和dac输出的控制电压一一对应，并以串口的形式输出		，这个函数跑完要很久做好散热				  
void Automatic_Current_Acquisition();//测量电源输出电流，和传感器返回电压一一对应，并以串口的形式输出		，这个函数跑完要很久做好散热




#endif