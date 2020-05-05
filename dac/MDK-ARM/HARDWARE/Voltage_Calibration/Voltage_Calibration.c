#include "Voltage_Calibration.h"
#include "delay.h"
#include "adc.h"
#include "dac.h"
#include "Data_Processing.h"
#include <stdio.h>
#include "usart.h"
u16 Uout_Convert(float Uout)
{
	u16 x=0;
	//x=(u16)((37.55-Uout)/0.0093-0.4);
	x=(u16)((-103.9*Uout)+ 3903.2);

	return x;
	
}
void Automatic_Voltage_Acquisition()
{
	u16 DAC_Value_AVA=4096;
	uint16_t ADC_ConvertedValue_PA6_AVA[10] = {0};//改成uint32_t，数据会溢出，存adc的数据
	u16 out_ADCxConvertedValue_PA6_AVA=0;
	float prevData_AVA=0; 
	float adv_real_AVA=0;
	for(int i=0 ;i<4096;i++)
	{
		//输出DAC
	 	HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, (DAC_Value_AVA-i));//从4096开始减到1
		HAL_DAC_Start(&hdac, DAC_CHANNEL_1);
		//等待电源稳定
		for(int n=0;n<10;n++)
		{
			delay_ms(10);
		}
	
		//输入ADC
		HAL_ADC_Start_DMA(&hadc1, (uint32_t*)&ADC_ConvertedValue_PA6_AVA,10);//用ADC的DMA通道读取10个数据
		delay_us(50);//不延时adc数据不会更新,延时对数据结果有影响太小数据就有便宜，接地时有1.0v拉高时偶2.v,推测和采样次数有关
		HAL_ADC_Stop_DMA(&hadc1);//ADC的DMA通道关闭，一直开会影响其他数据
		out_ADCxConvertedValue_PA6_AVA=AGV_Remove_the_Ross_Error(ADC_ConvertedValue_PA6_AVA,10);//粗大误差剔除
		out_ADCxConvertedValue_PA6_AVA=kalmanFilter_A(out_ADCxConvertedValue_PA6_AVA,&prevData_AVA);
		adv_real_AVA=out_ADCxConvertedValue_PA6_AVA*0.00080566*10.8;//0.00080566为分辨率，11为电阻分压后的放大倍数
		
		//创建字符串
		printf("DAC_out:%d  DAC_in:%f \r\n",(DAC_Value_AVA-i),adv_real_AVA); 
//		sprintf((char*)aTxBuffer_AVA,"DAC_out:%d  DAC_in:%f \r\n",(DAC_Value_AVA-i),adv_real_AVA);
		//串口发送字符串
//		HAL_UART_Transmit_DMA(&huart1,aTxBuffer_AVA,sizeof(aTxBuffer_AVA));// DMA发生数据
		//等待数据发送完毕
		delay_ms(100);
		
	}

}
/******************************************************************************
函数原型：void Automatic_Current_Acquisition()
函数功能：把电流传感器返回的adc值和计算出的真实电流打印到串口中
参数说明：用PA0口，ADC通道3：hadc3
返回值：  
********************************************************************************/
void Automatic_Current_Acquisition()
{
	u16 DAC_Value_AVA=3940;
	uint16_t ADC_ConvertedValue_PA0_AVA[10] = {0};//改成uint32_t，数据会溢出，存adc的数据
	u16 out_ADCxConvertedValue_PA0_AVA=0;
	float prevData_AVA=0; 
	float adv_real_AVA=0;
	float current_value=0;
	for(int i=0 ;i<4096;i++)
	{
		//输出DAC
	 	HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, (DAC_Value_AVA-i));//从4096开始减到1
		HAL_DAC_Start(&hdac, DAC_CHANNEL_1);
		//等待电源稳定
		for(int n=0;n<5;n++)//等待时间视具体情况而定
		{
			delay_ms(100);
		}
	
		//输入ADC
		HAL_ADC_Start_DMA(&hadc3, (uint32_t*)&ADC_ConvertedValue_PA0_AVA,10);//用ADC的DMA通道读取10个数据
		delay_us(50);//不延时adc数据不会更新,延时对数据结果有影响太小数据就有便宜，接地时有1.0v拉高时偶2.v,推测和采样次数有关
		HAL_ADC_Stop_DMA(&hadc3);//ADC的DMA通道关闭，一直开会影响其他数据
		out_ADCxConvertedValue_PA0_AVA = AGV_Remove_the_Ross_Error(ADC_ConvertedValue_PA0_AVA,10);//粗大误差剔除
		out_ADCxConvertedValue_PA0_AVA = kalmanFilter_A(out_ADCxConvertedValue_PA0_AVA,&prevData_AVA);//卡尔曼滤波
		adv_real_AVA=out_ADCxConvertedValue_PA0_AVA*0.00080566;//0.00080566为分辨率，11为电阻分压后的放大倍数
		current_value= ((-0.0062)*(DAC_Value_AVA-i)+24.71)/18;
		//创建字符串
		printf("current_out:%f : current_in:%f \r\n",current_value,adv_real_AVA); 
//		sprintf((char*)aTxBuffer_AVA,"DAC_out:%d  DAC_in:%f \r\n",(DAC_Value_AVA-i),adv_real_AVA);
		//串口发送字符串
//		HAL_UART_Transmit_DMA(&huart1,aTxBuffer_AVA,sizeof(aTxBuffer_AVA));// DMA发生数据
		//等待数据发送完毕
		delay_ms(100);
		
	}

}