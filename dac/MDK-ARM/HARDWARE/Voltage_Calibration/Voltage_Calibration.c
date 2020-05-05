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
	uint16_t ADC_ConvertedValue_PA6_AVA[10] = {0};//�ĳ�uint32_t�����ݻ��������adc������
	u16 out_ADCxConvertedValue_PA6_AVA=0;
	float prevData_AVA=0; 
	float adv_real_AVA=0;
	for(int i=0 ;i<4096;i++)
	{
		//���DAC
	 	HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, (DAC_Value_AVA-i));//��4096��ʼ����1
		HAL_DAC_Start(&hdac, DAC_CHANNEL_1);
		//�ȴ���Դ�ȶ�
		for(int n=0;n<10;n++)
		{
			delay_ms(10);
		}
	
		//����ADC
		HAL_ADC_Start_DMA(&hadc1, (uint32_t*)&ADC_ConvertedValue_PA6_AVA,10);//��ADC��DMAͨ����ȡ10������
		delay_us(50);//����ʱadc���ݲ������,��ʱ�����ݽ����Ӱ��̫С���ݾ��б��ˣ��ӵ�ʱ��1.0v����ʱż2.v,�Ʋ�Ͳ��������й�
		HAL_ADC_Stop_DMA(&hadc1);//ADC��DMAͨ���رգ�һֱ����Ӱ����������
		out_ADCxConvertedValue_PA6_AVA=AGV_Remove_the_Ross_Error(ADC_ConvertedValue_PA6_AVA,10);//�ִ�����޳�
		out_ADCxConvertedValue_PA6_AVA=kalmanFilter_A(out_ADCxConvertedValue_PA6_AVA,&prevData_AVA);
		adv_real_AVA=out_ADCxConvertedValue_PA6_AVA*0.00080566*10.8;//0.00080566Ϊ�ֱ��ʣ�11Ϊ�����ѹ��ķŴ���
		
		//�����ַ���
		printf("DAC_out:%d  DAC_in:%f \r\n",(DAC_Value_AVA-i),adv_real_AVA); 
//		sprintf((char*)aTxBuffer_AVA,"DAC_out:%d  DAC_in:%f \r\n",(DAC_Value_AVA-i),adv_real_AVA);
		//���ڷ����ַ���
//		HAL_UART_Transmit_DMA(&huart1,aTxBuffer_AVA,sizeof(aTxBuffer_AVA));// DMA��������
		//�ȴ����ݷ������
		delay_ms(100);
		
	}

}
/******************************************************************************
����ԭ�ͣ�void Automatic_Current_Acquisition()
�������ܣ��ѵ������������ص�adcֵ�ͼ��������ʵ������ӡ��������
����˵������PA0�ڣ�ADCͨ��3��hadc3
����ֵ��  
********************************************************************************/
void Automatic_Current_Acquisition()
{
	u16 DAC_Value_AVA=3940;
	uint16_t ADC_ConvertedValue_PA0_AVA[10] = {0};//�ĳ�uint32_t�����ݻ��������adc������
	u16 out_ADCxConvertedValue_PA0_AVA=0;
	float prevData_AVA=0; 
	float adv_real_AVA=0;
	float current_value=0;
	for(int i=0 ;i<4096;i++)
	{
		//���DAC
	 	HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, (DAC_Value_AVA-i));//��4096��ʼ����1
		HAL_DAC_Start(&hdac, DAC_CHANNEL_1);
		//�ȴ���Դ�ȶ�
		for(int n=0;n<5;n++)//�ȴ�ʱ���Ӿ����������
		{
			delay_ms(100);
		}
	
		//����ADC
		HAL_ADC_Start_DMA(&hadc3, (uint32_t*)&ADC_ConvertedValue_PA0_AVA,10);//��ADC��DMAͨ����ȡ10������
		delay_us(50);//����ʱadc���ݲ������,��ʱ�����ݽ����Ӱ��̫С���ݾ��б��ˣ��ӵ�ʱ��1.0v����ʱż2.v,�Ʋ�Ͳ��������й�
		HAL_ADC_Stop_DMA(&hadc3);//ADC��DMAͨ���رգ�һֱ����Ӱ����������
		out_ADCxConvertedValue_PA0_AVA = AGV_Remove_the_Ross_Error(ADC_ConvertedValue_PA0_AVA,10);//�ִ�����޳�
		out_ADCxConvertedValue_PA0_AVA = kalmanFilter_A(out_ADCxConvertedValue_PA0_AVA,&prevData_AVA);//�������˲�
		adv_real_AVA=out_ADCxConvertedValue_PA0_AVA*0.00080566;//0.00080566Ϊ�ֱ��ʣ�11Ϊ�����ѹ��ķŴ���
		current_value= ((-0.0062)*(DAC_Value_AVA-i)+24.71)/18;
		//�����ַ���
		printf("current_out:%f : current_in:%f \r\n",current_value,adv_real_AVA); 
//		sprintf((char*)aTxBuffer_AVA,"DAC_out:%d  DAC_in:%f \r\n",(DAC_Value_AVA-i),adv_real_AVA);
		//���ڷ����ַ���
//		HAL_UART_Transmit_DMA(&huart1,aTxBuffer_AVA,sizeof(aTxBuffer_AVA));// DMA��������
		//�ȴ����ݷ������
		delay_ms(100);
		
	}

}