#include "UartData.h"
#include "main.h"
#include <stdio.h>
#include "usart.h"
#include "stddef.h"


extern UART_HandleTypeDef huart1;
extern uint8_t	aTxBuffer1[10];//0为不显示
extern uint8_t  aTxBuffer2[5];
extern frames Frames;
extern double adv1_real;
extern double adv1_IN6_real;//超级电容电压，按照比例放大之后的值
extern double adv1_IN7_real;//超级电容电流，按照比例放大之后的值
extern double adv2_IN2_real;//输出电压
extern double adv2_IN14_real;//输出电压，按照比例放大之后的值
extern double adv3_real;//输入电流，按照比例放大之后的值
extern double power_input;//输入功率
extern double power_cap;//超级电容充电功率
extern double power_output;//输出功率
void UartSend()
{
		
	//power_input
	aTxBuffer2[4]=Frames.power_input;//数据功能位
	sprintf(aTxBuffer1,"%.3lf\r\n",power_input);//\r\n
	aTxBuffer2[2]=(sizeof(aTxBuffer1));//数据长度位
	HAL_UART_Transmit(&huart1,aTxBuffer2, sizeof(aTxBuffer2),200);	
	HAL_UART_Transmit(&huart1,aTxBuffer1, sizeof(aTxBuffer1),200);
	//power_cap
	aTxBuffer2[4]=Frames.power_cap;//数据功能位
	sprintf(aTxBuffer1,"%.3lf\r\n",power_cap);//\r\n
	aTxBuffer2[2]=(sizeof(aTxBuffer1));//数据长度位
	HAL_UART_Transmit(&huart1,aTxBuffer2, sizeof(aTxBuffer2),200);	
	HAL_UART_Transmit(&huart1,aTxBuffer1, sizeof(aTxBuffer1),200);
	//power_output
	aTxBuffer2[4]=Frames.power_output;//数据功能位
	sprintf(aTxBuffer1,"%.3lf\r\n",power_output);//\r\n
	aTxBuffer2[2]=(sizeof(aTxBuffer1));//数据长度位
	HAL_UART_Transmit(&huart1,aTxBuffer2, sizeof(aTxBuffer2),200);	
	HAL_UART_Transmit(&huart1,aTxBuffer1, sizeof(aTxBuffer1),200);
	//输入电压
	aTxBuffer2[4]=Frames.Voltage_Input;//数据功能位
	sprintf(aTxBuffer1,"%.3lf\r\n",adv2_IN14_real);//\r\n
	aTxBuffer2[2]=(sizeof(aTxBuffer1));//数据长度位
	HAL_UART_Transmit(&huart1,aTxBuffer2, sizeof(aTxBuffer2),200);	
	HAL_UART_Transmit(&huart1,aTxBuffer1, sizeof(aTxBuffer1),200);
	//输出电压
	aTxBuffer2[4]=Frames.Voltage_Output;//数据功能位
	sprintf(aTxBuffer1,"%.3lf\r\n",adv2_IN2_real);//\r\n
	aTxBuffer2[2]=(sizeof(aTxBuffer1));//数据长度位
	HAL_UART_Transmit(&huart1,aTxBuffer2, sizeof(aTxBuffer2),200);	
	HAL_UART_Transmit(&huart1,aTxBuffer1, sizeof(aTxBuffer1),200);
	//输入电流
	aTxBuffer2[4]=Frames.Current_Input;//数据功能位
	sprintf(aTxBuffer1,"%.3lf\r\n",adv3_real);//\r\n
	aTxBuffer2[2]=(sizeof(aTxBuffer1));//数据长度位
	HAL_UART_Transmit(&huart1,aTxBuffer2, sizeof(aTxBuffer2),200);	
	HAL_UART_Transmit(&huart1,aTxBuffer1, sizeof(aTxBuffer1),200);
	//电容充电电流
	aTxBuffer2[4]=Frames.Current_Cap_Input;//数据功能位
	sprintf(aTxBuffer1,"%.3lf\r\n",adv1_IN7_real);//\r\n
	aTxBuffer2[2]=(sizeof(aTxBuffer1));//数据长度位
	HAL_UART_Transmit(&huart1,aTxBuffer2, sizeof(aTxBuffer2),200);	
	HAL_UART_Transmit(&huart1,aTxBuffer1, sizeof(aTxBuffer1),200);
	//电容充电电压
	aTxBuffer2[4]=Frames.Voltage_Cap_Input;//数据功能位
	sprintf(aTxBuffer1,"%.3lf\r\n",adv1_IN6_real);//\r\n
	aTxBuffer2[2]=(sizeof(aTxBuffer1));//数据长度位
	HAL_UART_Transmit(&huart1,aTxBuffer2, sizeof(aTxBuffer2),200);	
	HAL_UART_Transmit(&huart1,aTxBuffer1, sizeof(aTxBuffer1),200);

}