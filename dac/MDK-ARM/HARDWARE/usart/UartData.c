#include "UartData.h"
#include "main.h"
#include <stdio.h>
#include "usart.h"
#include "stddef.h"


extern UART_HandleTypeDef huart1;
extern uint8_t	aTxBuffer1[10];//0Ϊ����ʾ
extern uint8_t  aTxBuffer2[5];
extern frames Frames;
extern double adv1_real;
extern double adv1_IN6_real;//�������ݵ�ѹ�����ձ����Ŵ�֮���ֵ
extern double adv1_IN7_real;//�������ݵ��������ձ����Ŵ�֮���ֵ
extern double adv2_IN2_real;//�����ѹ
extern double adv2_IN14_real;//�����ѹ�����ձ����Ŵ�֮���ֵ
extern double adv3_real;//������������ձ����Ŵ�֮���ֵ
extern double power_input;//���빦��
extern double power_cap;//�������ݳ�繦��
extern double power_output;//�������
void UartSend()
{
		
	//power_input
	aTxBuffer2[4]=Frames.power_input;//���ݹ���λ
	sprintf(aTxBuffer1,"%.3lf\r\n",power_input);//\r\n
	aTxBuffer2[2]=(sizeof(aTxBuffer1));//���ݳ���λ
	HAL_UART_Transmit(&huart1,aTxBuffer2, sizeof(aTxBuffer2),200);	
	HAL_UART_Transmit(&huart1,aTxBuffer1, sizeof(aTxBuffer1),200);
	//power_cap
	aTxBuffer2[4]=Frames.power_cap;//���ݹ���λ
	sprintf(aTxBuffer1,"%.3lf\r\n",power_cap);//\r\n
	aTxBuffer2[2]=(sizeof(aTxBuffer1));//���ݳ���λ
	HAL_UART_Transmit(&huart1,aTxBuffer2, sizeof(aTxBuffer2),200);	
	HAL_UART_Transmit(&huart1,aTxBuffer1, sizeof(aTxBuffer1),200);
	//power_output
	aTxBuffer2[4]=Frames.power_output;//���ݹ���λ
	sprintf(aTxBuffer1,"%.3lf\r\n",power_output);//\r\n
	aTxBuffer2[2]=(sizeof(aTxBuffer1));//���ݳ���λ
	HAL_UART_Transmit(&huart1,aTxBuffer2, sizeof(aTxBuffer2),200);	
	HAL_UART_Transmit(&huart1,aTxBuffer1, sizeof(aTxBuffer1),200);
	//�����ѹ
	aTxBuffer2[4]=Frames.Voltage_Input;//���ݹ���λ
	sprintf(aTxBuffer1,"%.3lf\r\n",adv2_IN14_real);//\r\n
	aTxBuffer2[2]=(sizeof(aTxBuffer1));//���ݳ���λ
	HAL_UART_Transmit(&huart1,aTxBuffer2, sizeof(aTxBuffer2),200);	
	HAL_UART_Transmit(&huart1,aTxBuffer1, sizeof(aTxBuffer1),200);
	//�����ѹ
	aTxBuffer2[4]=Frames.Voltage_Output;//���ݹ���λ
	sprintf(aTxBuffer1,"%.3lf\r\n",adv2_IN2_real);//\r\n
	aTxBuffer2[2]=(sizeof(aTxBuffer1));//���ݳ���λ
	HAL_UART_Transmit(&huart1,aTxBuffer2, sizeof(aTxBuffer2),200);	
	HAL_UART_Transmit(&huart1,aTxBuffer1, sizeof(aTxBuffer1),200);
	//�������
	aTxBuffer2[4]=Frames.Current_Input;//���ݹ���λ
	sprintf(aTxBuffer1,"%.3lf\r\n",adv3_real);//\r\n
	aTxBuffer2[2]=(sizeof(aTxBuffer1));//���ݳ���λ
	HAL_UART_Transmit(&huart1,aTxBuffer2, sizeof(aTxBuffer2),200);	
	HAL_UART_Transmit(&huart1,aTxBuffer1, sizeof(aTxBuffer1),200);
	//���ݳ�����
	aTxBuffer2[4]=Frames.Current_Cap_Input;//���ݹ���λ
	sprintf(aTxBuffer1,"%.3lf\r\n",adv1_IN7_real);//\r\n
	aTxBuffer2[2]=(sizeof(aTxBuffer1));//���ݳ���λ
	HAL_UART_Transmit(&huart1,aTxBuffer2, sizeof(aTxBuffer2),200);	
	HAL_UART_Transmit(&huart1,aTxBuffer1, sizeof(aTxBuffer1),200);
	//���ݳ���ѹ
	aTxBuffer2[4]=Frames.Voltage_Cap_Input;//���ݹ���λ
	sprintf(aTxBuffer1,"%.3lf\r\n",adv1_IN6_real);//\r\n
	aTxBuffer2[2]=(sizeof(aTxBuffer1));//���ݳ���λ
	HAL_UART_Transmit(&huart1,aTxBuffer2, sizeof(aTxBuffer2),200);	
	HAL_UART_Transmit(&huart1,aTxBuffer1, sizeof(aTxBuffer1),200);

}