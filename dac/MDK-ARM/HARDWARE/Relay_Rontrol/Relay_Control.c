#include "delay.h"
#include "Relay_Control.h"

extern u8 RelayContro_flag;//�̵���״̬��־��1Ϊ��ͨ��0Ϊ�Ͽ�
extern u8 RelayContro_delay_flag;//��ʱ����ͨ��ʱ��ʱ��־
extern u8 Cap_Hreshold_Voltage_mode;//����������ֵ����ģʽ��־
void Relay_Control(u8 state)
{
	if(state==1)//�̵�����ͨ(˳���ر�ע��)
	{
		HAL_GPIO_WritePin(RelayControl0_GPIO_Port, RelayControl0_Pin, GPIO_PIN_SET);//f0
		delay_ms(10);
		HAL_GPIO_WritePin(RelayControl1_GPIO_Port, RelayControl1_Pin, GPIO_PIN_SET);//f1
		delay_ms(10);
		HAL_GPIO_WritePin(RelayControl2_GPIO_Port, RelayControl2_Pin, GPIO_PIN_SET);//f2
		RelayContro_flag=1;
		Cap_Hreshold_Voltage_mode=0;//��ͨ�̵�������ձ�־��������ѹģʽ
		
	}
	else if(state==0)//�̵����ض�(˳���ر�ע��)
	{
		HAL_GPIO_WritePin(RelayControl2_GPIO_Port, RelayControl2_Pin, GPIO_PIN_RESET);//f2
		HAL_GPIO_WritePin(RelayControl1_GPIO_Port, RelayControl1_Pin, GPIO_PIN_RESET);//f1
		HAL_GPIO_WritePin(RelayControl0_GPIO_Port, RelayControl0_Pin, GPIO_PIN_RESET);//f0
		RelayContro_flag=0;
		RelayContro_delay_flag=0;
	
	}
	else////�̵����ض�(˳���ر�ע��)
	{
		HAL_GPIO_WritePin(RelayControl2_GPIO_Port, RelayControl2_Pin, GPIO_PIN_RESET);//f2
		HAL_GPIO_WritePin(RelayControl1_GPIO_Port, RelayControl1_Pin, GPIO_PIN_RESET);//f1
		HAL_GPIO_WritePin(RelayControl0_GPIO_Port, RelayControl0_Pin, GPIO_PIN_RESET);//f0
		RelayContro_flag=0;
		RelayContro_delay_flag=0;
	}


}