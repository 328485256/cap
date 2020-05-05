#include "delay.h"
#include "Relay_Control.h"

extern u8 RelayContro_flag;//继电器状态标志，1为接通，0为断开
extern u8 RelayContro_delay_flag;//定时器接通计时延时标志
extern u8 Cap_Hreshold_Voltage_mode;//超级电容阈值超出模式标志
void Relay_Control(u8 state)
{
	if(state==1)//继电器接通(顺序特别注意)
	{
		HAL_GPIO_WritePin(RelayControl0_GPIO_Port, RelayControl0_Pin, GPIO_PIN_SET);//f0
		delay_ms(10);
		HAL_GPIO_WritePin(RelayControl1_GPIO_Port, RelayControl1_Pin, GPIO_PIN_SET);//f1
		delay_ms(10);
		HAL_GPIO_WritePin(RelayControl2_GPIO_Port, RelayControl2_Pin, GPIO_PIN_SET);//f2
		RelayContro_flag=1;
		Cap_Hreshold_Voltage_mode=0;//接通继电器后，清空标志，跳出恒压模式
		
	}
	else if(state==0)//继电器关断(顺序特别注意)
	{
		HAL_GPIO_WritePin(RelayControl2_GPIO_Port, RelayControl2_Pin, GPIO_PIN_RESET);//f2
		HAL_GPIO_WritePin(RelayControl1_GPIO_Port, RelayControl1_Pin, GPIO_PIN_RESET);//f1
		HAL_GPIO_WritePin(RelayControl0_GPIO_Port, RelayControl0_Pin, GPIO_PIN_RESET);//f0
		RelayContro_flag=0;
		RelayContro_delay_flag=0;
	
	}
	else////继电器关断(顺序特别注意)
	{
		HAL_GPIO_WritePin(RelayControl2_GPIO_Port, RelayControl2_Pin, GPIO_PIN_RESET);//f2
		HAL_GPIO_WritePin(RelayControl1_GPIO_Port, RelayControl1_Pin, GPIO_PIN_RESET);//f1
		HAL_GPIO_WritePin(RelayControl0_GPIO_Port, RelayControl0_Pin, GPIO_PIN_RESET);//f0
		RelayContro_flag=0;
		RelayContro_delay_flag=0;
	}


}