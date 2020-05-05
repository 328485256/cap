#include "delay.h"
#include "EXTI_Callback.h"
#include "lcd.h"
#include "Relay_Control.h"
#include "Data_Processing.h"
#include "dac.h"
extern int flage_display;//0为不显示
extern u8 RelayContro_flag;
extern int DAC_Value;
extern float V;
extern PID pid;
extern PID pid_power_cap;
extern uint8_t usart_flage;
void EXTI_CALLBACK_key1()//key2
{
	delay_ms(50);
	if(HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_2 )==0)
	{
		if(RelayContro_flag==0)
		{
			Relay_Control(1);
		}

		else
		{
			Relay_Control(0);
		}
		}

}

void EXTI_CALLBACK_key2()//key1
{
	delay_ms(50);
	static int i=0;
	
	if(HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_3 )==0)
	{
		if( pid_power_cap.setpoint< 50)
		{
			pid_power_cap.setpoint=pid_power_cap.setpoint+5;
		}
		else 
		{
			pid_power_cap.setpoint=0;
		}
		
//		if( DAC_Value>200)
//		{
//			DAC_Value=DAC_Value-100;
//		}
//		else 
//		{
//			DAC_Value=4095;
//		}
		

	
		

	}


}
void EXTI_CALLBACK_key3()//key0
{
	delay_ms(50);
	//LCD_Clear(WHITE);//按一下按键清一次，一直清刷新太慢
	if(HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_4 )==0)
	{
		HAL_GPIO_WritePin(GPIOF, GPIO_PIN_10, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOF, GPIO_PIN_9, GPIO_PIN_RESET);
		if(usart_flage==0)//如果串口为关闭状态，打开串口
		{
			//关闭显示
			//LCD_Fill(0,0,480,800,BLACK);				//填充指定颜色
			LCD_SSD_BackLightSet(1);
			LCD_DisplayOff();
			//打开串口
			usart_flage=1;
		}
		else
		{
			//打开显示
			LCD_DisplayOn();
			//LCD_Fill(0,0,480,800,WHITE);
			LCD_SSD_BackLightSet(99);
			//关闭串口
			usart_flage=0;
		}
		delay_ms(500);
		HAL_GPIO_WritePin(GPIOF, GPIO_PIN_10, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOF, GPIO_PIN_9, GPIO_PIN_SET);
		
	}


}
void EXTI_CALLBACK_key4()
{

}



















