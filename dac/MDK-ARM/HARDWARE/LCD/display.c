#include "lcd.h"
#include "Data_Processing.h"
#include <stdio.h>

u8 lcd_buffer1[30]={0}; //Ҫ������飬̫С���ܻ����
extern double adv1_IN6_real;
extern double adv3_real;
extern double power_cap;
extern double power_input;
extern double adv1_IN7_real;
extern double adv2_IN2_real;
extern double power_output;
extern double adv2_IN14_real;
extern volatile PID pid;
extern volatile PID pid_power_cap;
extern uint16_t uhADCxConvertedValue_PA6_IN6[10];
void display()
{
	POINT_COLOR=BLUE ;
	sprintf((char*)lcd_buffer1,"adc_pa6:%fV",(double)adv1_IN6_real);//�������ݵ�ѹ
	LCD_ShowString(30,50,200,24,24,lcd_buffer1);
	sprintf((char*)lcd_buffer1,"adc_pa0:%fV",(double)adv3_real);//�������
	LCD_ShowString(250,50,200,24,24,lcd_buffer1);
	sprintf((char*)lcd_buffer1,"pow_cap:%fW",(double)power_cap);//�������ݹ���
	LCD_ShowString(250,80,200,24,24,lcd_buffer1);
	sprintf((char*)lcd_buffer1,"pow_in:%fW",(double)power_input);//���빦��
	LCD_ShowString(250,110,200,24,24,lcd_buffer1);
	sprintf((char*)lcd_buffer1,"pow_out:%fW",(double)power_output);//�������
	LCD_ShowString(250,140,200,24,24,lcd_buffer1);
	sprintf((char*)lcd_buffer1,"adc_pa7:%fA",(double)adv1_IN7_real);//�������ݵ���
	LCD_ShowString(30,410,300,24,24,lcd_buffer1);
	sprintf((char*)lcd_buffer1,"setpoint:%3f      ",pid_power_cap.setpoint);//PIDĿ��ֵ
	LCD_ShowString(250,410,300,24,24,lcd_buffer1);
	sprintf((char*)lcd_buffer1,"result:%3f      ",pid_power_cap.result);//PID���
	LCD_ShowString(250,440,300,24,24,lcd_buffer1);
	sprintf((char*)lcd_buffer1,"adc_pa2:%fV",(double)adv2_IN2_real);//�����ѹ
	LCD_ShowString(40,440,300,24,24,lcd_buffer1);
	sprintf((char*)lcd_buffer1,"adc_pc4:%fV",(double)adv2_IN14_real);//�����ѹ
	LCD_ShowString(40,470,300,24,24,lcd_buffer1);
}