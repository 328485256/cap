#include "Self_Inspection.h"
#include "lcd.h"
#include "logo.h"
#include "dac.h"
#include "Voltage_Calibration.h"
#include "delay.h"
extern double adv1_IN6_real;//�������ݵ�ѹ
extern double adv1_IN7_real;//�������ݵ���
extern double adv3_real;//�������
extern double adv2_IN14_real;//�����ѹ
extern double adv2_IN2_real;//�����ѹ
extern double power_output;
extern double power_cap;
extern double power_input;
extern const unsigned char gImage_qq[];
extern int Self_Inspecton_flage  ;//0Ϊû��ͨ���Լ죬1Ϊͨ���Լ�


void Self_Inspection()
{
	//��ʾͼƬ
	int x,y;
	y=350;
	x=140;
	int Input_Voltage_flage=0;//0Ϊû�ü�⵽�����ѹ��1Ϊ��⵽�����ѹ
	int Cap_flage = 0 ;//0Ϊû��ͨ���Լ죬1Ϊͨ���Լ�
	int Current_flage =0 ;//0Ϊû��ͨ���Լ죬1Ϊͨ���Լ�
	
  LCD_Color_Fill(x+1,y+1,LOGO_W+x,LOGO_H+y,sky_animation_mask); //ָ���������ɫ�飨colorΪɫ�����飩
	LCD_ShowString(100,470,200,24,24, "Initializing......");
	while(Self_Inspecton_flage==0)
	{
			//��������ѹ�Ƿ�����
		if(adv2_IN14_real>22)
		{
			//0Ϊû��ͨ���Լ죬1Ϊͨ���Լ�
			if(Input_Voltage_flage==0)
			{
				LCD_ShowString(100,500,300,24,24, "Input_Voltage Get ");
			}
			
			Input_Voltage_flage = 1;
			
		}
		else 
		{
			Input_Voltage_flage = 0;
			LCD_ShowString(100,500,300,24,24, "Input_Voltage Not Found  ");
			
		}
		//��ⳬ�������Ƿ�װ
		HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, Uout_Convert(2));
		HAL_DAC_Start(&hdac, DAC_CHANNEL_1);
		delay_ms(1000);
		HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, 4095);
		HAL_DAC_Start(&hdac, DAC_CHANNEL_1);
		delay_ms(3000);
		if (adv1_IN6_real>1 )
		{
			Cap_flage=1;
			LCD_ShowString(100,500,300,24,24, "Cap Get                          ");
		}
		else
		{
			Cap_flage=0;
			LCD_ShowString(100,500,300,24,24, "Cap Not Found                   ");
			delay_ms(1000);
			
		}
		HAL_DAC_SetValue(&hdac, DAC_CHANNEL_2, DAC_ALIGN_12B_R, Uout_Convert(5));
		HAL_DAC_Start(&hdac, DAC_CHANNEL_2);
		delay_ms(100);
		if(adv3_real > 1)
		{
			HAL_DAC_SetValue(&hdac, DAC_CHANNEL_2, DAC_ALIGN_12B_R, Uout_Convert(0));
			HAL_DAC_Start(&hdac, DAC_CHANNEL_2);
			Current_flage =0;
			POINT_COLOR=RED ;
			LCD_ShowString(30,500,400,24,24, "Short Sircuit inspect and restart    ");
			POINT_COLOR=BLUE ;
			while(1);
		}
		else if (Cap_flage==1 && Input_Voltage_flage==1 )
		{
			HAL_DAC_SetValue(&hdac, DAC_CHANNEL_2, DAC_ALIGN_12B_R, Uout_Convert(0));
			HAL_DAC_Start(&hdac, DAC_CHANNEL_2);
			Current_flage =1;
			LCD_ShowString(100,500,300,24,24, "Current OK                    ");
		}
		if(Input_Voltage_flage==1 && Cap_flage==1 && Current_flage==1 )
		{
			POINT_COLOR=GREEN ;
			LCD_ShowString(100,500,300,24,24, "Self Inspection   OK                ");
			delay_ms(1000);
			POINT_COLOR=BLUE ;
			
			Self_Inspecton_flage=1;
		}
	}
	LCD_Clear(WHITE);

	
	
}