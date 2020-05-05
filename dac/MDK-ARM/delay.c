#include "main.h"
#include "delay.h"

static int8_t  fac_us=0;							//us��ʱ������			   
static int16_t fac_ms=0;							//ms��ʱ������,��os��,����ÿ�����ĵ�ms��
void delay_init(int16_t SYSCLK)       //Ϊʲô��int8_t��������ݴ���
{	
	  SysTick->CTRL&=~(1<<2);				//SYSTICKʹ���ⲿʱ��Դ	
    fac_us = SYSCLK/8;						//�����Ƿ�ʹ��OS,fac_us����Ҫʹ��
    fac_ms = (int16_t)fac_us*1000;				//��OS��,����ÿ��ms��Ҫ��systickʱ����   

 
}

						    

void delay_ms(int16_t nms) 
 {
//  int32_t temp; 
//  SysTick->LOAD = fac_ms*nms; //����ֵ
//  SysTick->VAL=0X00;//,��ǰֵ��ռ����� 
//  SysTick->CTRL=0X01;//ʹ�ܣ����������޶����������ⲿʱ��Դ 
//  do 
//  { 
//       temp=SysTick->CTRL;//��ȡ��ǰ������ֵ 
//  }
//     while((temp&0x01)&&(!(temp&(1<<16))));//�ȴ�ʱ�䵽�� 
//     
//     SysTick->CTRL=0x00; //�رռ����� 
//     SysTick->VAL =0X00; //��ռ����� 
	 for(int i=0;i<nms;i++)
	 {
		 delay_us((u32)(nms));
	 }
	 
 }
 
 //��ʱnus
//nusΪҪ��ʱ��us��.	
//ע��:nus��ֵ,��Ҫ����798915us(���ֵ��2^24/fac_us@fac_us=21)
void delay_us(int16_t nus)
{		
	int32_t temp;	   //��ʱ���� 	 
	SysTick->LOAD=nus*fac_us; 				//ʱ�����	  		 
	SysTick->VAL=0x00;        				//��ռ�����
	SysTick->CTRL=0X01 ; //��ʼ���� 	 
	do
	{
		temp=SysTick->CTRL;
	}while((temp&0x01)&&!(temp&(1<<16)));	//�ȴ�ʱ�䵽��   
	SysTick->CTRL=0x00; //�رռ�����
	SysTick->VAL =0X00;       				//��ռ����� 
}

 /************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
 