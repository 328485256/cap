#include "main.h"
#include "delay.h"

static int8_t  fac_us=0;							//us延时倍乘数			   
static int16_t fac_ms=0;							//ms延时倍乘数,在os下,代表每个节拍的ms数
void delay_init(int16_t SYSCLK)       //为什么用int8_t会出现数据错误
{	
	  SysTick->CTRL&=~(1<<2);				//SYSTICK使用外部时钟源	
    fac_us = SYSCLK/8;						//不论是否使用OS,fac_us都需要使用
    fac_ms = (int16_t)fac_us*1000;				//非OS下,代表每个ms需要的systick时钟数   

 
}

						    

void delay_ms(int16_t nms) 
 {
//  int32_t temp; 
//  SysTick->LOAD = fac_ms*nms; //重载值
//  SysTick->VAL=0X00;//,当前值清空计数器 
//  SysTick->CTRL=0X01;//使能，减到零是无动作，采用外部时钟源 
//  do 
//  { 
//       temp=SysTick->CTRL;//读取当前倒计数值 
//  }
//     while((temp&0x01)&&(!(temp&(1<<16))));//等待时间到达 
//     
//     SysTick->CTRL=0x00; //关闭计数器 
//     SysTick->VAL =0X00; //清空计数器 
	 for(int i=0;i<nms;i++)
	 {
		 delay_us((u32)(nms));
	 }
	 
 }
 
 //延时nus
//nus为要延时的us数.	
//注意:nus的值,不要大于798915us(最大值即2^24/fac_us@fac_us=21)
void delay_us(int16_t nus)
{		
	int32_t temp;	   //临时变量 	 
	SysTick->LOAD=nus*fac_us; 				//时间加载	  		 
	SysTick->VAL=0x00;        				//清空计数器
	SysTick->CTRL=0X01 ; //开始倒数 	 
	do
	{
		temp=SysTick->CTRL;
	}while((temp&0x01)&&!(temp&(1<<16)));	//等待时间到达   
	SysTick->CTRL=0x00; //关闭计数器
	SysTick->VAL =0X00;       				//清空计数器 
}

 /************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
 