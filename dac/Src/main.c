/* USER CODE BEGIN Header */
/**
	******************************************************************************
	* @file           : main.c
	* @brief          : Main program body
	******************************************************************************
	* @attention
	*
	* <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
	* All rights reserved.</center></h2>
	*
	* This software component is licensed by ST under BSD 3-Clause license,
	* the "License"; You may not use this file except in compliance with the
	* License. You may obtain a copy of the License at:
	*                        opensource.org/licenses/BSD-3-Clause
	*
	******************************************************************************
	*/
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dac.h"
#include "dma.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "fsmc.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#define CSB1_Pin GPIO_PIN_0
#define CSB1_GPIO_Port GPIOC
#define CSB2_Pin GPIO_PIN_1
#define CSB2_GPIO_Port GPIOC

#include <stdio.h>
#include "string.h"
#include "stdint.h"
#include "delay.h"
#include "lcd.h"
#include "ad.h"
#include "EXTI_Callback.h"
#include "Data_Processing.h"
#include "Relay_Control.h"
#include "Voltage_Calibration.h"
#include "display.h"
#include "UartData.h"
#include "Self_Inspection.h"
//#include "bmi08x_defs.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int x; 
//电相关变量
/*
创建ADC格式：
							__IO uint16_t uhADCxConvertedValue_ADCX[10] = {0};   //DMA buffer
							uint16_t out_ADCxConvertedValue_PXX_INX=0;           //数据处理结果存放buffer
							double advX_real=0;                                  //计算后的真实值
							static float prevDataX=0;                            //卡尔曼滤波buffer
*/
#define Cap_Hreshold_Voltage 10.5  //阈值电压
#define power_sys 2 //mcu和lcd屏等产生的功耗
#define bias_ADC 0 //ADC补偿值，ADC接地时输出不是0，要加偏置
#define Voltag_out 20  //默认输出电压
#define Voltag_out1_min 25
__IO uint16_t uhADCxConvertedValue_ADC1[10] = {0};//改成uint32_t，数据会溢出
__IO uint16_t uhADCxConvertedValue_PA6_IN6[10] = {0};//改成uint32_t，数据会溢出-----超级电容电压
__IO uint16_t uhADCxConvertedValue_PA7_IN7[10] = {0};//改成uint32_t，数据会溢出-----超级电容电流
__IO uint16_t uhADCxConvertedValue_ADC2[10] = {0};//adc通道2的数据
__IO uint16_t uhADCxConvertedValue_PC4_IN14[10] = {0};//输入电压
__IO uint16_t uhADCxConvertedValue_PA2_IN2[10] = {0};//输出电压
__IO uint16_t uhADCxConvertedValue_PA0[10] = {0};//输入电流
uint16_t out_ADCxConvertedValue_PA6=0;
uint16_t out_ADCxConvertedValue_PA6_IN6=0;//数据处理结果存放
uint16_t out_ADCxConvertedValue_PA7_IN7=0;//数据处理结果存放
uint16_t out_ADCxConvertedValue_PC4_IN14=0;//数据处理结果存放
uint16_t out_ADCxConvertedValue_PA2_IN2=0;//数据处理结果存放
uint16_t out_ADCxConvertedValue_PA0=0;//数据处理结果存放
uint16_t DMA_CNT=0;//DMA计数
double adv1_real=0;
double adv1_IN6_real=0;//超级电容电压，按照比例放大之后的值
double adv1_IN7_real=0;//超级电容电流，按照比例放大之后的值
double adv2_IN2_real=0;//输出电压
double adv2_IN14_real=0;//输入电压，按照比例放大之后的值
double adv3_real=0;//输入电流，按照比例放大之后的值
uint32_t DAC_Value=4095;//920;超级电容充电，初始输出0V----------------------------------------------------------------------------------------------------
double power_input=0;//输入功率
double power_cap=0;//超级电容充电功率
double power_output=0;//输出功率
uint16_t power_set=60;

//输出标记
int flage=0;//功率超过额定功率（flage=1）功率没有超过额定功率（flage=0）
int Self_Inspecton_flage = 0;//0为没有通过自检，1为通过自检


//屏幕显示相关变量
uint16_t flage_display=2;//0为清空屏幕，1为显示数据，3为暂停
uint16_t Waveform_Date1[400]={0};//波形缓存buff
uint16_t Waveform_Date2[400]={0};//波形缓存buff
//继电器相关变量
u8 RelayContro_flag=0;//继电器是否工作，工作为1，不工作为0
u8 RelayContro_delay_flag=0;//当继电器工作时开始计时，计时100ms，再检测输出电压情况，1为延时完成，0为没有完成

//串口相关变量
uint8_t aTxBuffer[] = "*********SENDING DATA USING USART1 with DMA***********\r\n";
volatile uint8_t rx_len=0;                     //接收数据长度
volatile uint8_t recv_end_flag=0;              //接收完成标记位
uint8_t aRxBuffer1[100];      //接收缓存
uint8_t aTxBuffer1[10];			//发送data缓存
uint8_t aTxBuffer2[5];			//发送字头缓存
char  BUFFER_SIZE=100;      //不定长数据的最大长度，设置为100
uint8_t usart_flage=0;      //是否向上位机传输数据
frames Frames;//数据帧


//卡尔曼滤波buffer
static float prevData1=0; 
static float prevData2=0; 
static float prevData3=0; 
static float prevData4=0;
static float prevData5=0;
static float prevData6=0;

//PID相关
volatile PID pid;//恒流
volatile PID pid_power_cap;//恒功率
volatile PID pid_power_out;//恒功率
float V=0;//中断控制电压，调试时用，一般情况没有用
u8 Cap_Hreshold_Voltage_mode=0;//1为超过电压阈值，进入恒压模式。

//pringf 重定向
int fputc(int c, FILE *stream)    //重写fputc函数
{ /*    huart1是工具生成代码定义的UART1结构体，    如果以后要使用其他串口打印，只需要把这个结构体改成其他UART结构体。*/  
	HAL_UART_Transmit(&huart1, (unsigned char *)&c, 1, 1000);       
	return 1;
}


/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	//字符输出缓存数组
	u8 adc_vale1[12]={0}; //要设成数组，太小可能会溢出
	u8 adc_vale2[12]={0};
	u8 adc_vale3[12]={0};
	u8 BUFF[12]={0};
  /* USER CODE END 1 */
  

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_FSMC_Init();
  MX_ADC1_Init();
  MX_DAC_Init();
  MX_ADC3_Init();
  MX_USART1_UART_Init();
  MX_SPI2_Init();
  MX_ADC2_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
	
	PID_int(&pid);//初始化恒流pid
	PID_power_cap_init(&pid_power_cap);//初始化电容充电恒功率pid
  PID_power_out_init(&pid_power_out);//初始化输出恒功率pid
	Frames_init(&Frames);//帧头初始化
	aTxBuffer2[0]=Frames.Frame_Header1;//帧头1
	aTxBuffer2[1]=Frames.Frame_Header2;//帧头2
	aTxBuffer2[3]=Frames.Frame_Verify;//校验位

	
	__HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);//使能idle中断
	HAL_UART_Receive_DMA(&huart1,aRxBuffer1,BUFFER_SIZE);//打开DMA接收，数据存入aRxBuffer1数组中。	
	HAL_UART_Transmit_DMA(&huart1,aTxBuffer,sizeof(aTxBuffer));// DMA发生数据
	delay_init(168);
	LCD_Init();
	
	LCD_ShowString(30,30,200,16,16, "-----------------");	
	
 
	//BMI088--------------------------------------------------------------------------------------------------------------
	//屏幕显示
	//adc可用性检测
	HAL_ADC_Start_DMA(&hadc1, (uint32_t*)&uhADCxConvertedValue_ADC1,10);
	POINT_COLOR=BLUE ;
	HAL_ADC_Stop_DMA(&hadc1);
	LCD_ShowString(30,30,200,16,16, "uhADCxConvertedValu0");	
	HAL_ADC_Start_DMA(&hadc2, (uint32_t*)&uhADCxConvertedValue_ADC2,10);
	POINT_COLOR=BLUE ;	
	LCD_ShowString(30,30,200,16,16, "uhADCxConvertedValu01");	
	HAL_ADC_Stop_DMA(&hadc2);
	HAL_ADC_Start_DMA(&hadc3, (uint32_t*)&uhADCxConvertedValue_PA0,10);
	POINT_COLOR=BLUE ;
	LCD_ShowString(30,30,200,16,16, "uhADCxConvertedValu012");	
	HAL_ADC_Stop_DMA(&hadc3);
	//adc检测完毕
	//意思是开启dma传输,传送一个字的数据到uhADCxConvertedValue这个变量里面
	//保证继电器是关闭的
	HAL_GPIO_WritePin(RelayControl0_GPIO_Port, RelayControl0_Pin, GPIO_PIN_RESET);//f0
	HAL_GPIO_WritePin(RelayControl0_GPIO_Port, RelayControl1_Pin, GPIO_PIN_RESET);//f1
	HAL_GPIO_WritePin(RelayControl0_GPIO_Port, RelayControl2_Pin, GPIO_PIN_RESET);//f2
	//
	HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, Uout_Convert(0));//PA4
	HAL_DAC_Start(&hdac, DAC_CHANNEL_1);
	delay_ms(1000);
	
	//24v接通，mos管打开
	HAL_GPIO_WritePin(GPIOF, GPIO_PIN_3, GPIO_PIN_SET);//24v接通
	//定时器要最后开，串口和主程序都在定时器里
	HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, Uout_Convert(0));//超级电容输出电压
	HAL_DAC_Start(&hdac, DAC_CHANNEL_1);
	//最终输出电压
	HAL_DAC_SetValue(&hdac, DAC_CHANNEL_2, DAC_ALIGN_12B_R, Uout_Convert(0));//模块输出电压
	HAL_DAC_Start(&hdac, DAC_CHANNEL_2);
	//
	HAL_TIM_Base_Start_IT(&htim2);//开启定时器在屏幕初始化之后，不然可能出问题，具体原因不明，可能和延时有关
	HAL_TIM_Base_Start_IT(&htim3);//开启定时器在屏幕初始化之后，不然可能出问题，具体原因不明，可能和延时有关
	Self_Inspection();


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1)
	{
		
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  if(Self_Inspecton_flage==0)//没有完成自检就不进行控制
	  {
		  break;
  	}
		display();
		//显示数据波形
		data_display_adc(uhADCxConvertedValue_PA6_IN6,10);

		

//		//运行状态灯关闭
////		HAL_GPIO_WritePin(GPIOF, GPIO_PIN_10, GPIO_PIN_RESET);//f10
////		HAL_GPIO_WritePin(GPIOF, GPIO_PIN_9, GPIO_PIN_RESET);//f9
//		

//			//HAL_Delay(100);
		
	}
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage 
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
//主程序，放在定时器里跑让执行周期确定。
void Control_Program()
{
//adc读取
	//ADC1，双通道
	for(int i=0;i<10;i=i+2)
	{
		HAL_ADC_Start_DMA(&hadc1, (uint32_t*)&uhADCxConvertedValue_ADC1,4);//意思是开启dma传输,传送一个字的数据到uhADCxConvertedValue这个变量里面 
		delay_us(50);//不延时adc数据不会更新,延时对数据结果有影响太小数据就有便宜，接地时有1.0v拉高时偶2.v,推测和采样次数有关
		HAL_ADC_Stop_DMA(&hadc1);
		uhADCxConvertedValue_PA6_IN6[i]=uhADCxConvertedValue_ADC1[0];
		uhADCxConvertedValue_PA7_IN7[i]=uhADCxConvertedValue_ADC1[1];
		uhADCxConvertedValue_PA6_IN6[i+1]=uhADCxConvertedValue_ADC1[2];
		uhADCxConvertedValue_PA7_IN7[i+1]=uhADCxConvertedValue_ADC1[3];
	}
	//ADC2
	for(int i=0;i<10;i=i+2)
	{
		HAL_ADC_Start_DMA(&hadc2, (uint32_t*)&uhADCxConvertedValue_ADC2,4);//意思是开启dma传输,传送一个字的数据到uhADCxConvertedValue这个变量里面 
		delay_us(50);//不延时adc数据不会更新,延时对数据结果有影响太小数据就有便宜，接地时有1.0v拉高时偶2.v,推测和采样次数有关
		HAL_ADC_Stop_DMA(&hadc1);
		uhADCxConvertedValue_PA2_IN2[i] =uhADCxConvertedValue_ADC2[0];
		uhADCxConvertedValue_PC4_IN14[i]=uhADCxConvertedValue_ADC2[1];
		uhADCxConvertedValue_PA2_IN2[i+1] =uhADCxConvertedValue_ADC2[2];
		uhADCxConvertedValue_PC4_IN14[i+1]=uhADCxConvertedValue_ADC2[3];
	}

	delay_us(50);//不延时adc数据不会更新,延时对数据结果有影响太小数据就有便宜，接地时有1.0v拉高时偶2.v,推测和采样次数有关
	HAL_ADC_Stop_DMA(&hadc2);
	//ADC3
	HAL_ADC_Start_DMA(&hadc3, (uint32_t*)&uhADCxConvertedValue_PA0,10);
	delay_us(50);//不延时adc数据不会更新,延时对数据结果有影响太小数据就有便宜，接地时有1.0v拉高时偶2.v,推测和采样次数有关
	HAL_ADC_Stop_DMA(&hadc3);
	//
	//数据处理(adc输出值取10次平均)
	//粗大误差剔除,
	out_ADCxConvertedValue_PA6_IN6=AGV_Remove_the_Ross_Error(uhADCxConvertedValue_PA6_IN6,10);//超级电容电压
	out_ADCxConvertedValue_PA7_IN7=AGV_Remove_the_Ross_Error(uhADCxConvertedValue_PA7_IN7,10);//超级电容电流
	out_ADCxConvertedValue_PA0=AGV_Remove_the_Ross_Error(uhADCxConvertedValue_PA0,10);//输入电流
	//out_ADCxConvertedValue_PA2=AGV_Remove_the_Ross_Error(uhADCxConvertedValue_PA2_IN2,10);//输出电压
	out_ADCxConvertedValue_PA2_IN2=AGV_Remove_the_Ross_Error(uhADCxConvertedValue_PA2_IN2,10);//输出电压
	out_ADCxConvertedValue_PC4_IN14=AGV_Remove_the_Ross_Error(uhADCxConvertedValue_PC4_IN14,10);//输出电压
	
	//卡尔曼滤波
	out_ADCxConvertedValue_PA6_IN6=kalmanFilter_A(out_ADCxConvertedValue_PA6_IN6,&prevData1);
	out_ADCxConvertedValue_PA7_IN7=kalmanFilter_A(out_ADCxConvertedValue_PA7_IN7,&prevData4);
	out_ADCxConvertedValue_PA0=kalmanFilter_A(out_ADCxConvertedValue_PA0,&prevData2);
	//out_ADCxConvertedValue_PA2=kalmanFilter_A(out_ADCxConvertedValue_PA2,&prevData3);
	out_ADCxConvertedValue_PA2_IN2=kalmanFilter_A(out_ADCxConvertedValue_PA2_IN2,&prevData3);
	out_ADCxConvertedValue_PC4_IN14=kalmanFilter_A(out_ADCxConvertedValue_PC4_IN14,&prevData6);
	

	//比例放大还原真实值
	adv1_IN6_real=out_ADCxConvertedValue_PA6_IN6*0.00080566*10.8;//0.00080566为分辨率，11为电阻分压后的放大倍数//超级电容电压
	adv1_IN7_real=(out_ADCxConvertedValue_PA7_IN7-bias_ADC)*0.00080566*7.094*1.27 ;//超级电容电流----0.00080566为分辨率，*1.066更加接近真实电流，但是为了防止电流过大，特意将放大倍数向上取整
	adv3_real=(out_ADCxConvertedValue_PA0-bias_ADC)*0.00080566*7.094*1.19;//*0.00080566*7.094/2;//输入电流
	adv3_real=kalmanFilter_A(adv3_real,&prevData5);//卡尔曼滤波,第一次滤波效果不是太好
	adv2_IN2_real=out_ADCxConvertedValue_PA2_IN2*0.00080566*10.8;//输出电压
	adv2_IN14_real=out_ADCxConvertedValue_PC4_IN14*0.00080566*10.8;//输入电压
	
	//!!!!!!!因为采样电阻等原因，电流只要在大于0.16时电流计返回的值才是准的，具体情况具体分析
	
	//功率计算
	power_input=adv2_IN14_real*adv3_real;//输入功率 电压*电流
	power_cap=adv1_IN6_real*adv1_IN7_real;//超级电容充电功率，电压*电流
	power_output=power_input-power_cap-power_sys;//输出功率
	
	
	//Automatic_Voltage_Acquisition();自动获取每个DAC输出时对应电压的值，一共可以跑4096次，用串口输出结果 ， 串口115200
  if(Self_Inspecton_flage==0)//没有完成自检就不进行控制
	{
		return ;
	}
	//PID控制
	//功率检测
	if(power_input>=power_set+2)
	{
		flage=1;
		HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, Uout_Convert(0));//电容不冲电
		PID_power_cap_init(&pid_power_cap);//初始化恒功率pid <-不加这个初始化，pid_power_cap.result的值会一直保留，等下次超级电容要充电时应发超调
		pid_power_out.setpoint = power_set;
	
	}
	else if(power_input<power_set-5)
	{
		flage=0;
		PID_power_out_init(&pid_power_out);//初始化恒功率pid <-不加这个初始化，pid_power_cap.result的值会一直保留，等下次超级电容要充电时应发超调
		//还原最终输出电压
		HAL_DAC_SetValue(&hdac, DAC_CHANNEL_2, DAC_ALIGN_12B_R, Uout_Convert(Voltag_out));//PA5
		HAL_DAC_Start(&hdac, DAC_CHANNEL_2);
	}
	//功率超过额定功率（flage=1）
	if(flage==1)
	{
		if(pid_power_out.result>=0)
		{
			PIDRegulation(&pid_power_out, power_input);//pid 控制 计算 ，这里放的是你要维持的功率，可以用power_input/power_output
			HAL_DAC_SetValue(&hdac, DAC_CHANNEL_2, DAC_ALIGN_12B_R, Uout_Convert(pid_power_out.result));//PA5
		}
		else if(pid_power_out.result<0||pid_power_out.result>50)
		{
			HAL_DAC_SetValue(&hdac, DAC_CHANNEL_2, DAC_ALIGN_12B_R, Uout_Convert(0));//PA5
		}
	  
		HAL_DAC_Start(&hdac, DAC_CHANNEL_2);
	}
	//功率小于等于额定功率（flage=0）
	if(flage==0&&RelayContro_flag==0)//在继电器不接通时
	{
		//HAL_DAC_SetValue(&hdac, DAC_CHANNEL_2, DAC_ALIGN_12B_R, 4095);//PA5
		
		if(adv1_IN7_real>=9.0)//电流大于9A，关闭pid
		{
			HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, Uout_Convert(pid_power_cap.result-1));//PA4
			HAL_GPIO_WritePin(GPIOF, GPIO_PIN_10, GPIO_PIN_RESET);//f10
			PID_power_cap_init(&pid_power_cap);//初始化恒功率pid
		}
		else if((adv1_IN6_real>Cap_Hreshold_Voltage) && (Cap_Hreshold_Voltage_mode==0))//超级电容充电电压大于阈值电压，关闭pid
		{
			//HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, Uout_Convert(Cap_Hreshold_Voltage+0.2));//PA4,加0.2是为了维持这个过电压的状态不跳变
			HAL_GPIO_WritePin(GPIOF, GPIO_PIN_9, GPIO_PIN_RESET);
			Cap_Hreshold_Voltage_mode=1;
			PID_power_cap_init(&pid_power_cap);//初始化恒功率pid <-不加这个初始化，pid_power_cap.result的值会一直保留，等下次超级电容要充电时应发超调
		}
		else if(pid_power_cap.result<0.0||pid_power_cap.result>100)//PID输出小于0   
		{
			HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, Uout_Convert(0));//PA4
			PID_power_cap_init(&pid_power_cap);//初始化恒功率pid
		}
		else  //正常充电
		{
			if(Cap_Hreshold_Voltage_mode==0)//恒功率模式
			{
				pid_power_cap.setpoint=(int16_t)(power_set-power_sys-power_output);
				if(pid_power_cap.setpoint<0)//防止阈值小于0；
				{
					pid_power_cap.setpoint=0;
				}
				PIDRegulation(&pid_power_cap, power_cap);//pid 控制 计算
				HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, Uout_Convert(pid_power_cap.result));//PA4
				HAL_GPIO_WritePin(GPIOF, GPIO_PIN_10, GPIO_PIN_SET);//f10
				HAL_GPIO_WritePin(GPIOF, GPIO_PIN_9, GPIO_PIN_SET);//f9
			}
			else if(Cap_Hreshold_Voltage_mode==1)//恒压模式
			{
				HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, Uout_Convert(Cap_Hreshold_Voltage));
			}

		}
		
		//HAL_DAC_Start(&hdac, DAC_CHANNEL_2);
		HAL_DAC_Start(&hdac, DAC_CHANNEL_1);
		flage=0;
	}
	else if(flage==0&&RelayContro_flag==1)//继电器导通，超级电容向外输电时，充电的dcdc不工作保持输出0V电压，以免继电器断开（下次充电）时发生未知问题
	{
		HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, Uout_Convert(0));//PA4
		HAL_DAC_Start(&hdac, DAC_CHANNEL_1);
	}
//	HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, Uout_Convert(13));//PA4
//	HAL_DAC_Start(&hdac, DAC_CHANNEL_1);
	//继电器控制
	if(RelayContro_delay_flag==1 && adv2_IN2_real<=Voltag_out1_min)//等待延时100ms后在做电压检测
	{
		Relay_Control(0);
	}
	//串口数据发送



	//运行状态灯关闭
//		HAL_GPIO_WritePin(GPIOF, GPIO_PIN_10, GPIO_PIN_RESET);//f10
//		HAL_GPIO_WritePin(GPIOF, GPIO_PIN_9, GPIO_PIN_RESET);//f9
	

		//HAL_Delay(100);
}
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	/* Turn LED1 on: Transfer process is correct */
 // BSP_LED_On(LED1);
	//HAL_GPIO_WritePin (GPIOF,GPIO_PIN_6,GPIO_PIN_SET );
	 /*if(hadc==(&hadc1))

	{

		DMA_CNT++;

	}  */  
}
void data_display_adc(uint16_t* ADCxConvertedValue,int Length)//显示数据
{
	char adc_vale[50]={0};//可能会溢出
	uint16_t ADCxConvertedValue_copy[Length];
	uint16_t avg=0;
	
	double var=0;
	int difference_value=0;
	int i=0;

	if(flage_display==1)//显示数据细节
	{

		LCD_Draw_Output_waveform(400,400,30,70,BLUE,out_ADCxConvertedValue_PA0,Waveform_Date1);//370,760
		
	
	}
	else if(flage_display==0)//清空屏幕
	{
		LCD_Clear(WHITE);//按一下按键清一次，一直清刷新太慢
		flage_display=3;//防止不停的清空屏幕
	}
	else if(flage_display==2)//显示波形
	{
	//	LCD_Clear(WHITE);uint16_t
		LCD_Draw_Output_waveform(400,400,30,70,BLUE,(uint16_t)power_cap*40,Waveform_Date1);//电容输入功率
		LCD_Draw_Output_waveform(400,770,30,440,BLUE,out_ADCxConvertedValue_PA7_IN7,Waveform_Date2);//电容电流
	}
	else if(flage_display==3)//暂停
	{
		
	}
}
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)  
{  
  /* NOTE: This function Should not be modified, when the callback is needed, 
           the HAL_GPIO_EXTI_Callback could be implemented in the user file 
   */  
    switch(GPIO_Pin)  
    {  
        case GPIO_PIN_2:EXTI_CALLBACK_key1();break;  
        case GPIO_PIN_3:EXTI_CALLBACK_key2();break;  
        case GPIO_PIN_4:EXTI_CALLBACK_key3();break;  
        case GPIO_PIN_15:EXTI_CALLBACK_key4();break;  
        default:break;  
    }  
} 
void usart_feedback()
{
	char str[4];
	//u16 num_temp=0;

//..................................................................
	if(recv_end_flag ==1)
	{
		//printf("rx_len=%d\r\n",rx_len);//打印接收长度
		HAL_UART_Transmit(&huart1,aRxBuffer1, rx_len,100);//接收数据打印出来
		HAL_UART_Transmit(&huart1,"\r\n", sizeof("\r\n"),100);//接收数据打印出来
//  		HAL_UART_Transmit_DMA(&huart1,aRxBuffer1, rx_len);//接收数据打印出来
//  		HAL_UART_Transmit_DMA(&huart1,"\r\n", sizeof("\r\n"));//接收数据打印出来
		if(memcmp(aRxBuffer1,"waveform",strlen("waveform"))==0)
		{
			flage_display=2;
			LCD_Clear(WHITE);
		}
		else if(memcmp(aRxBuffer1,"date",strlen("date"))==0)
		{
			flage_display=1;
			LCD_Clear(WHITE);
		}
		else if(memcmp(aRxBuffer1,"clear",strlen("clear"))==0)
		{
			flage_display=0;
		}
		else if(memcmp(aRxBuffer1,"power_set",strlen("power_set"))==0)//从上位机获取指令
		{
			x=strlen(aRxBuffer1)-strlen("power_set");
			for(int i=0;i<(strlen(aRxBuffer1)-strlen("power_set"));i++)
			{
				str[i]=aRxBuffer1[strlen("power_set")+i];
			}
			if(atoi(str) <= (power_set+30))//power_set的值不能变化太大，太大的话系统就会不稳定，所有两次差值控制在30
			{
				power_set = atoi(str);
			}
			else if(atoi(str) > (power_set+30))//两次差值超过30，新的值为power_set+30，并让上位机重新传数据，
			{
				power_set = power_set+30;
				aTxBuffer2[4]=Frames.power_input_set_error;//数据功能位，告诉上位机power_set的数据从新传一次
				sprintf(aTxBuffer1,"%.3lf\r\n",0);//\r\n
				aTxBuffer2[2]=(sizeof(aTxBuffer1));//数据长度位
				HAL_UART_Transmit(&huart1,aTxBuffer2, sizeof(aTxBuffer2),100);	
				HAL_UART_Transmit(&huart1,aTxBuffer1, sizeof(aTxBuffer1),100);
			}
			
			for(int i=0;i<sizeof(str);i++)
			{
				str[i]=0;
			}
			
		}
		for(uint8_t i=0;i<rx_len;i++)
		{
			aRxBuffer1[i]=0;//清接收缓存
		}
		rx_len=0;//清除计数
		recv_end_flag=0;//清除接收结束标志位
	}
	HAL_UART_Receive_DMA(&huart1,aRxBuffer1,BUFFER_SIZE);// 启动DMA接收
	//..................................................................
}
//定时器中断
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	static u8 flage_temp=0;
	static u8 flage_temp2=0;
	static u32 time=0;
	//定时器2
  if(htim==(&htim2))
  {
		//超级电容停止充电后，串入输出电源，
		if(RelayContro_flag==1)//继电器工作,开始计时 
		{
			if(time >= 10)//完成计时
			{
				RelayContro_delay_flag=1;
				HAL_GPIO_WritePin(GPIOF, GPIO_PIN_10, GPIO_PIN_RESET);//状态灯亮
				time=0;
			}
			else if(time <10)//没有完成计时
			{
				time=time+1;
			}
		}
		else if(RelayContro_flag==0)//继电器没有工作
		{
			HAL_GPIO_WritePin(GPIOF, GPIO_PIN_10, GPIO_PIN_SET);//状态灯灭
		}
		if (usart_flage==1)
		{

			UartSend();
			
		}
		else if(usart_flage==0)
		{
		}
		usart_feedback();//串口接收后反馈信号的函数，可以控制显示内容，显示波形pc发送：waveform，显示数据pc发送：date，清空显示pc发送：clear

    

  }
	//定时器3，
	if(htim==(&htim3))
	{
		
		
		Control_Program();//主要控制函数

		
	}
}
void Frames_init(frames *vframes)
{
	vframes->Frame_Header1=0xAA ;
	vframes->Frame_Header2=0x0A ;
	vframes->Frame_Len =0;
	vframes->Frame_Verify =0x01;
	//功能位
	vframes->Voltage_Input = 0x11;//为了区别校验位所有从0x11开始
	vframes->Current_Input = 0x12;
	vframes->Voltage_Output = 0x13;
	vframes->Current_Output = 0x14;
	vframes->Voltage_Cap_Input = 0x15;
	vframes->Current_Cap_Input = 0x16;
	vframes->Voltage_Cap_Output = 0x17;
	vframes->power_input = 0x18;//输入功率
	vframes->power_cap = 0x19;//超级电容充电功率
	vframes->power_output = 0x1A;//输出功率
	vframes->power_input_set_error = 0xFF;//输出功率

}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
	/* User can add his own implementation to report the file name and line number,
		 tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
