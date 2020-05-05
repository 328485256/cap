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
//����ر���
/*
����ADC��ʽ��
							__IO uint16_t uhADCxConvertedValue_ADCX[10] = {0};   //DMA buffer
							uint16_t out_ADCxConvertedValue_PXX_INX=0;           //���ݴ��������buffer
							double advX_real=0;                                  //��������ʵֵ
							static float prevDataX=0;                            //�������˲�buffer
*/
#define Cap_Hreshold_Voltage 10.5  //��ֵ��ѹ
#define power_sys 2 //mcu��lcd���Ȳ����Ĺ���
#define bias_ADC 0 //ADC����ֵ��ADC�ӵ�ʱ�������0��Ҫ��ƫ��
#define Voltag_out 20  //Ĭ�������ѹ
#define Voltag_out1_min 25
__IO uint16_t uhADCxConvertedValue_ADC1[10] = {0};//�ĳ�uint32_t�����ݻ����
__IO uint16_t uhADCxConvertedValue_PA6_IN6[10] = {0};//�ĳ�uint32_t�����ݻ����-----�������ݵ�ѹ
__IO uint16_t uhADCxConvertedValue_PA7_IN7[10] = {0};//�ĳ�uint32_t�����ݻ����-----�������ݵ���
__IO uint16_t uhADCxConvertedValue_ADC2[10] = {0};//adcͨ��2������
__IO uint16_t uhADCxConvertedValue_PC4_IN14[10] = {0};//�����ѹ
__IO uint16_t uhADCxConvertedValue_PA2_IN2[10] = {0};//�����ѹ
__IO uint16_t uhADCxConvertedValue_PA0[10] = {0};//�������
uint16_t out_ADCxConvertedValue_PA6=0;
uint16_t out_ADCxConvertedValue_PA6_IN6=0;//���ݴ��������
uint16_t out_ADCxConvertedValue_PA7_IN7=0;//���ݴ��������
uint16_t out_ADCxConvertedValue_PC4_IN14=0;//���ݴ��������
uint16_t out_ADCxConvertedValue_PA2_IN2=0;//���ݴ��������
uint16_t out_ADCxConvertedValue_PA0=0;//���ݴ��������
uint16_t DMA_CNT=0;//DMA����
double adv1_real=0;
double adv1_IN6_real=0;//�������ݵ�ѹ�����ձ����Ŵ�֮���ֵ
double adv1_IN7_real=0;//�������ݵ��������ձ����Ŵ�֮���ֵ
double adv2_IN2_real=0;//�����ѹ
double adv2_IN14_real=0;//�����ѹ�����ձ����Ŵ�֮���ֵ
double adv3_real=0;//������������ձ����Ŵ�֮���ֵ
uint32_t DAC_Value=4095;//920;�������ݳ�磬��ʼ���0V----------------------------------------------------------------------------------------------------
double power_input=0;//���빦��
double power_cap=0;//�������ݳ�繦��
double power_output=0;//�������
uint16_t power_set=60;

//������
int flage=0;//���ʳ�������ʣ�flage=1������û�г�������ʣ�flage=0��
int Self_Inspecton_flage = 0;//0Ϊû��ͨ���Լ죬1Ϊͨ���Լ�


//��Ļ��ʾ��ر���
uint16_t flage_display=2;//0Ϊ�����Ļ��1Ϊ��ʾ���ݣ�3Ϊ��ͣ
uint16_t Waveform_Date1[400]={0};//���λ���buff
uint16_t Waveform_Date2[400]={0};//���λ���buff
//�̵�����ر���
u8 RelayContro_flag=0;//�̵����Ƿ���������Ϊ1��������Ϊ0
u8 RelayContro_delay_flag=0;//���̵�������ʱ��ʼ��ʱ����ʱ100ms���ټ�������ѹ�����1Ϊ��ʱ��ɣ�0Ϊû�����

//������ر���
uint8_t aTxBuffer[] = "*********SENDING DATA USING USART1 with DMA***********\r\n";
volatile uint8_t rx_len=0;                     //�������ݳ���
volatile uint8_t recv_end_flag=0;              //������ɱ��λ
uint8_t aRxBuffer1[100];      //���ջ���
uint8_t aTxBuffer1[10];			//����data����
uint8_t aTxBuffer2[5];			//������ͷ����
char  BUFFER_SIZE=100;      //���������ݵ���󳤶ȣ�����Ϊ100
uint8_t usart_flage=0;      //�Ƿ�����λ����������
frames Frames;//����֡


//�������˲�buffer
static float prevData1=0; 
static float prevData2=0; 
static float prevData3=0; 
static float prevData4=0;
static float prevData5=0;
static float prevData6=0;

//PID���
volatile PID pid;//����
volatile PID pid_power_cap;//�㹦��
volatile PID pid_power_out;//�㹦��
float V=0;//�жϿ��Ƶ�ѹ������ʱ�ã�һ�����û����
u8 Cap_Hreshold_Voltage_mode=0;//1Ϊ������ѹ��ֵ�������ѹģʽ��

//pringf �ض���
int fputc(int c, FILE *stream)    //��дfputc����
{ /*    huart1�ǹ������ɴ��붨���UART1�ṹ�壬    ����Ժ�Ҫʹ���������ڴ�ӡ��ֻ��Ҫ������ṹ��ĳ�����UART�ṹ�塣*/  
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
	//�ַ������������
	u8 adc_vale1[12]={0}; //Ҫ������飬̫С���ܻ����
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
	
	PID_int(&pid);//��ʼ������pid
	PID_power_cap_init(&pid_power_cap);//��ʼ�����ݳ��㹦��pid
  PID_power_out_init(&pid_power_out);//��ʼ������㹦��pid
	Frames_init(&Frames);//֡ͷ��ʼ��
	aTxBuffer2[0]=Frames.Frame_Header1;//֡ͷ1
	aTxBuffer2[1]=Frames.Frame_Header2;//֡ͷ2
	aTxBuffer2[3]=Frames.Frame_Verify;//У��λ

	
	__HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);//ʹ��idle�ж�
	HAL_UART_Receive_DMA(&huart1,aRxBuffer1,BUFFER_SIZE);//��DMA���գ����ݴ���aRxBuffer1�����С�	
	HAL_UART_Transmit_DMA(&huart1,aTxBuffer,sizeof(aTxBuffer));// DMA��������
	delay_init(168);
	LCD_Init();
	
	LCD_ShowString(30,30,200,16,16, "-----------------");	
	
 
	//BMI088--------------------------------------------------------------------------------------------------------------
	//��Ļ��ʾ
	//adc�����Լ��
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
	//adc������
	//��˼�ǿ���dma����,����һ���ֵ����ݵ�uhADCxConvertedValue�����������
	//��֤�̵����ǹرյ�
	HAL_GPIO_WritePin(RelayControl0_GPIO_Port, RelayControl0_Pin, GPIO_PIN_RESET);//f0
	HAL_GPIO_WritePin(RelayControl0_GPIO_Port, RelayControl1_Pin, GPIO_PIN_RESET);//f1
	HAL_GPIO_WritePin(RelayControl0_GPIO_Port, RelayControl2_Pin, GPIO_PIN_RESET);//f2
	//
	HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, Uout_Convert(0));//PA4
	HAL_DAC_Start(&hdac, DAC_CHANNEL_1);
	delay_ms(1000);
	
	//24v��ͨ��mos�ܴ�
	HAL_GPIO_WritePin(GPIOF, GPIO_PIN_3, GPIO_PIN_SET);//24v��ͨ
	//��ʱ��Ҫ��󿪣����ں��������ڶ�ʱ����
	HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, Uout_Convert(0));//�������������ѹ
	HAL_DAC_Start(&hdac, DAC_CHANNEL_1);
	//���������ѹ
	HAL_DAC_SetValue(&hdac, DAC_CHANNEL_2, DAC_ALIGN_12B_R, Uout_Convert(0));//ģ�������ѹ
	HAL_DAC_Start(&hdac, DAC_CHANNEL_2);
	//
	HAL_TIM_Base_Start_IT(&htim2);//������ʱ������Ļ��ʼ��֮�󣬲�Ȼ���ܳ����⣬����ԭ���������ܺ���ʱ�й�
	HAL_TIM_Base_Start_IT(&htim3);//������ʱ������Ļ��ʼ��֮�󣬲�Ȼ���ܳ����⣬����ԭ���������ܺ���ʱ�й�
	Self_Inspection();


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1)
	{
		
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  if(Self_Inspecton_flage==0)//û������Լ�Ͳ����п���
	  {
		  break;
  	}
		display();
		//��ʾ���ݲ���
		data_display_adc(uhADCxConvertedValue_PA6_IN6,10);

		

//		//����״̬�ƹر�
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
//�����򣬷��ڶ�ʱ��������ִ������ȷ����
void Control_Program()
{
//adc��ȡ
	//ADC1��˫ͨ��
	for(int i=0;i<10;i=i+2)
	{
		HAL_ADC_Start_DMA(&hadc1, (uint32_t*)&uhADCxConvertedValue_ADC1,4);//��˼�ǿ���dma����,����һ���ֵ����ݵ�uhADCxConvertedValue����������� 
		delay_us(50);//����ʱadc���ݲ������,��ʱ�����ݽ����Ӱ��̫С���ݾ��б��ˣ��ӵ�ʱ��1.0v����ʱż2.v,�Ʋ�Ͳ��������й�
		HAL_ADC_Stop_DMA(&hadc1);
		uhADCxConvertedValue_PA6_IN6[i]=uhADCxConvertedValue_ADC1[0];
		uhADCxConvertedValue_PA7_IN7[i]=uhADCxConvertedValue_ADC1[1];
		uhADCxConvertedValue_PA6_IN6[i+1]=uhADCxConvertedValue_ADC1[2];
		uhADCxConvertedValue_PA7_IN7[i+1]=uhADCxConvertedValue_ADC1[3];
	}
	//ADC2
	for(int i=0;i<10;i=i+2)
	{
		HAL_ADC_Start_DMA(&hadc2, (uint32_t*)&uhADCxConvertedValue_ADC2,4);//��˼�ǿ���dma����,����һ���ֵ����ݵ�uhADCxConvertedValue����������� 
		delay_us(50);//����ʱadc���ݲ������,��ʱ�����ݽ����Ӱ��̫С���ݾ��б��ˣ��ӵ�ʱ��1.0v����ʱż2.v,�Ʋ�Ͳ��������й�
		HAL_ADC_Stop_DMA(&hadc1);
		uhADCxConvertedValue_PA2_IN2[i] =uhADCxConvertedValue_ADC2[0];
		uhADCxConvertedValue_PC4_IN14[i]=uhADCxConvertedValue_ADC2[1];
		uhADCxConvertedValue_PA2_IN2[i+1] =uhADCxConvertedValue_ADC2[2];
		uhADCxConvertedValue_PC4_IN14[i+1]=uhADCxConvertedValue_ADC2[3];
	}

	delay_us(50);//����ʱadc���ݲ������,��ʱ�����ݽ����Ӱ��̫С���ݾ��б��ˣ��ӵ�ʱ��1.0v����ʱż2.v,�Ʋ�Ͳ��������й�
	HAL_ADC_Stop_DMA(&hadc2);
	//ADC3
	HAL_ADC_Start_DMA(&hadc3, (uint32_t*)&uhADCxConvertedValue_PA0,10);
	delay_us(50);//����ʱadc���ݲ������,��ʱ�����ݽ����Ӱ��̫С���ݾ��б��ˣ��ӵ�ʱ��1.0v����ʱż2.v,�Ʋ�Ͳ��������й�
	HAL_ADC_Stop_DMA(&hadc3);
	//
	//���ݴ���(adc���ֵȡ10��ƽ��)
	//�ִ�����޳�,
	out_ADCxConvertedValue_PA6_IN6=AGV_Remove_the_Ross_Error(uhADCxConvertedValue_PA6_IN6,10);//�������ݵ�ѹ
	out_ADCxConvertedValue_PA7_IN7=AGV_Remove_the_Ross_Error(uhADCxConvertedValue_PA7_IN7,10);//�������ݵ���
	out_ADCxConvertedValue_PA0=AGV_Remove_the_Ross_Error(uhADCxConvertedValue_PA0,10);//�������
	//out_ADCxConvertedValue_PA2=AGV_Remove_the_Ross_Error(uhADCxConvertedValue_PA2_IN2,10);//�����ѹ
	out_ADCxConvertedValue_PA2_IN2=AGV_Remove_the_Ross_Error(uhADCxConvertedValue_PA2_IN2,10);//�����ѹ
	out_ADCxConvertedValue_PC4_IN14=AGV_Remove_the_Ross_Error(uhADCxConvertedValue_PC4_IN14,10);//�����ѹ
	
	//�������˲�
	out_ADCxConvertedValue_PA6_IN6=kalmanFilter_A(out_ADCxConvertedValue_PA6_IN6,&prevData1);
	out_ADCxConvertedValue_PA7_IN7=kalmanFilter_A(out_ADCxConvertedValue_PA7_IN7,&prevData4);
	out_ADCxConvertedValue_PA0=kalmanFilter_A(out_ADCxConvertedValue_PA0,&prevData2);
	//out_ADCxConvertedValue_PA2=kalmanFilter_A(out_ADCxConvertedValue_PA2,&prevData3);
	out_ADCxConvertedValue_PA2_IN2=kalmanFilter_A(out_ADCxConvertedValue_PA2_IN2,&prevData3);
	out_ADCxConvertedValue_PC4_IN14=kalmanFilter_A(out_ADCxConvertedValue_PC4_IN14,&prevData6);
	

	//�����Ŵ�ԭ��ʵֵ
	adv1_IN6_real=out_ADCxConvertedValue_PA6_IN6*0.00080566*10.8;//0.00080566Ϊ�ֱ��ʣ�11Ϊ�����ѹ��ķŴ���//�������ݵ�ѹ
	adv1_IN7_real=(out_ADCxConvertedValue_PA7_IN7-bias_ADC)*0.00080566*7.094*1.27 ;//�������ݵ���----0.00080566Ϊ�ֱ��ʣ�*1.066���ӽӽ���ʵ����������Ϊ�˷�ֹ�����������⽫�Ŵ�������ȡ��
	adv3_real=(out_ADCxConvertedValue_PA0-bias_ADC)*0.00080566*7.094*1.19;//*0.00080566*7.094/2;//�������
	adv3_real=kalmanFilter_A(adv3_real,&prevData5);//�������˲�,��һ���˲�Ч������̫��
	adv2_IN2_real=out_ADCxConvertedValue_PA2_IN2*0.00080566*10.8;//�����ѹ
	adv2_IN14_real=out_ADCxConvertedValue_PC4_IN14*0.00080566*10.8;//�����ѹ
	
	//!!!!!!!��Ϊ���������ԭ�򣬵���ֻҪ�ڴ���0.16ʱ�����Ʒ��ص�ֵ����׼�ģ���������������
	
	//���ʼ���
	power_input=adv2_IN14_real*adv3_real;//���빦�� ��ѹ*����
	power_cap=adv1_IN6_real*adv1_IN7_real;//�������ݳ�繦�ʣ���ѹ*����
	power_output=power_input-power_cap-power_sys;//�������
	
	
	//Automatic_Voltage_Acquisition();�Զ���ȡÿ��DAC���ʱ��Ӧ��ѹ��ֵ��һ��������4096�Σ��ô��������� �� ����115200
  if(Self_Inspecton_flage==0)//û������Լ�Ͳ����п���
	{
		return ;
	}
	//PID����
	//���ʼ��
	if(power_input>=power_set+2)
	{
		flage=1;
		HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, Uout_Convert(0));//���ݲ����
		PID_power_cap_init(&pid_power_cap);//��ʼ���㹦��pid <-���������ʼ����pid_power_cap.result��ֵ��һֱ���������´γ�������Ҫ���ʱӦ������
		pid_power_out.setpoint = power_set;
	
	}
	else if(power_input<power_set-5)
	{
		flage=0;
		PID_power_out_init(&pid_power_out);//��ʼ���㹦��pid <-���������ʼ����pid_power_cap.result��ֵ��һֱ���������´γ�������Ҫ���ʱӦ������
		//��ԭ���������ѹ
		HAL_DAC_SetValue(&hdac, DAC_CHANNEL_2, DAC_ALIGN_12B_R, Uout_Convert(Voltag_out));//PA5
		HAL_DAC_Start(&hdac, DAC_CHANNEL_2);
	}
	//���ʳ�������ʣ�flage=1��
	if(flage==1)
	{
		if(pid_power_out.result>=0)
		{
			PIDRegulation(&pid_power_out, power_input);//pid ���� ���� ������ŵ�����Ҫά�ֵĹ��ʣ�������power_input/power_output
			HAL_DAC_SetValue(&hdac, DAC_CHANNEL_2, DAC_ALIGN_12B_R, Uout_Convert(pid_power_out.result));//PA5
		}
		else if(pid_power_out.result<0||pid_power_out.result>50)
		{
			HAL_DAC_SetValue(&hdac, DAC_CHANNEL_2, DAC_ALIGN_12B_R, Uout_Convert(0));//PA5
		}
	  
		HAL_DAC_Start(&hdac, DAC_CHANNEL_2);
	}
	//����С�ڵ��ڶ���ʣ�flage=0��
	if(flage==0&&RelayContro_flag==0)//�ڼ̵�������ͨʱ
	{
		//HAL_DAC_SetValue(&hdac, DAC_CHANNEL_2, DAC_ALIGN_12B_R, 4095);//PA5
		
		if(adv1_IN7_real>=9.0)//��������9A���ر�pid
		{
			HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, Uout_Convert(pid_power_cap.result-1));//PA4
			HAL_GPIO_WritePin(GPIOF, GPIO_PIN_10, GPIO_PIN_RESET);//f10
			PID_power_cap_init(&pid_power_cap);//��ʼ���㹦��pid
		}
		else if((adv1_IN6_real>Cap_Hreshold_Voltage) && (Cap_Hreshold_Voltage_mode==0))//�������ݳ���ѹ������ֵ��ѹ���ر�pid
		{
			//HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, Uout_Convert(Cap_Hreshold_Voltage+0.2));//PA4,��0.2��Ϊ��ά���������ѹ��״̬������
			HAL_GPIO_WritePin(GPIOF, GPIO_PIN_9, GPIO_PIN_RESET);
			Cap_Hreshold_Voltage_mode=1;
			PID_power_cap_init(&pid_power_cap);//��ʼ���㹦��pid <-���������ʼ����pid_power_cap.result��ֵ��һֱ���������´γ�������Ҫ���ʱӦ������
		}
		else if(pid_power_cap.result<0.0||pid_power_cap.result>100)//PID���С��0   
		{
			HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, Uout_Convert(0));//PA4
			PID_power_cap_init(&pid_power_cap);//��ʼ���㹦��pid
		}
		else  //�������
		{
			if(Cap_Hreshold_Voltage_mode==0)//�㹦��ģʽ
			{
				pid_power_cap.setpoint=(int16_t)(power_set-power_sys-power_output);
				if(pid_power_cap.setpoint<0)//��ֹ��ֵС��0��
				{
					pid_power_cap.setpoint=0;
				}
				PIDRegulation(&pid_power_cap, power_cap);//pid ���� ����
				HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, Uout_Convert(pid_power_cap.result));//PA4
				HAL_GPIO_WritePin(GPIOF, GPIO_PIN_10, GPIO_PIN_SET);//f10
				HAL_GPIO_WritePin(GPIOF, GPIO_PIN_9, GPIO_PIN_SET);//f9
			}
			else if(Cap_Hreshold_Voltage_mode==1)//��ѹģʽ
			{
				HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, Uout_Convert(Cap_Hreshold_Voltage));
			}

		}
		
		//HAL_DAC_Start(&hdac, DAC_CHANNEL_2);
		HAL_DAC_Start(&hdac, DAC_CHANNEL_1);
		flage=0;
	}
	else if(flage==0&&RelayContro_flag==1)//�̵�����ͨ�����������������ʱ������dcdc�������������0V��ѹ������̵����Ͽ����´γ�磩ʱ����δ֪����
	{
		HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, Uout_Convert(0));//PA4
		HAL_DAC_Start(&hdac, DAC_CHANNEL_1);
	}
//	HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, Uout_Convert(13));//PA4
//	HAL_DAC_Start(&hdac, DAC_CHANNEL_1);
	//�̵�������
	if(RelayContro_delay_flag==1 && adv2_IN2_real<=Voltag_out1_min)//�ȴ���ʱ100ms��������ѹ���
	{
		Relay_Control(0);
	}
	//�������ݷ���



	//����״̬�ƹر�
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
void data_display_adc(uint16_t* ADCxConvertedValue,int Length)//��ʾ����
{
	char adc_vale[50]={0};//���ܻ����
	uint16_t ADCxConvertedValue_copy[Length];
	uint16_t avg=0;
	
	double var=0;
	int difference_value=0;
	int i=0;

	if(flage_display==1)//��ʾ����ϸ��
	{

		LCD_Draw_Output_waveform(400,400,30,70,BLUE,out_ADCxConvertedValue_PA0,Waveform_Date1);//370,760
		
	
	}
	else if(flage_display==0)//�����Ļ
	{
		LCD_Clear(WHITE);//��һ�°�����һ�Σ�һֱ��ˢ��̫��
		flage_display=3;//��ֹ��ͣ�������Ļ
	}
	else if(flage_display==2)//��ʾ����
	{
	//	LCD_Clear(WHITE);uint16_t
		LCD_Draw_Output_waveform(400,400,30,70,BLUE,(uint16_t)power_cap*40,Waveform_Date1);//�������빦��
		LCD_Draw_Output_waveform(400,770,30,440,BLUE,out_ADCxConvertedValue_PA7_IN7,Waveform_Date2);//���ݵ���
	}
	else if(flage_display==3)//��ͣ
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
		//printf("rx_len=%d\r\n",rx_len);//��ӡ���ճ���
		HAL_UART_Transmit(&huart1,aRxBuffer1, rx_len,100);//�������ݴ�ӡ����
		HAL_UART_Transmit(&huart1,"\r\n", sizeof("\r\n"),100);//�������ݴ�ӡ����
//  		HAL_UART_Transmit_DMA(&huart1,aRxBuffer1, rx_len);//�������ݴ�ӡ����
//  		HAL_UART_Transmit_DMA(&huart1,"\r\n", sizeof("\r\n"));//�������ݴ�ӡ����
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
		else if(memcmp(aRxBuffer1,"power_set",strlen("power_set"))==0)//����λ����ȡָ��
		{
			x=strlen(aRxBuffer1)-strlen("power_set");
			for(int i=0;i<(strlen(aRxBuffer1)-strlen("power_set"));i++)
			{
				str[i]=aRxBuffer1[strlen("power_set")+i];
			}
			if(atoi(str) <= (power_set+30))//power_set��ֵ���ܱ仯̫��̫��Ļ�ϵͳ�ͻ᲻�ȶ����������β�ֵ������30
			{
				power_set = atoi(str);
			}
			else if(atoi(str) > (power_set+30))//���β�ֵ����30���µ�ֵΪpower_set+30��������λ�����´����ݣ�
			{
				power_set = power_set+30;
				aTxBuffer2[4]=Frames.power_input_set_error;//���ݹ���λ��������λ��power_set�����ݴ��´�һ��
				sprintf(aTxBuffer1,"%.3lf\r\n",0);//\r\n
				aTxBuffer2[2]=(sizeof(aTxBuffer1));//���ݳ���λ
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
			aRxBuffer1[i]=0;//����ջ���
		}
		rx_len=0;//�������
		recv_end_flag=0;//������ս�����־λ
	}
	HAL_UART_Receive_DMA(&huart1,aRxBuffer1,BUFFER_SIZE);// ����DMA����
	//..................................................................
}
//��ʱ���ж�
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	static u8 flage_temp=0;
	static u8 flage_temp2=0;
	static u32 time=0;
	//��ʱ��2
  if(htim==(&htim2))
  {
		//��������ֹͣ���󣬴��������Դ��
		if(RelayContro_flag==1)//�̵�������,��ʼ��ʱ 
		{
			if(time >= 10)//��ɼ�ʱ
			{
				RelayContro_delay_flag=1;
				HAL_GPIO_WritePin(GPIOF, GPIO_PIN_10, GPIO_PIN_RESET);//״̬����
				time=0;
			}
			else if(time <10)//û����ɼ�ʱ
			{
				time=time+1;
			}
		}
		else if(RelayContro_flag==0)//�̵���û�й���
		{
			HAL_GPIO_WritePin(GPIOF, GPIO_PIN_10, GPIO_PIN_SET);//״̬����
		}
		if (usart_flage==1)
		{

			UartSend();
			
		}
		else if(usart_flage==0)
		{
		}
		usart_feedback();//���ڽ��պ����źŵĺ��������Կ�����ʾ���ݣ���ʾ����pc���ͣ�waveform����ʾ����pc���ͣ�date�������ʾpc���ͣ�clear

    

  }
	//��ʱ��3��
	if(htim==(&htim3))
	{
		
		
		Control_Program();//��Ҫ���ƺ���

		
	}
}
void Frames_init(frames *vframes)
{
	vframes->Frame_Header1=0xAA ;
	vframes->Frame_Header2=0x0A ;
	vframes->Frame_Len =0;
	vframes->Frame_Verify =0x01;
	//����λ
	vframes->Voltage_Input = 0x11;//Ϊ������У��λ���д�0x11��ʼ
	vframes->Current_Input = 0x12;
	vframes->Voltage_Output = 0x13;
	vframes->Current_Output = 0x14;
	vframes->Voltage_Cap_Input = 0x15;
	vframes->Current_Cap_Input = 0x16;
	vframes->Voltage_Cap_Output = 0x17;
	vframes->power_input = 0x18;//���빦��
	vframes->power_cap = 0x19;//�������ݳ�繦��
	vframes->power_output = 0x1A;//�������
	vframes->power_input_set_error = 0xFF;//�������

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
