/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
typedef struct 
{
	u8 Frame_Header1 ;
	u8 Frame_Header2 ;
	u8 Frame_Len ;
	u8 Frame_Verify;
	//����λ
	u8 Voltage_Input;
	u8 Current_Input;
	u8 Voltage_Output;
	u8 Current_Output;
	u8 Voltage_Cap_Input;
	u8 Current_Cap_Input;
	u8 Voltage_Cap_Output;
	u8 power_input;//���빦��
	u8 power_cap;//�������ݳ�繦��
	u8 power_output;//�������
	u8 power_input_set_error;
} frames;

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);//�ⲿ�ж�
//double ADC_change(uint16_t* ADCxConvertedValue, int Length  );
void data_display_adc(uint16_t* ADCxConvertedValue,int Length);//��ʾ���ݣ������ֿ�ѡ��ʾ���Σ���ʾ���ݣ������ʾ
void usart_feedback();//���ڽ��պ����źŵĺ��������Կ�����ʾ���ݣ���ʾ����pc���ͣ�waveform����ʾ����pc���ͣ�date�������ʾpc���ͣ�clear
void Control_Program();//�����򣬷��ڶ�ʱ��������ִ������ȷ����
void Frames_init(frames *vframes) ;

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define RelayControl0_Pin GPIO_PIN_0
#define RelayControl0_GPIO_Port GPIOF
#define RelayControl1_Pin GPIO_PIN_1
#define RelayControl1_GPIO_Port GPIOF
#define RelayControl2_Pin GPIO_PIN_2
#define RelayControl2_GPIO_Port GPIOF
#define POWER_ON_OFF_Pin GPIO_PIN_3
#define POWER_ON_OFF_GPIO_Port GPIOF
#define LED0_Pin GPIO_PIN_9
#define LED0_GPIO_Port GPIOF
#define LED1_Pin GPIO_PIN_10
#define LED1_GPIO_Port GPIOF
/* USER CODE BEGIN Private defines */


/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
