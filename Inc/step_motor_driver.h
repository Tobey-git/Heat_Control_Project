/**
  ******************************************************************************
  * @file    step_motor_driver.h
  * @author  Tobey
  * @version V1.0.0
  * @date    2-August-2016
  * @brief   �����������ģ���ͷ�ļ�
  *  
  ******************************************************************************
  * @attention
	* ���������к궨�弰��������STEP_MOTOR��ͷ
	*	���й��ܺ�������ʹ��BSP_STEP_MOTOR��ͷ
	*
  ******************************************************************************
  */

/* Define to prevent recursive includion -------------------------------------*/
#ifndef __STEP_MOTOR_DRIVER_H
#define __STEP_MOTOR_DRIVER_H

#ifdef __cpluscplus
	extern "C" {
#endif
		
/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"
		
/* Exported types ------------------------------------------------------------*/
/**
  * @brief  �����������ϸ��������
  */
typedef enum 
{
  FULL,/**<  Full step */
  HALF,/**<  Half step */
	QUARTER,/**<  Quarter step */
	EIGHTH,/**<  Eighth step */
  
} STEP_MOTOR_MicroStep_Def;

/**
  * @brief  ���������ת����
  */
typedef enum 
{
  FORWARD,/**<  ˳ʱ�뷽�� */
  REVERSAL,/**<  ��ʱ�뷽�� */
  
} STEP_MOTOR_Dir_Def;

/** 
  * @brief   STEP_MOTOR Init structure definition  
  */ 
typedef struct
{
  STEP_MOTOR_Dir_Def Dir;      												/*!< �����ת���� */ 
  STEP_MOTOR_MicroStep_Def MicroStep;      						/*!< ����ϸ���� */
	int32_t Step;       																/*!< ��Ҫ��ת�Ĳ��� */
	
} STEP_MOTOR_InitTypeDef;

/* Exported constants --------------------------------------------------------*/
extern STEP_MOTOR_InitTypeDef STEP_MOTOR_Msg;					/*!< ��������ṹ�� */
extern TIM_HandleTypeDef STEP_MOTOR_Timx;							/*!< ���������ʱ���ṹ�� */

/* Exported macro ------------------------------------------------------------*/
/* GPIO ----*/
#define STEP_MOTOR_STEP_GPIO_CLK_ENABLE()       __GPIOA_CLK_ENABLE()     					/*!< �������STEP����ʱ��ʹ�ܿ� */
#define STEP_MOTOR_STEP_GPIO_PORT 							GPIOA															/*!< �������STEP���Ŷ˿� */
#define STEP_MOTOR_STEP_GPIO_PIN 								GPIO_PIN_2												/*!< �������STEP���� */

#define STEP_MOTOR_DIR_GPIO_CLK_ENABLE()       	__GPIOC_CLK_ENABLE()    					/*!< �������DIR����ʱ��ʹ�ܿ� */
#define STEP_MOTOR_DIR_GPIO_PORT 								GPIOC															/*!< �������DIR���Ŷ˿� */
#define STEP_MOTOR_DIR_GPIO_PIN 								GPIO_PIN_15												/*!< �������DIR���� */

#define STEP_MOTOR_MS1_GPIO_CLK_ENABLE()        __GPIOA_CLK_ENABLE()     					/*!< �������MS1����ʱ��ʹ�ܿ� */
#define STEP_MOTOR_MS1_GPIO_PORT GPIOA																						/*!< �������MS1���Ŷ˿� */
#define STEP_MOTOR_MS1_GPIO_PIN GPIO_PIN_0																				/*!< �������MS1���� */

#define STEP_MOTOR_MS2_GPIO_CLK_ENABLE()        __GPIOA_CLK_ENABLE()     					/*!< �������MS2����ʱ��ʹ�ܿ� */
#define STEP_MOTOR_MS2_GPIO_PORT 								GPIOA															/*!< �������MS2���Ŷ˿� */
#define STEP_MOTOR_MS2_GPIO_PIN 								GPIO_PIN_1												/*!< �������MS2���� */

/* Exported functions --------------------------------------------------------*/
void BSP_STEP_MOTOR_Init(void);
void BSP_STEP_MOTOR_MicroStep_Set(STEP_MOTOR_MicroStep_Def Step);
void BSP_STEP_MOTOR_Dir_Set(STEP_MOTOR_Dir_Def Dir);

#ifdef __cplusplus
}
#endif

#endif /* __STEP_MOTOR_DRIVER_H */

/************************ (C) COPYRIGHT Tobey *****END OF FILE****/		
