/**
  ******************************************************************************
  * @file    step_motor_driver.h
  * @author  Tobey
  * @version V1.0.0
  * @date    2-August-2016
  * @brief   步进电机控制模块的头文件
  *  
  ******************************************************************************
  * @attention
	* 本驱动所有宏定义及变量名以STEP_MOTOR开头
	*	所有功能函数命名使用BSP_STEP_MOTOR开头
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
  * @brief  步进电机步进细分数量级
  */
typedef enum 
{
  FULL,/**<  Full step */
  HALF,/**<  Half step */
	QUARTER,/**<  Quarter step */
	EIGHTH,/**<  Eighth step */
  
} STEP_MOTOR_MicroStep_Def;

/**
  * @brief  步进电机旋转方向
  */
typedef enum 
{
  FORWARD,/**<  顺时针方向 */
  REVERSAL,/**<  逆时针方向 */
  
} STEP_MOTOR_Dir_Def;

/** 
  * @brief   STEP_MOTOR Init structure definition  
  */ 
typedef struct
{
  STEP_MOTOR_Dir_Def Dir;      												/*!< 电机旋转方向 */ 
  STEP_MOTOR_MicroStep_Def MicroStep;      						/*!< 步进细分数 */
	int32_t Step;       																/*!< 需要旋转的步数 */
	
} STEP_MOTOR_InitTypeDef;

/* Exported constants --------------------------------------------------------*/
extern STEP_MOTOR_InitTypeDef STEP_MOTOR_Msg;					/*!< 步进电机结构体 */
extern TIM_HandleTypeDef STEP_MOTOR_Timx;							/*!< 步进电机定时器结构体 */

/* Exported macro ------------------------------------------------------------*/
/* GPIO ----*/
#define STEP_MOTOR_STEP_GPIO_CLK_ENABLE()       __GPIOA_CLK_ENABLE()     					/*!< 步进电机STEP引脚时钟使能口 */
#define STEP_MOTOR_STEP_GPIO_PORT 							GPIOA															/*!< 步进电机STEP引脚端口 */
#define STEP_MOTOR_STEP_GPIO_PIN 								GPIO_PIN_2												/*!< 步进电机STEP引脚 */

#define STEP_MOTOR_DIR_GPIO_CLK_ENABLE()       	__GPIOC_CLK_ENABLE()    					/*!< 步进电机DIR引脚时钟使能口 */
#define STEP_MOTOR_DIR_GPIO_PORT 								GPIOC															/*!< 步进电机DIR引脚端口 */
#define STEP_MOTOR_DIR_GPIO_PIN 								GPIO_PIN_15												/*!< 步进电机DIR引脚 */

#define STEP_MOTOR_MS1_GPIO_CLK_ENABLE()        __GPIOA_CLK_ENABLE()     					/*!< 步进电机MS1引脚时钟使能口 */
#define STEP_MOTOR_MS1_GPIO_PORT GPIOA																						/*!< 步进电机MS1引脚端口 */
#define STEP_MOTOR_MS1_GPIO_PIN GPIO_PIN_0																				/*!< 步进电机MS1引脚 */

#define STEP_MOTOR_MS2_GPIO_CLK_ENABLE()        __GPIOA_CLK_ENABLE()     					/*!< 步进电机MS2引脚时钟使能口 */
#define STEP_MOTOR_MS2_GPIO_PORT 								GPIOA															/*!< 步进电机MS2引脚端口 */
#define STEP_MOTOR_MS2_GPIO_PIN 								GPIO_PIN_1												/*!< 步进电机MS2引脚 */

/* Exported functions --------------------------------------------------------*/
void BSP_STEP_MOTOR_Init(void);
void BSP_STEP_MOTOR_MicroStep_Set(STEP_MOTOR_MicroStep_Def Step);
void BSP_STEP_MOTOR_Dir_Set(STEP_MOTOR_Dir_Def Dir);

#ifdef __cplusplus
}
#endif

#endif /* __STEP_MOTOR_DRIVER_H */

/************************ (C) COPYRIGHT Tobey *****END OF FILE****/		
