/**
  ******************************************************************************
  * @file    pid_driver.h
  * @author  Tobey
  * @version V1.0.0
  * @date    31-August-2016
  * @brief   PID控制模块的头文件
  *  
  ******************************************************************************
  * @attention
	* 本驱动所有宏定义及变量名以STEP_MOTOR开头
	*	所有功能函数命名使用BSP_STEP_MOTOR开头
	*
  ******************************************************************************
  */

/* Define to prevent recursive includion -------------------------------------*/
#ifndef __PID_DRIVER_H
#define __PID_DRIVER_H

#ifdef __cpluscplus
	extern "C" {
#endif
		
/* Includes ------------------------------------------------------------------*/
#include "tim.h"
#include "stm32f1xx_hal.h"
		
/* Exported types ------------------------------------------------------------*/

/**
  * @brief  pid结构体
  */
typedef struct PID
{
	int SetPoint; 			 /*!< 设定目标 Desired Value */ 
	long SumError;       /*!< 误差累计 */ 
	double Proportion;   /*!< 比例常数 Proportional Const */
	double Integral;     /*!< 积分常数 Integral Const */ 
	double Derivative;   /*!< 微分常数 Derivative Const */ 
	int LastError;       /*!< Error[-1] */
	int PrevError; 			 /*!< Error[-2] */
} PID;

/* Exported functions --------------------------------------------------------*/
void PID_Init(PID *sptr);//增量PID参数初始化

void PID_SetPoint(PID *sptr, int SetPoint);//设置 PID 调节的目标值
int PID_GetSetpoint(PID *sptr);//读取 PID 调节设置的目标值
void PID_SetKp(PID *sptr, double Kp);//设置 PID 的 Kp 值
double PID_GetKp(PID *sptr);//读取 PID 中所设置的 Kp 值
void PID_SetKi(PID *sptr, double dKii);//设置 PID 的 Ki 值
double PID_GetKi(PID *sptr);//读取 PID 中所设置的 Ki 值
void PID_SetKd(PID *sptr, double dKdd);//设置 PID 的 Kd 值
double PID_GetKd(PID *sptr);//读取 PID 中所设置的 Kd 值

int PID_IncCalc(PID *sptr, int NextPoint);//PID增量式控制设计
unsigned int PID_LocCalc(PID *sptr, int NextPoint);//PID位置式控制设计

void PID_LocPWMCtr(uint8_t Channel, uint8_t pulse);
void PID_IncPWMCtr(uint8_t Channel, uint8_t Inc);

#ifdef __cplusplus
}
#endif

#endif /* __PID_DRIVER_H */

/************************ (C) COPYRIGHT Tobey *****END OF FILE****/		
