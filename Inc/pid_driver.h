/**
  ******************************************************************************
  * @file    pid_driver.h
  * @author  Tobey
  * @version V1.0.0
  * @date    31-August-2016
  * @brief   PID����ģ���ͷ�ļ�
  *  
  ******************************************************************************
  * @attention
	* ���������к궨�弰��������STEP_MOTOR��ͷ
	*	���й��ܺ�������ʹ��BSP_STEP_MOTOR��ͷ
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
  * @brief  pid�ṹ��
  */
typedef struct PID
{
	int SetPoint; 			 /*!< �趨Ŀ�� Desired Value */ 
	long SumError;       /*!< ����ۼ� */ 
	double Proportion;   /*!< �������� Proportional Const */
	double Integral;     /*!< ���ֳ��� Integral Const */ 
	double Derivative;   /*!< ΢�ֳ��� Derivative Const */ 
	int LastError;       /*!< Error[-1] */
	int PrevError; 			 /*!< Error[-2] */
} PID;

/* Exported functions --------------------------------------------------------*/
void PID_Init(PID *sptr);//����PID������ʼ��

void PID_SetPoint(PID *sptr, int SetPoint);//���� PID ���ڵ�Ŀ��ֵ
int PID_GetSetpoint(PID *sptr);//��ȡ PID �������õ�Ŀ��ֵ
void PID_SetKp(PID *sptr, double Kp);//���� PID �� Kp ֵ
double PID_GetKp(PID *sptr);//��ȡ PID �������õ� Kp ֵ
void PID_SetKi(PID *sptr, double dKii);//���� PID �� Ki ֵ
double PID_GetKi(PID *sptr);//��ȡ PID �������õ� Ki ֵ
void PID_SetKd(PID *sptr, double dKdd);//���� PID �� Kd ֵ
double PID_GetKd(PID *sptr);//��ȡ PID �������õ� Kd ֵ

int PID_IncCalc(PID *sptr, int NextPoint);//PID����ʽ�������
unsigned int PID_LocCalc(PID *sptr, int NextPoint);//PIDλ��ʽ�������

void PID_LocPWMCtr(uint8_t Channel, uint8_t pulse);
void PID_IncPWMCtr(uint8_t Channel, uint8_t Inc);

#ifdef __cplusplus
}
#endif

#endif /* __PID_DRIVER_H */

/************************ (C) COPYRIGHT Tobey *****END OF FILE****/		
