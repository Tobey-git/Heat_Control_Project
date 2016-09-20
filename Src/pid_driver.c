/**
  ******************************************************************************
  * @file    pid_driver.c
  * @author  Tobey
  * @version V1.0.0
  * @date    31-August-2016
  * @brief   ���ļ���PID���Ƶ�����ģ��
  * 				 ���ļ��ṩ���й������ڶ�PID���й���
	*						+ ��ʼ��PID����
	*						+ ����PID
	*						+ ��ȡPID������
  ******************************************************************************
  * @attention
	*
	*
	********* ʹ�÷�ʽ *********
	* 1����Դ�ļ�����������ͷ�ļ�
	*		#include "pid_driver.h"
	* 2����ʼ��pid������pid������
	*   PID_Init(*sptr); //��ʼ��PID����PID���õ������в������㣩
	*		PID_SetPoint(*sptr, SetPoint); //���� PID ���ڵ�Ŀ��ֵ
	*   PID_SetKp(*sptr, Kp); //���� PID �� Kp ֵ
	*   PID_SetKi(*sptr, Ki); //���� PID �� Ki ֵ
	*   PID_SetKd(*sptr, Kd); //���� PID �� Kd ֵ
	*	3�����ݲ��õĿ��Ʒ�����ͬ��ȡ��Ӧ��PID��������
	* 	Inc = PID_IncCalc(*sptr, NextPoint): ��ȡָ��PID����ʽ������
	* 	Loc = PID_LocCalc(*sptr, NextPoint): ��ȡָ��PIDλ��ʽ������
	*
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "pid_driver.h"


/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/

/**
  * @brief  PID ������ʼ��.
  * @param  *sptr��PID�ṹ��ָ��.
  * @note   ��PID���õ������в�������.
  * @retval None
  */
void PID_Init(PID *sptr)
{
	sptr->SumError = 0;
	sptr->LastError = 0; 
	sptr->PrevError = 0; 
	sptr->Proportion = 0; 
	sptr->Integral = 0; 
	sptr->Derivative = 0; 
	sptr->SetPoint = 0;
}

/**
  * @brief  ���� PID ���ڵ�Ŀ��ֵ.
  * @param  *sptr��PID�ṹ��ָ��.
	* @param  Point��PIDĿ��ֵ.
  * @note   None.
  * @retval None
  */
void PID_SetPoint(PID *sptr, int SetPoint)
{
	sptr->SetPoint = SetPoint;
}

/**
  * @brief  ��ȡ PID �������õ�Ŀ��ֵ.
  * @param  *sptr��PID�ṹ��ָ��.
  * @note   None.
  * @retval PIDĿ��ֵ
  */
int PID_GetSetpoint(PID *sptr)
{
	return sptr->SetPoint;
}

/**
  * @brief  ���� PID �� Kp ֵ.
  * @param  *sptr��PID�ṹ��ָ��.
	* @param  Kp��Kpֵ.
  * @note   ������������� PID ��λ�� PID �ļ����д����Ų�ͬ����˼.
	*							λ��ʽ PID �� Kp ���Ǳ���ϵ��
	*							����ʽ PID �� Kp �൱�� ek ��ϵ�� Kp(1+T/Ti+Td/T��
  * @retval None
  */
void PID_SetKp(PID *sptr, double Kp)
{
	sptr->Proportion = Kp;
}

/**
  * @brief  ��ȡ PID �������õ� Kp ֵ.
  * @param  *sptr��PID�ṹ��ָ��.
  * @note   ����ֵ������ PID ��λ�� PID �ļ����д����Ų�ͬ����˼.
	*								λ��ʽ PID �� Kp ���Ǳ���ϵ��
	*								����ʽ PID �� Kp �൱�� ek ��ϵ�� Kp(1+T/Ti+Td/T��.
  * @retval Kp
  */
double PID_GetKp(PID *sptr)
{
	return sptr->Proportion;
}

/**
  * @brief  ���� PID �� Ki ֵ.
  * @param  *sptr��PID�ṹ��ָ��.
	* @param  dKii��Ki.
  * @note   ������������� PID ��λ�� PID �ļ����д����Ų�ͬ����˼.
	*								λ��ʽ PID �� Ki �ǻ���ϵ��Kp*T/Ti
	*								����ʽ PID �� Ki �൱�� ek-1 ��ϵ�� Kp(1+2*Td/T��.
  * @retval None
  */
void PID_SetKi(PID *sptr, double dKii)
{
	sptr->Integral = dKii;
}

/**
  * @brief  ��ȡ PID �������õ� Ki ֵ.
  * @param  *sptr��PID�ṹ��ָ��.
  * @note   ����ֵ������ PID ��λ�� PID �ļ����д����Ų�ͬ����˼.
	*								λ��ʽ PID �� Ki �ǻ���ϵ��Kp*T/Ti
	*								����ʽ PID �� Ki �൱�� ek-1 ��ϵ�� Kp(1+2*Td/T��.
  * @retval Ki
  */
double PID_GetKi(PID *sptr)
{
	return sptr->Integral;
}

/**
  * @brief  ���� PID �� Kd ֵ.
  * @param  *sptr��PID�ṹ��ָ��.
	* @param  dKdd��Kd.
  * @note   ������������� PID ��λ�� PID �ļ����д����Ų�ͬ����˼.
	*								λ��ʽ PID �� Kd �ǻ���ϵ��Kp*Td/T
	*								����ʽ PID �� Kd �൱�� ek-2 ��ϵ�� Kp*Td/T.
  * @retval None
  */
void PID_SetKd(PID *sptr, double dKdd)
{
	sptr->Derivative = dKdd;
}

/**
  * @brief  ��ȡ PID �������õ� Kd ֵ.
  * @param  *sptr��PID�ṹ��ָ��.
  * @note   ����ֵ������ PID ��λ�� PID �ļ����д����Ų�ͬ����˼.
	*								λ��ʽ PID �� Kd �ǻ���ϵ��Kp*Td/T
	*								����ʽ PID �� Kd �൱�� ek-2 ��ϵ�� Kp*Td/T.
  * @retval Kd
  */
double PID_GetKd(PID *sptr)
{
	return sptr->Derivative;
}

/**
  * @brief  ����ʽ PID �������.
  * @param  *sptr��PID�ṹ��ָ��.
  * @param  NextPoint��PID�������.
  * @note   None.
  * @retval iIncpid��PID����ֵ
  */
int PID_IncCalc(PID *sptr, int NextPoint)
{
	register int iError, iIncpid;
	 //��ǰ���
	iError = sptr->SetPoint - NextPoint;
	 //��������
	iIncpid = sptr->Proportion * iError //E[k] ��
		 - sptr->Integral * sptr->LastError //E[k��1] ��
		 + sptr->Derivative * sptr->PrevError; //E[k��2] ��
	 //�洢�������´μ���
	sptr->PrevError = sptr->LastError;
	sptr->LastError = iError;
	 //��������ֵ
	return(iIncpid);
}

/**
  * @brief  λ��ʽ PID �������.
  * @param  *sptr��PID�ṹ��ָ��.
  * @param  NextPoint��PID�������.
  * @note   None.
  * @retval PIDλ��ʽ������
  */
unsigned int PID_LocCalc(PID *sptr, int NextPoint)
{
	register int iError,dError;
	iError = sptr->SetPoint - NextPoint; //ƫ��
	sptr->SumError += iError;   //����
	dError = iError - sptr->LastError; //΢��
	sptr->LastError = iError;
	return(sptr->Proportion * iError //������
		 + sptr->Integral * sptr->SumError //������
		 + sptr->Derivative * dError); //΢����
}

/**
  * @brief  λ��ʽPID��PWMռ�ձȿ���.
  * @param  Channel��ͨ�����.
  * @param  pulse��ռ�ձ�.
  * @note   None.
  * @retval None
  */
void PID_LocPWMCtr(uint8_t Channel, uint8_t pulse)
{
	// check PWM status
	/*�����ж�PWM�仯���������ǰ����ռ�ձ����ϴ����õ�ֵ��ͬ����ֱ�ӷ���*/
	static uint8_t pre_pulse = 0;
	
	if(pre_pulse == pulse)
	{
		return;
	}
	else
	{
		pre_pulse = pulse;
	}
	
	// configure the PWM pulses
	/*����ռ�ձ�Pulse*/
	TIM_OC_InitTypeDef sConfigOC;
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = pulse;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_LOW;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	/*�����ƶ���ʱ����Ӧͨ����PWM���*/
	switch(Channel)
	{
		case 0: // H_HEAT1
			HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_3);
			HAL_TIM_PWM_Start_IT(&htim4,TIM_CHANNEL_3);
			break;
		case 1: // H_HEAT2
			HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2);
			HAL_TIM_PWM_Start_IT(&htim2,TIM_CHANNEL_2);
			break;
		case 2: // H_HEAT3
			HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1);
			HAL_TIM_PWM_Start_IT(&htim3,TIM_CHANNEL_1);
			break;
		case 3: // H_HEAT4
			HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2);
			HAL_TIM_PWM_Start_IT(&htim3,TIM_CHANNEL_2);
			break;
		case 4: // H_HEAT5
			HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1);
			HAL_TIM_PWM_Start_IT(&htim4,TIM_CHANNEL_1);
			break;
		case 5: // H_HEAT6
			HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_2);
			HAL_TIM_PWM_Start_IT(&htim4,TIM_CHANNEL_2);
			break;
		default:
			break;
	}

}

/**
  * @brief  ����ʽPID��PWMռ�ձȿ���.
  * @param  Channel��ͨ�����.
  * @param  Inc������ֵ.
  * @note   None.
  * @retval None
  */
void PID_IncPWMCtr(uint8_t Channel, uint8_t Inc)
{
	// check PWM status
	/*�����ж�PWM�仯���������ǰ����ռ�ձ����ϴ����õ�ֵ��ͬ����ֱ�ӷ���*/
	static uint8_t pre_pulse = 0;
	
	if(Inc == 0)
	{
		return;
	}
	else
	{
		pre_pulse += Inc;
	}
	
	// configure the PWM pulses
	/*����ռ�ձ�Pulse*/
	TIM_OC_InitTypeDef sConfigOC;
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = pre_pulse;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_LOW;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	/*�����ƶ���ʱ����Ӧͨ����PWM���*/
	switch(Channel)
	{
		case 0: // H_HEAT1
			HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_3);
			HAL_TIM_PWM_Start_IT(&htim4,TIM_CHANNEL_3);
			break;
		case 1: // H_HEAT2
			HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2);
			HAL_TIM_PWM_Start_IT(&htim2,TIM_CHANNEL_2);
			break;
		case 2: // H_HEAT3
			HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1);
			HAL_TIM_PWM_Start_IT(&htim3,TIM_CHANNEL_1);
			break;
		case 3: // H_HEAT4
			HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2);
			HAL_TIM_PWM_Start_IT(&htim3,TIM_CHANNEL_2);
			break;
		case 4: // H_HEAT5
			HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1);
			HAL_TIM_PWM_Start_IT(&htim4,TIM_CHANNEL_1);
			break;
		case 5: // H_HEAT6
			HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_2);
			HAL_TIM_PWM_Start_IT(&htim4,TIM_CHANNEL_2);
			break;
		default:
			break;
	}

}

/************************ (C) COPYRIGHT Tobey *****END OF FILE****/		
