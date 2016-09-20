/**
  ******************************************************************************
  * @file    pid_driver.c
  * @author  Tobey
  * @version V1.0.0
  * @date    31-August-2016
  * @brief   本文件是PID控制的驱动模块
  * 				 本文件提供下列功能用于对PID进行管理
	*						+ 初始化PID参数
	*						+ 配置PID
	*						+ 获取PID计算结果
  ******************************************************************************
  * @attention
	*
	*
	********* 使用方式 *********
	* 1、在源文件中引入驱动头文件
	*		#include "pid_driver.h"
	* 2、初始化pid并配置pid参数：
	*   PID_Init(*sptr); //初始化PID（对PID所用到的所有参数清零）
	*		PID_SetPoint(*sptr, SetPoint); //设置 PID 调节的目标值
	*   PID_SetKp(*sptr, Kp); //设置 PID 的 Kp 值
	*   PID_SetKi(*sptr, Ki); //设置 PID 的 Ki 值
	*   PID_SetKd(*sptr, Kd); //设置 PID 的 Kd 值
	*	3、根据采用的控制方法不同获取对应的PID计算结果：
	* 	Inc = PID_IncCalc(*sptr, NextPoint): 读取指定PID增量式运算结果
	* 	Loc = PID_LocCalc(*sptr, NextPoint): 读取指定PID位置式运算结果
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
  * @brief  PID 参数初始化.
  * @param  *sptr：PID结构体指针.
  * @note   对PID所用到的所有参数清零.
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
  * @brief  设置 PID 调节的目标值.
  * @param  *sptr：PID结构体指针.
	* @param  Point：PID目标值.
  * @note   None.
  * @retval None
  */
void PID_SetPoint(PID *sptr, int SetPoint)
{
	sptr->SetPoint = SetPoint;
}

/**
  * @brief  读取 PID 调节设置的目标值.
  * @param  *sptr：PID结构体指针.
  * @note   None.
  * @retval PID目标值
  */
int PID_GetSetpoint(PID *sptr)
{
	return sptr->SetPoint;
}

/**
  * @brief  设置 PID 的 Kp 值.
  * @param  *sptr：PID结构体指针.
	* @param  Kp：Kp值.
  * @note   这个参数在增量 PID 和位置 PID 的计算中代表着不同的意思.
	*							位置式 PID 中 Kp 就是比例系数
	*							增量式 PID 中 Kp 相当于 ek 的系数 Kp(1+T/Ti+Td/T）
  * @retval None
  */
void PID_SetKp(PID *sptr, double Kp)
{
	sptr->Proportion = Kp;
}

/**
  * @brief  读取 PID 中所设置的 Kp 值.
  * @param  *sptr：PID结构体指针.
  * @note   返回值在增量 PID 和位置 PID 的计算中代表着不同的意思.
	*								位置式 PID 中 Kp 就是比例系数
	*								增量式 PID 中 Kp 相当于 ek 的系数 Kp(1+T/Ti+Td/T）.
  * @retval Kp
  */
double PID_GetKp(PID *sptr)
{
	return sptr->Proportion;
}

/**
  * @brief  设置 PID 的 Ki 值.
  * @param  *sptr：PID结构体指针.
	* @param  dKii：Ki.
  * @note   这个参数在增量 PID 和位置 PID 的计算中代表着不同的意思.
	*								位置式 PID 中 Ki 是积分系数Kp*T/Ti
	*								增量式 PID 中 Ki 相当于 ek-1 的系数 Kp(1+2*Td/T）.
  * @retval None
  */
void PID_SetKi(PID *sptr, double dKii)
{
	sptr->Integral = dKii;
}

/**
  * @brief  读取 PID 中所设置的 Ki 值.
  * @param  *sptr：PID结构体指针.
  * @note   返回值在增量 PID 和位置 PID 的计算中代表着不同的意思.
	*								位置式 PID 中 Ki 是积分系数Kp*T/Ti
	*								增量式 PID 中 Ki 相当于 ek-1 的系数 Kp(1+2*Td/T）.
  * @retval Ki
  */
double PID_GetKi(PID *sptr)
{
	return sptr->Integral;
}

/**
  * @brief  设置 PID 的 Kd 值.
  * @param  *sptr：PID结构体指针.
	* @param  dKdd：Kd.
  * @note   这个参数在增量 PID 和位置 PID 的计算中代表着不同的意思.
	*								位置式 PID 中 Kd 是积分系数Kp*Td/T
	*								增量式 PID 中 Kd 相当于 ek-2 的系数 Kp*Td/T.
  * @retval None
  */
void PID_SetKd(PID *sptr, double dKdd)
{
	sptr->Derivative = dKdd;
}

/**
  * @brief  读取 PID 中所设置的 Kd 值.
  * @param  *sptr：PID结构体指针.
  * @note   返回值在增量 PID 和位置 PID 的计算中代表着不同的意思.
	*								位置式 PID 中 Kd 是积分系数Kp*Td/T
	*								增量式 PID 中 Kd 相当于 ek-2 的系数 Kp*Td/T.
  * @retval Kd
  */
double PID_GetKd(PID *sptr)
{
	return sptr->Derivative;
}

/**
  * @brief  增量式 PID 控制设计.
  * @param  *sptr：PID结构体指针.
  * @param  NextPoint：PID传入参数.
  * @note   None.
  * @retval iIncpid：PID增量值
  */
int PID_IncCalc(PID *sptr, int NextPoint)
{
	register int iError, iIncpid;
	 //当前误差
	iError = sptr->SetPoint - NextPoint;
	 //增量计算
	iIncpid = sptr->Proportion * iError //E[k] 项
		 - sptr->Integral * sptr->LastError //E[k－1] 项
		 + sptr->Derivative * sptr->PrevError; //E[k－2] 项
	 //存储误差，用于下次计算
	sptr->PrevError = sptr->LastError;
	sptr->LastError = iError;
	 //返回增量值
	return(iIncpid);
}

/**
  * @brief  位置式 PID 控制设计.
  * @param  *sptr：PID结构体指针.
  * @param  NextPoint：PID传入参数.
  * @note   None.
  * @retval PID位置式计算结果
  */
unsigned int PID_LocCalc(PID *sptr, int NextPoint)
{
	register int iError,dError;
	iError = sptr->SetPoint - NextPoint; //偏差
	sptr->SumError += iError;   //积分
	dError = iError - sptr->LastError; //微分
	sptr->LastError = iError;
	return(sptr->Proportion * iError //比例项
		 + sptr->Integral * sptr->SumError //积分项
		 + sptr->Derivative * dError); //微分项
}

/**
  * @brief  位置式PID的PWM占空比控制.
  * @param  Channel：通道编号.
  * @param  pulse：占空比.
  * @note   None.
  * @retval None
  */
void PID_LocPWMCtr(uint8_t Channel, uint8_t pulse)
{
	// check PWM status
	/*用于判断PWM变化情况，若当前所需占空比与上次设置的值相同，则直接返回*/
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
	/*配置占空比Pulse*/
	TIM_OC_InitTypeDef sConfigOC;
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = pulse;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_LOW;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	/*启动制定定时器对应通道的PWM输出*/
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
  * @brief  增量式PID的PWM占空比控制.
  * @param  Channel：通道编号.
  * @param  Inc：增量值.
  * @note   None.
  * @retval None
  */
void PID_IncPWMCtr(uint8_t Channel, uint8_t Inc)
{
	// check PWM status
	/*用于判断PWM变化情况，若当前所需占空比与上次设置的值相同，则直接返回*/
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
	/*配置占空比Pulse*/
	TIM_OC_InitTypeDef sConfigOC;
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = pre_pulse;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_LOW;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	/*启动制定定时器对应通道的PWM输出*/
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
