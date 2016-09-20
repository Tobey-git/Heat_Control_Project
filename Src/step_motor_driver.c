/**
  ******************************************************************************
  * @file    step_motor_driver.c
  * @author  Tobey
  * @version V1.0.0
  * @date    2-August-2016
  * @brief   ���ļ��ǲ������������ģ��
  * 				 ���ļ��ṩ���й������ڶԲ���������й���
	*						+ ���ò����������ϸ����
	*						+ ���ò��������ת����
	*						+ ���ò������������
  *  
  ******************************************************************************
  * @attention
	* ����������������ʹ��stm32cubex���ɵ���Ŀģ��ʹ��
	*
  *	�����û����Ӳ��������ʹ�õĶ˿ڡ����š����Ƶ���Ķ�ʱ��������ʱ������ͬ���û���Ҫ�޸ĺ궨�岿�ֵ���ض���
	*
	********* �����û�ʹ�õ����Ų�ͬ����Ҫ�޸���Ӧ�Ķ˿ڼ����ź궨��(step_motor_driver.h -->  Exported macro) ************
	*
	****�������ÿ�����ŵĶ�Ӧ�˿�
	* STEP_MOTOR_STEP_GPIO_PORT���������STEP���Ŷ˿�
	* STEP_MOTOR_DIR_GPIO_PORT: �������DIR���Ŷ˿�
	* STEP_MOTOR_MS1_GPIO_PORT: �������MS1���Ŷ˿�
	* STEP_MOTOR_MS2_GPIO_PORT: �������MS2���Ŷ˿�
	*
	****�������ÿ�����ŵĶ�Ӧ���ź�
	* STEP_MOTOR_STEP_GPIO_PIN�����������������
	* STEP_MOTOR_DIR_GPIO_PIN�����������ת��������
	* STEP_MOTOR_MS1_GPIO_PIN��MS1����
	* STEP_MOTOR_MS2_GPIO_PIN��MS2����
	*
	*
	********* ʹ�÷�ʽ *********
	*	������ͨ���޸Ĳ�������ṹ��(STEP_MOTOR_InitTypeDef)�еĲ����������(���򣬲���ϸ������ֵ��������)ʵ�ֶԲ�������Ŀ���
	* ����������ٶ��ڶ�ʱ����ʼ��ʱ�����趨
	* ������ʹ�ñ���������ϸ˵����
	*
	* 1����stm32f1xx_it.c�ļ��ж�Ӧ���жϴ���������������жϴ������
	*		��Ҫ����step_motor.driver.h�ļ�
	
					static int CNT = (int)0;
					if(CNT == 1)
					{
						CNT = 0;
						HAL_GPIO_WritePin(STEP_MOTOR_STEP_GPIO_PORT, STEP_MOTOR_STEP_GPIO_PIN, GPIO_PIN_SET);
						STEP_MOTOR_Msg.Step--;
					}
					else 
					{
						HAL_GPIO_WritePin(STEP_MOTOR_STEP_GPIO_PORT, STEP_MOTOR_STEP_GPIO_PIN, GPIO_PIN_RESET);
						BSP_STEP_MOTOR_Dir_Set(STEP_MOTOR_Msg.Dir);
						BSP_STEP_MOTOR_MicroStep_Set(STEP_MOTOR_Msg.MicroStep);
						CNT++;
					}
					if(STEP_MOTOR_Msg.Step<=0)HAL_TIM_Base_Stop_IT(&htim1);
	
	*	3�����������������г�ʼ�������������ʱ������ʵ�ֶԲ�������Ŀ���
					**
						* @brief  ���������������.
						* @param  None.
						* @note   ������̬�����ֱ����ڿ��Ƶ�����������������ÿ���.
						*      @arg openTIM: 0��ʾ�ж�δ������1��ʾ�ж��ѿ���
						*      @arg microStep: 0��1��2��3�ֱ����ϸ�ּ�����FULL��HALF��QUARTER��EIGHTH
						*           ÿ���л�ϸ�ּ���ʱͬʱ�޸ĵ��ת������
						��						 FULL   -> 200����(һȦ)  (˳ʱ����ת)
						��						 HALF   -> 400����(һȦ)  (��ʱ����ת)
						��						 FOUTH  -> 800����(һȦ)  (˳ʱ����ת)
						��						 EIGTH  -> 1600����(һȦ) (��ʱ����ת)
						* @retval None
						*
						
					__task void motorTask (void) {
						BSP_STEP_MOTOR_Init();			// ��ʼ�����
						static uint8_t openTIM = 1; // Ĭ�ϳ�ʼ��ʱStepΪ0����ʱ��ʱ���ж�Ӧ��Ϊ�ر�״̬������Ϊ�˽��в�������Ϊ1
						static uint8_t microStep = 0;	// ϸ�ֲ�����־
						for (;;) {
							if(openTIM == 0 && STEP_MOTOR_Msg.Step > 0)	// ��Step>0�Ҽ�ʱ�����ڹر�״̬ʱ������ʱ��
							{
								HAL_TIM_Base_Start_IT(&htim1);
								openTIM=1;
							}
							if(openTIM == 1 && STEP_MOTOR_Msg.Step == 0)	// ����ʱ�����ڿ���״̬��Step��Ϊ0ʱ���ü�ʱ����־λΪ1
							{
								openTIM = 0;
								os_dly_wait(100);
								switch(microStep)
								{
									case 0: // FULL FORWARD
										STEP_MOTOR_Msg.Dir = FORWARD;
										STEP_MOTOR_Msg.MicroStep = FULL;
										STEP_MOTOR_Msg.Step = 200;
										microStep++;
										break;
									case 1:	// HALF REVERSAL
										STEP_MOTOR_Msg.Dir = REVERSAL;
										STEP_MOTOR_Msg.MicroStep = HALF;
										STEP_MOTOR_Msg.Step = 400;
										microStep++;
										break;
									case 2: // QUARTER FORWARD
										STEP_MOTOR_Msg.Dir = FORWARD;
										STEP_MOTOR_Msg.MicroStep = QUARTER;
										STEP_MOTOR_Msg.Step = 800;
										microStep++;
										break;
									case 3: // EIGHTH REVERSAL
										STEP_MOTOR_Msg.Dir = REVERSAL;
										STEP_MOTOR_Msg.MicroStep = EIGHTH;
										STEP_MOTOR_Msg.Step = 1600;
										microStep = 0;
										break;
									default:
										break;
								}
							}
							os_tsk_pass();
						}
					}
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "step_motor_driver.h"
#include "math.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
STEP_MOTOR_InitTypeDef STEP_MOTOR_Msg;			// ����ṹ��
TIM_HandleTypeDef STEP_MOTOR_Timx;		// �����ʱ�����

/**
  * @brief  ���������ʼ��.
  * @param  None.
  * @note   None.
  * @retval None
  */
void BSP_STEP_MOTOR_Init(void)
{
	STEP_MOTOR_Msg.Dir=FORWARD;;
	STEP_MOTOR_Msg.Step = 0;
	STEP_MOTOR_Msg.MicroStep = FULL;
	HAL_GPIO_WritePin(STEP_MOTOR_STEP_GPIO_PORT, STEP_MOTOR_STEP_GPIO_PIN, GPIO_PIN_RESET);
	BSP_STEP_MOTOR_Dir_Set(STEP_MOTOR_Msg.Dir);
	BSP_STEP_MOTOR_MicroStep_Set(STEP_MOTOR_Msg.MicroStep);
}

/**
  * @brief  ���ò������ϸ����������.
  * @param  Step: ��������ϸ��ֵ.
  *          ���������Ϊ����ֵ:
  *            @arg FULL: Full step
  *            @arg HALF: Half step
	*            @arg QUARTER:Quarter step
	*            @arg EIGHTH: Eighth step
  * @note   �����������ʱϸ���������޸ģ���ϸ�����޸�ʱ��.
  * @retval None
  */
void BSP_STEP_MOTOR_MicroStep_Set(STEP_MOTOR_MicroStep_Def Step)
{
	
	switch(Step)
	{
		case FULL:
			HAL_GPIO_WritePin(STEP_MOTOR_MS1_GPIO_PORT,STEP_MOTOR_MS1_GPIO_PIN, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(STEP_MOTOR_MS2_GPIO_PORT,STEP_MOTOR_MS2_GPIO_PIN, GPIO_PIN_RESET);
			break;
		case HALF:
			HAL_GPIO_WritePin(STEP_MOTOR_MS1_GPIO_PORT,STEP_MOTOR_MS1_GPIO_PIN, GPIO_PIN_SET);
			HAL_GPIO_WritePin(STEP_MOTOR_MS2_GPIO_PORT,STEP_MOTOR_MS2_GPIO_PIN, GPIO_PIN_RESET);
			break;
	  case QUARTER:
			HAL_GPIO_WritePin(STEP_MOTOR_MS1_GPIO_PORT,STEP_MOTOR_MS1_GPIO_PIN, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(STEP_MOTOR_MS2_GPIO_PORT,STEP_MOTOR_MS2_GPIO_PIN, GPIO_PIN_SET);
			break;
	  case EIGHTH:
			HAL_GPIO_WritePin(STEP_MOTOR_MS1_GPIO_PORT,STEP_MOTOR_MS1_GPIO_PIN, GPIO_PIN_SET);
			HAL_GPIO_WritePin(STEP_MOTOR_MS2_GPIO_PORT,STEP_MOTOR_MS2_GPIO_PIN, GPIO_PIN_SET);
			break;
		default:
			break;
	}
}

/**
  * @brief  ���ò��������ת����.
  * @param  Dir: �������ת����.
  *          ���������Ϊ����ֵ:
  *            @arg FORWARD�������˳ʱ�뷽����ת					
	*            @arg REVERSAL���������ʱ�뷽����ת
  * @note   �����������ʱ��������޸�.
  * @retval None
  */
void BSP_STEP_MOTOR_Dir_Set(STEP_MOTOR_Dir_Def Dir)
{
	if(Dir == FORWARD)
	{
		HAL_GPIO_WritePin(STEP_MOTOR_DIR_GPIO_PORT,STEP_MOTOR_DIR_GPIO_PIN,GPIO_PIN_SET);
	}
	else
	{
		HAL_GPIO_WritePin(STEP_MOTOR_DIR_GPIO_PORT,STEP_MOTOR_DIR_GPIO_PIN,GPIO_PIN_RESET);
	}
	
}




/************************ (C) COPYRIGHT Tobey *****END OF FILE****/		
