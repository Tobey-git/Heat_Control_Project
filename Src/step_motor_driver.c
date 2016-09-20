/**
  ******************************************************************************
  * @file    step_motor_driver.c
  * @author  Tobey
  * @version V1.0.0
  * @date    2-August-2016
  * @brief   本文件是步进电机的驱动模块
  * 				 本文件提供下列功能用于对步进电机进行管理
	*						+ 设置步进电机步进细分数
	*						+ 设置步进电机旋转方向
	*						+ 设置步进电机步进数
  *  
  ******************************************************************************
  * @attention
	* 本驱动程序适用于使用stm32cubex生成的项目模板使用
	*
  *	根据用户连接步进电机所使用的端口、引脚、控制电机的定时器、步进时间间隔不同，用户需要修改宏定义部分的相关定义
	*
	********* 根据用户使用的引脚不同，需要修改相应的端口及引脚宏定义(step_motor_driver.h -->  Exported macro) ************
	*
	****步进电机每个引脚的对应端口
	* STEP_MOTOR_STEP_GPIO_PORT：步进电机STEP引脚端口
	* STEP_MOTOR_DIR_GPIO_PORT: 步进电机DIR引脚端口
	* STEP_MOTOR_MS1_GPIO_PORT: 步进电机MS1引脚端口
	* STEP_MOTOR_MS2_GPIO_PORT: 步进电机MS2引脚端口
	*
	****步进电机每个引脚的对应引脚号
	* STEP_MOTOR_STEP_GPIO_PIN：步进电机步进引脚
	* STEP_MOTOR_DIR_GPIO_PIN：步进电机旋转方向引脚
	* STEP_MOTOR_MS1_GPIO_PIN：MS1引脚
	* STEP_MOTOR_MS2_GPIO_PIN：MS2引脚
	*
	*
	********* 使用方式 *********
	*	本驱动通过修改步进电机结构体(STEP_MOTOR_InitTypeDef)中的步进电机数据(方向，步进细分数量值，步进数)实现对步进电机的控制
	* 步进电机的速度在定时器初始化时进行设定
	* 下面是使用本驱动的详细说明：
	*
	* 1、在stm32f1xx_it.c文件中对应的中断处理函数中添加下列中断处理代码
	*		需要引入step_motor.driver.h文件
	
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
	
	*	3、建立任务并在任务中初始化电机并启动定时器即可实现对步进电机的控制
					**
						* @brief  步进电机控制任务.
						* @param  None.
						* @note   两个静态变量分别用于控制电机步进控制与电机配置控制.
						*      @arg openTIM: 0表示中断未开启，1表示中断已开启
						*      @arg microStep: 0、1、2、3分别代表细分级数：FULL、HALF、QUARTER、EIGHTH
						*           每次切换细分级数时同时修改电机转动方向。
						×						 FULL   -> 200节拍(一圈)  (顺时针旋转)
						×						 HALF   -> 400节拍(一圈)  (逆时针旋转)
						×						 FOUTH  -> 800节拍(一圈)  (顺时针旋转)
						×						 EIGTH  -> 1600节拍(一圈) (逆时针旋转)
						* @retval None
						*
						
					__task void motorTask (void) {
						BSP_STEP_MOTOR_Init();			// 初始化电机
						static uint8_t openTIM = 1; // 默认初始化时Step为0，此时计时器中断应该为关闭状态，这里为了进行测试设置为1
						static uint8_t microStep = 0;	// 细分步数标志
						for (;;) {
							if(openTIM == 0 && STEP_MOTOR_Msg.Step > 0)	// 当Step>0且计时器处于关闭状态时开启计时器
							{
								HAL_TIM_Base_Start_IT(&htim1);
								openTIM=1;
							}
							if(openTIM == 1 && STEP_MOTOR_Msg.Step == 0)	// 当计时器处于开启状态且Step已为0时设置计时器标志位为1
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
STEP_MOTOR_InitTypeDef STEP_MOTOR_Msg;			// 电机结构体
TIM_HandleTypeDef STEP_MOTOR_Timx;		// 电机定时器句柄

/**
  * @brief  步进电机初始化.
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
  * @brief  设置步进电机细分数数量级.
  * @param  Step: 电机步距的细分值.
  *          传入参数可为下述值:
  *            @arg FULL: Full step
  *            @arg HALF: Half step
	*            @arg QUARTER:Quarter step
	*            @arg EIGHTH: Eighth step
  * @note   步进电机工作时细分数可以修改，当细分数修改时，.
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
  * @brief  设置步进电机旋转方向.
  * @param  Dir: 电机的旋转方向.
  *          传入参数可为下述值:
  *            @arg FORWARD：电机沿顺时针方向旋转					
	*            @arg REVERSAL：电机沿逆时针方向旋转
  * @note   步进电机工作时方向可以修改.
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
