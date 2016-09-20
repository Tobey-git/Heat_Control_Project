/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  *
  * COPYRIGHT(c) 2016 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */
#include <RTL.h>
#include "ADS1248.h"
#include "step_motor_driver.h"
#include "pid_driver.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
OS_TID t_adc;                        /* assigned task id of task: adcTask */
OS_TID t_printf;                        /* assigned task id of task: printfTask */
OS_TID t_motor;                        /* assigned task id of task: motorTask */
OS_TID t_test;                        /* assigned task id of task: testTask */
volatile uint32_t chanelDataArray[7];												/* AD���� */
volatile float chanelTempArray[7]; 				/* ��ͨ���¶����� */
const float chanelDrift[7] = { 0.6, 0.25, -0.3, -0.35, 0, 0, -0.3 };
static PID PID_Chanel[7];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/**
  * @brief  ����������Ʋ�������.
  * @param  None.
  * @note   ������̬�����ֱ����ڿ��Ƶ�����������������ÿ���.
	*      @arg openTIM: 0��ʾ�ж�δ������1��ʾ�ж��ѿ���
	*      @arg microStep: 0��1��2��3�ֱ����ϸ�ּ�����FULL��HALF��QUARTER��EIGHTH
	*           ÿ���л�ϸ�ּ���ʱͬʱ�޸ĵ��ת������
	*						 FULL   -> 200����(һȦ)  (˳ʱ����ת)
	*						 HALF   -> 400����(һȦ)  (��ʱ����ת)
	*						 FOUTH  -> 800����(һȦ)  (˳ʱ����ת)
	*						 EIGTH  -> 1600����(һȦ) (��ʱ����ת)
  * @retval None
  */
__task void motorTask (void) {
	BSP_STEP_MOTOR_Init();			// ��ʼ�����
	uint8_t openTIM = 1; // Ĭ�ϳ�ʼ��ʱStepΪ0����ʱ��ʱ���ж�Ӧ��Ϊ�ر�״̬������Ϊ�˽��в�������Ϊ1
	uint8_t microStep = 0;	// ϸ�ֲ�����־
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
		os_tsk_pass();//�����ж��������ִ����ɺ󽻳�CPU����Ȩ
  }
}

/**
  * @brief  ��������.
  * @param  None.
  * @note   ����LED1ÿ����˸һ��.
  * @retval None
  */
__task void testTask (void) {
  for (;;) {
		HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_13);
		os_dly_wait(1000);
  }
}

/**
  * @brief  ��ӡ�������ڴ�ӡ��������.
  * @param  None.
  * @note   ��ǰ����ӡADֵ�������Ӧ���¶�ֵ
  * @retval None
  */
__task void printfTask (void) {
	
  for (;;) {
		
		int i;
		float Temp;
		for(i=0; i<7; i++)
		{
			// R = Ad/Full/Gain*2*Rref+R2
//			R = chanelDataArray[i]/0x7fffff/8*2*810+180;  
			Temp = ADS1248_AdToTemp(chanelDataArray[i]);//ֱ�Ӹ���ADֵ���õ��¶�ֵ
			printf("%5.2d ,  %4.2f  ;", chanelDataArray[i], Temp);
			
		}
		printf("\n");
		os_dly_wait(100);
  }
}

/**
  * @brief  ADC��PID��������.
  * @param  None.
  * @note   ѭ����ȡ7��ADͨ����ADֵ������ADֵ�洢��������
	* 				����ת���õ����¶�ֵ����PID����
	* 				chanel:ͨ�����
	* 				PIDSet[7]:��ʶ��ͨ��PID����ʱ����
	*					chanelDataArray[7]���洢��ͨ��ADֵ
	*					chanelTempArray[7]���洢��ͨ��ADֵ��Ӧ���¶�ֵ
  * @retval None
  */
__task void adcTask (void) {
	int i;
	for(i=0; i<7; i++)//PID��ʼ��
	{
		PID_Init(&PID_Chanel[i]);
		//PID���ã����ø�ͨ��Kp,Ki,Kdֵ��SetPointֵӦ���������¶�ʱ�������ã��˴����ڲ��Զ����������
		PID_SetPoint(&PID_Chanel[i], 40); //���� PID ���ڵ�Ŀ��ֵ
		PID_SetKp(&PID_Chanel[i], 2); //���� PID �� Kp ֵ
		PID_SetKi(&PID_Chanel[i], 0); //���� PID �� Ki ֵ
		PID_SetKd(&PID_Chanel[i], 0); //���� PID �� Kd ֵ
	}
	
	os_dly_wait(17);
	ADS1248_Init(ADC_GAIN_16|ADC_SPS_20);             //��ʼ�� AD1248
	int8_t chanel = 0;
	int16_t PIDSet[7] = { 0, 0, 0, 0, 0, 0, 0 };
  for (;;) {
	int32_t Data; 

	Data = ADS1248_Channel_Data(chanel);
	chanelTempArray[chanel] = ADS1248_AdToTemp(Data);
	
	if(PIDSet[chanel] == 30) // ���ָ��ʱ�����һ��PID����
	{
		/*�����ǰ�¶����趨ֵ���50�ȣ������PID���Ʒ�Χ*/
		if( (PID_Chanel[chanel].SetPoint - chanelTempArray[chanel] < 50 && PID_Chanel[chanel].SetPoint - chanelTempArray[chanel] > 0)
				|| (PID_Chanel[chanel].SetPoint - chanelTempArray[chanel] > -50 && PID_Chanel[chanel].SetPoint - chanelTempArray[chanel] < 0))
		{
			uint8_t pulse = PID_IncCalc(&PID_Chanel[chanel],chanelTempArray[chanel]);//����PID����ģʽ
			printf("puse%d: %d,  temp: %f\n",chanel, pulse, chanelTempArray[chanel]);
			PID_IncPWMCtr(chanel, pulse);//����ʽPID��PWM����
		}
		PIDSet[chanel] = 0;
		
	}
	
	chanelDataArray[chanel] = Data;
	PIDSet[chanel]++;
	chanel++;
	if(chanel==7)
	{
		chanel=0;
	};	
	os_dly_wait(10);
	
  }
}

/**
  * @brief  ��ʼ������.
  * @param  None.
  * @note   ���ڴ������񣬵����񶼴������ʱ����ʹ�������������.
  * @retval None
  */
__task void initTask (void) {
	t_adc = os_tsk_create(adcTask,1);
	t_printf = os_tsk_create(printfTask,1);
	t_motor = os_tsk_create(motorTask,1);
	t_test = os_tsk_create(testTask,1);
	os_tsk_delete_self();
}
/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_SPI2_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_USART1_UART_Init();

  /* USER CODE BEGIN 2 */

	HAL_TIM_PWM_Start_IT(&htim2,TIM_CHANNEL_2);

	HAL_TIM_PWM_Start_IT(&htim3,TIM_CHANNEL_1);		
	HAL_TIM_PWM_Start_IT(&htim3,TIM_CHANNEL_2);		
	
	HAL_TIM_PWM_Start_IT(&htim4,TIM_CHANNEL_1);	
	HAL_TIM_PWM_Start_IT(&htim4,TIM_CHANNEL_2);	
	HAL_TIM_PWM_Start_IT(&htim4,TIM_CHANNEL_3);	

	// ��ʼ��RTX�ں˲�������ʼ�����񣬴˺�Ӧ�ó����������м���ִ��
//	os_sys_init(initTask);
	os_sys_init_prio(initTask,3);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }

  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* USER CODE BEGIN 4 */
int fputc(int ch,FILE *f)
{
    uint8_t temp[1]={ch};
    HAL_UART_Transmit(&huart1,temp,1,2);
		return ch;
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
