/**
  ******************************************************************************
  * File Name          : mxconstants.h
  * Description        : This file contains the common defines of the application
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
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MXCONSTANT_H
#define __MXCONSTANT_H
  /* Includes ------------------------------------------------------------------*/

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/

#define LED0_Pin GPIO_PIN_13
#define LED0_GPIO_Port GPIOC
#define LED1_Pin GPIO_PIN_14
#define LED1_GPIO_Port GPIOC
#define MOTO_DIR_Pin GPIO_PIN_15
#define MOTO_DIR_GPIO_Port GPIOC
#define MOTO_MS1_Pin GPIO_PIN_0
#define MOTO_MS1_GPIO_Port GPIOA
#define MOTO_MS2_Pin GPIO_PIN_1
#define MOTO_MS2_GPIO_Port GPIOA
#define MOTO_STEP_Pin GPIO_PIN_2
#define MOTO_STEP_GPIO_Port GPIOA
#define CAN_S_Pin GPIO_PIN_4
#define CAN_S_GPIO_Port GPIOA
#define IN0_Pin GPIO_PIN_5
#define IN0_GPIO_Port GPIOA
#define IN1_Pin GPIO_PIN_6
#define IN1_GPIO_Port GPIOA
#define nADSRST_Pin GPIO_PIN_10
#define nADSRST_GPIO_Port GPIOB
#define AD_START_Pin GPIO_PIN_11
#define AD_START_GPIO_Port GPIOB
#define AD_CS_Pin GPIO_PIN_12
#define AD_CS_GPIO_Port GPIOB
#define nAD_DRDY_Pin GPIO_PIN_8
#define nAD_DRDY_GPIO_Port GPIOA
#define H_PROTECT_Pin GPIO_PIN_9
#define H_PROTECT_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

/**
  * @}
  */ 

/**
  * @}
*/ 

#endif /* __MXCONSTANT_H */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
