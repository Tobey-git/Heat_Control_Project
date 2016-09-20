/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __ADS1248_H
#define __ADS1248_H
#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"
#include "spi.h"
#include <RTL.h>
/* Exported constants --------------------------------------------------------*/	 
//ADS1248寄存器列表  
#define ADC_REG_MUX0        0x00  
#define ADC_REG_VBIAS       0x01  
#define ADC_REG_MUX1        0x02  
#define ADC_REG_SYS0        0x03  
#define ADC_REG_OFC0        0x04  
#define ADC_REG_OFC1        0x05  
#define ADC_REG_OFC2        0x06  
#define ADC_REG_FSC0        0x07  
#define ADC_REG_FSC1        0x08  
#define ADC_REG_FSC2        0x09  
#define ADC_REG_IDAC0       0x0A  
#define ADC_REG_IDAC1       0x0B 
#define ADC_REG_GPIOCFG     0x0C
#define ADC_REG_GPIODIR     0x0D
#define ADC_REG_GPIODAT     0x0E


//ADS1248支持的增益列表  
#define ADC_GAIN_1          0x00  
#define ADC_GAIN_2          0x10  
#define ADC_GAIN_4          0x20  
#define ADC_GAIN_8          0x30  
#define ADC_GAIN_16         0x40  
#define ADC_GAIN_32         0x50  
#define ADC_GAIN_64         0x60  
#define ADC_GAIN_128        0x70  
  
//ADS1248支持的转换速率列表  
#define ADC_SPS_5           0x00  
#define ADC_SPS_10          0x01  
#define ADC_SPS_20          0x02  
#define ADC_SPS_40          0x03  
#define ADC_SPS_80          0x04  
#define ADC_SPS_160         0x05  
#define ADC_SPS_320         0x06  
#define ADC_SPS_640         0x07  
#define ADC_SPS_1000        0x08  
#define ADC_SPS_2000        0x09  
  
//ADS1248转换模式设置  
#define ADC_MODE_SINGLECOV      0x00        /*!< 单次转换模式   */ 
#define ADC_MODE_CONTINUOUS     0x01        /*!< 连续转换模式   */  



//ADS1248命令码列表  
#define ADC_CMD_WAKEUP      0x00            /*!< 退出睡眠模式   */    
#define ADC_CMD_SLEEP       0x02            /*!< 进入睡眠模式   */  
#define ADC_CMD_SYNC        0x04            /*!< 同步ADC转换   */  
#define ADC_CMD_RESET       0x06            /*!< 芯片复位   */  
#define ADC_CMD_NOP         0xFF            /*!< 空操作   */  
#define ADC_CMD_RDATA       0x12            /*!< 单次读取数据   */  
#define ADC_CMD_RDATAC      0x14            /*!< 连续读取数据   */  
#define ADC_CMD_SDATAC      0x16            /*!< 停止连续读取   */  
#define ADC_CMD_RREG        0x20            /*!< 读寄存器   */  
#define ADC_CMD_WREG        0x40            /*!< 写寄存器   */  
#define ADC_CMD_SYSOCAL     0x60            /*!< 系统偏移校准   */  
#define ADC_CMD_SYSGCAL     0x61            /*!< 系统增益校准   */  
#define ADC_CMD_SELFOCAL    0x62            /*!< 系统自校准   */  
#define ADC_CMD_RESTRICTED  0xF1            // 

/* Definition for ADS1248 */
#define ADS1248_RESET_GPIO_PORT		GPIOB									/*!< RESET PORT */
#define ADS1248_RESET_GPIO_PIN		GPIO_PIN_10						/*!< RESET PIN */

#define ADS1248_START_GPIO_PORT		GPIOB									/*!< START PORT */
#define ADS1248_START_GPIO_PIN		GPIO_PIN_11						/*!< START PIN */

#define ADS1248_DIN_GPIO_PORT		GPIOB										/*!< DIN PORT */
#define ADS1248_DIN_GPIO_PIN		GPIO_PIN_15							/*!< DIN PIN */

#define ADS1248_DOUT_GPIO_PORT		GPIOB									/*!< DOUT PORT */
#define ADS1248_DOUT_GPIO_PIN		GPIO_PIN_14							/*!< DOUT PIN */

#define ADS1248_SCLK_GPIO_PORT		GPIOB									/*!< SCLK PORT */
#define ADS1248_SCLK_GPIO_PIN		GPIO_PIN_13							/*!< SCLK PIN */

#define ADS1248_CS_GPIO_PORT		GPIOB										/*!< CS PORT */
#define ADS1248_CS_GPIO_PIN		GPIO_PIN_12								/*!< CS PIN */

#define ADS1248_DRDY_GPIO_PORT		GPIOA									/*!< DRDY PORT */
#define ADS1248_DRDY_GPIO_PIN		GPIO_PIN_8							/*!< DRDY PIN */

#define ADS1248_SPI		hspi2                             /*!< SPI */

/* Exported macro ------------------------------------------------------------*/
#define ADC_SPI_CS_CLR  HAL_GPIO_WritePin(ADS1248_CS_GPIO_PORT,ADS1248_CS_GPIO_PIN,GPIO_PIN_RESET);
#define ADC_SPI_CS_SET  HAL_GPIO_WritePin(ADS1248_CS_GPIO_PORT,ADS1248_CS_GPIO_PIN,GPIO_PIN_SET);

#define ADC_START_SET   HAL_GPIO_WritePin(ADS1248_START_GPIO_PORT,ADS1248_START_GPIO_PIN,GPIO_PIN_SET);
#define ADC_START_CLR   HAL_GPIO_WritePin(ADS1248_START_GPIO_PORT,ADS1248_START_GPIO_PIN,GPIO_PIN_RESET);

/* ADS1248 FUNCTION */
void 		Delay(void);
void 		ADS1248_Reset(void);	
void    ADS1248_Start(uint8_t CovMode);
void 		ADS1248_Stop(void);
void 		ADS1248_WriteCmd(uint8_t Cmd);
void 		ADS1248_ReadReg(uint8_t RegAddr,uint8_t *Buffer,uint8_t Length);
void 		ADS1248_WriteReg(uint8_t RegAddr,uint8_t *Buffer,uint8_t Length);
uint8_t ADS1248_WaitBusy(void);
void    ADS1248_Config(uint8_t CovGain,uint8_t CovRate);
int32_t ADS1248_Read(void);
void Change_Channel(uint8_t Ch);
uint8_t ADS1248_Calibrate(uint8_t Gain);
uint8_t ADS1248_Init(uint8_t Gain);
int32_t ADS1248_Channel_Data(uint8_t Ch);
float ADS1248_AdToTemp(int32_t Ad);


#ifdef __cplusplus
}
#endif
#endif /*__ spi_H */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
