#ifndef __TFT_CONF_H
#define __TFT_CONF_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

/*################################ LCD #######################################*/
/* Chip Select macro definition */
#define LCD_CS_LOW()       HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_RESET)
#define LCD_CS_HIGH()      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_SET)

/* Set WRX High to send data */
#define LCD_WRX_LOW()      HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_RESET)
#define LCD_WRX_HIGH()     HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_SET)

/* Set RDX High to send data */
#define LCD_RDX_LOW()      HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_RESET)
#define LCD_RDX_HIGH()     HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_SET)

#define SPIx_TIMEOUT_MAX              ((uint32_t)0x1000)
#define I2Cx_TIMEOUT_MAX                    0x3000 /*<! The value of the maximal timeout for I2C waiting loops */

#define LCD_FRAME_BUFFER       ((uint32_t)0xD0000000)
#define TFT_HOR_RES 			240
#define TFT_VER_RES 			320

#define TS_I2C_ADDRESS                      0x82


void     LCD_IO_Init(void);
void     LCD_IO_WriteData(uint16_t RegValue);
void     LCD_IO_WriteReg(uint8_t Reg);
uint32_t LCD_IO_ReadData(uint16_t RegValue, uint8_t ReadSize);
void     LCD_Delay (uint32_t delay);

void     IOE_Init(void);
void     IOE_ITConfig (void);
void     IOE_Delay(uint32_t delay);
void     IOE_Write(uint8_t addr, uint8_t reg, uint8_t value);
uint8_t  IOE_Read(uint8_t addr, uint8_t reg);
uint16_t IOE_ReadMultiple(uint8_t addr, uint8_t reg, uint8_t *buffer, uint16_t length);


void 		tft_init(void);
void 		touchpad_init(void);

#ifdef __cplusplus
}
#endif

#endif /* __TFT_CONF_H */
