/*
 * I2C.h
 *
 *  Created on: Jul 18, 2020
 *      Author: perSec
 */

#ifndef __STM32F4xx_LL_I2C_H
#include "stm32f4xx_ll_i2c.h"
#endif

#ifndef __STM32F4xx_LL_DMA_H
#include "stm32f4xx_ll_dma.h"
#endif


#ifndef INC_I2C_H_
#define INC_I2C_H_

typedef struct{
	I2C_TypeDef * I2C;
	uint8_t i2c_busy;
	uint8_t i2c_IsRx;
}I2C_struct;


void I2C_init(I2C_struct* I2C,I2C_TypeDef* I2C_type);

void I2C_Transmit(I2C_struct* I2C, uint8_t address, uint8_t* data,uint16_t size);

void I2C_Receive(I2C_struct* I2C, uint8_t address, uint8_t* outputdata,uint16_t size);

#endif /* INC_I2C_H_ */
