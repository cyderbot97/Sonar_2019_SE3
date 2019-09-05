/*
 * i2c.h
 *
 *  Created on: 4 sept. 2019
 *      Author: cyril
 */

#ifndef BSP_INC_I2C_H_
#define BSP_INC_I2C_H_

#include "stm32f0xx.h"

uint8_t	BSP_OLED_SEND_CMD		( uint8_t command );
uint8_t	BSP_OLED_SEND_CHAR		( uint8_t command );
void BSP_I2C1_Init     			(void);
uint8_t	BSP_I2C1_Read			( uint8_t device_address, uint8_t register_address, uint8_t *buffer, uint8_t nbytes );


#endif /* BSP_INC_I2C_H_ */
