/*
 * my_spi.h
 *
 *  Created on: Oct 12, 2020
 *      Author: rtrusty
 */

#ifndef INC_MY_SPI_H_
#define INC_MY_SPI_H_

#include <stdint.h>
#include "stm32f401xe.h"


typedef struct{
uint8_t spi_num;
char gpio_port;
uint8_t mode;
uint8_t speed;
}Spi_Config;



void spi_init(Spi_Config* config);
int spi_write(SPI_TypeDef* SPIx, uint8_t *buffer, uint16_t datalen);
int spi_read(SPI_TypeDef* SPIx,uint8_t *buffer, uint32_t readlen);



#endif /* INC_MY_SPI_H_ */
