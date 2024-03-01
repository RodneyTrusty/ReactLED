/*
 * my_spi.c
 *
 *  Created on: Oct 12, 2020
 *      Author: rtrusty
 */
#include <stdint.h>
#include "stm32f401xe.h"
#include "my_spi.h"
/*
**********SPI PRACTICE***********
1. Enable SPI Peripheral Clock
2. Enable GPIO Clock
3. Configure GPIO pins for SPI
4. Configure a pin for Slave Select
5. Configure SPI Control Register SPI_CR1
6. Configure SPI Control Register SPI CR_2
7. Enable SPI by setting SPE bit in SPI_CR1
8. Monitor TXE flag bit until transmit fata register is empty
9. Assert SS pin
10. Write to SPI_DR register
11. Wait til BSY flag is cleared in SPI_SR then deassert SS
12. Rinse and repeat
*/

void spi_init(Spi_Config* config){


/*Enable Peripheral Clock*/
if(config->spi_num == 2){
RCC->APB1ENR |= RCC_APB1ENR_SPI2EN;

}
if(config->spi_num == 3){
RCC->APB1ENR |= RCC_APB1ENR_SPI3EN;
}

/*Enable GPIO Clock*/
if(config->gpio_port == 'a' | config->gpio_port == 'A'){
RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
config->gpio_port = 'a';
}
if(config->gpio_port == 'b' | config->gpio_port == 'B'){
RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
config->gpio_port = 'b';
}
if(config->gpio_port == 'c' | config->gpio_port == 'C'){
RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;
config->gpio_port = 'c';
}


/**** Configure SPI GPIO Pins *****/
if(config->spi_num == 2 && config->gpio_port == 'b')
{

GPIOB->MODER  |= (GPIO_MODER_MODE13_1 | GPIO_MODER_MODE14_1 |  GPIO_MODER_MODE15_1);
GPIOB->AFR[1] |= (GPIO_AFRH_AFSEL13_0 | GPIO_AFRH_AFSEL13_2 | GPIO_AFRH_AFSEL14_0 | GPIO_AFRH_AFSEL14_2 | GPIO_AFRH_AFSEL15_0 | GPIO_AFRH_AFSEL15_2);

}


if(config->spi_num == 3 && config->gpio_port == 'b')
{

GPIOB->MODER  |= (GPIO_MODER_MODE3_1 | GPIO_MODER_MODE4_1 |  GPIO_MODER_MODE5_1);
GPIOB->AFR[0] |= (GPIO_AFRL_AFSEL3_1 | GPIO_AFRL_AFSEL3_2 | GPIO_AFRL_AFSEL4_1 | GPIO_AFRL_AFSEL4_2 | GPIO_AFRL_AFSEL5_1 | GPIO_AFRL_AFSEL5_2);

}

if(config->spi_num == 3 && config->gpio_port == 'c')
{

GPIOC->MODER  |= (GPIO_MODER_MODE10_1 | GPIO_MODER_MODE11_1 |  GPIO_MODER_MODE12_1);
GPIOC->AFR[1] |= (GPIO_AFRH_AFSEL10_1 | GPIO_AFRH_AFSEL10_2 | GPIO_AFRH_AFSEL11_1 | GPIO_AFRH_AFSEL11_2 | GPIO_AFRH_AFSEL12_1 | GPIO_AFRH_AFSEL12_2);

}


if(config->spi_num == 2){
SPI2->CR1 = (SPI_CR1_MSTR |(config->speed)| SPI_CR1_SSI | SPI_CR1_SSM | config->mode);
SPI2->CR2 = 0x0;
SPI2->CR1 |= SPI_CR1_SPE;
}

if(config->spi_num == 3){
SPI3->CR1 = (SPI_CR1_MSTR |(config->speed)| SPI_CR1_SSI | SPI_CR1_SSM | config->mode);
SPI3->CR2 = 0x0;
SPI3->CR1 |= SPI_CR1_SPE;
}

}


int spi_write(SPI_TypeDef* SPIx, uint8_t *buffer, uint16_t datalen)
{
    for(uint16_t i=0; i<datalen; i++)
    {
    	SPIx->DR = buffer[i];
      while (!(SPIx->SR & SPI_SR_TXE_Msk ));
    	SPIx->DR;
    }
    return 0;
} // end writetospi()


/*! ------------------------------------------------------------------------------------------------------------------
 * Function: readfromspi()
 *
 * Low level abstract function to read from the SPI
 * Takes two separate byte buffers for write header and read data
 * returns the offset into read buffer where first byte of read data may be found,
 * or returns -1 if there was an error
 */
//#pragma GCC optimize ("O3")
int spi_read(SPI_TypeDef* SPIx,uint8_t *buffer, uint32_t readlen)
{

    for(uint16_t i=0; i<readlen; i++)
    {
    	SPIx->DR = 0xFF;  // Dummy write as we read the message body

    	while (!(SPIx->SR & SPI_SR_RXNE_Msk )); //Wait til RXNE

	   	buffer[i] = SPIx->DR ;//port_SPIx_receive_data(); //this clears RXNE bit
    }

    return 0;
}






