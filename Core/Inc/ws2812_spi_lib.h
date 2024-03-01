/*
 * ws2812_spi_lib.h
 *
 *  Created on: Jun 30, 2020
 *      Author: rtrusty
 */

#ifndef INC_WS2812_SPI_LIB_H_
#define INC_WS2812_SPI_LIB_H_

#include "stm32f4xx_hal.h"
#include "spi.h"
#include "main.h"
#include <stdint.h>

#define NUM_PIXELS			16
//#define NUM_PIXELS			138

struct Led_pixels{

	uint8_t green[3];
	uint8_t red[3];
	uint8_t blue[3];

};

struct Bar{
	uint8_t first_led;
	uint8_t last_led;
};


extern struct Led_pixels pixels[];



void set_pixel_color(struct Led_pixels * pixels, uint32_t pixel_num, uint8_t red, uint8_t green, uint8_t blue);

void place_bar(struct Led_pixels * pixels, struct Bar * bar, struct Color * color, uint8_t brightness, uint8_t range, uint8_t size);

void set_bar(struct Led_pixels * pixels, struct Bar * bar, struct Color * color, uint8_t brightness, uint8_t range);

int map(int x, int in_min, int in_max, int out_min, int out_max);


void byte_to_color(uint8_t byte, uint8_t * color);

uint8_t show_pixels(struct Led_pixels * pixels);

uint8_t clear_pixels(struct Led_pixels * pixels);





#endif /* INC_WS2812_SPI_LIB_H_ */
