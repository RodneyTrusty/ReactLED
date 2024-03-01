/*
 * ws2812_spi_lib.c
 *
 *  Created on: Jun 30, 2020
 *      Author: rtrusty
 */

#include "ws2812_spi_lib.h"



void byte_to_color(uint8_t byte, uint8_t * color){

	uint32_t holder = 0;
	//Cycle through bits in byte
	//Store them in variable then store in color array
	for(uint8_t x = 0; x < 8; x++){

		if(((byte >> x) & 0x01)){

			//if bit == 1 set accordingly in holder variable
			//0x6 is a logic 1 for 3MHz Spi transfer to WS2812
			holder |= 0x06 << (x * 3);

		}
		else{

			//0x04 is logic 0

			holder |= 0x04 << (x * 3);

		}

	}

	//Store holder in color array
	color[0] = (holder >> 16) & 0xFF;
	color[1] = (holder >> 8) & 0xFF;
	color[2] = holder & 0xFF;

};


void set_pixel_color(struct Led_pixels * pixels, uint32_t pixel_num, uint8_t red, uint8_t green, uint8_t blue){


	byte_to_color(red, pixels[pixel_num].red);
	byte_to_color(green, pixels[pixel_num].green);
	byte_to_color(blue, pixels[pixel_num].blue);



};

uint8_t show_pixels(struct Led_pixels * pixels){


	  //return HAL_SPI_Transmit(&hspi1, pixels, (NUM_PIXELS * 9), 1000);
	  return HAL_SPI_Transmit_DMA(&hspi1, pixels, (NUM_PIXELS * 9));

};

uint8_t clear_pixels(struct Led_pixels * pixels){


	for(int x = 0; x < NUM_PIXELS; x++){

		set_pixel_color(pixels, x, 0, 0, 0);

	}

	  return 0x0;

};



int map(int x, int in_min, int in_max, int out_min, int out_max){

  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;

}




//Sets defined bar to a mapped 0-100 value
void set_bar(struct Led_pixels * pixels, struct Bar * bar, struct Color * color, uint8_t brightness, uint8_t range){

	if(range > 100){

		range = 100;

	}

	if(brightness > 100){

		brightness = 100;

	}

	color->blue  *= (brightness/100);
	color->green *= (brightness/100);
	color->red   *= (brightness/100);

	uint8_t output_range = fabs(bar->first_led - bar->last_led);

	int height = map(range, 0, 100, 0, output_range);


	if(bar->first_led > bar->last_led){

		for(int x = bar->first_led; x >= (bar->first_led - height); x--){

			set_pixel_color(pixels, x, color->red, color->green, color->blue);


		};

	}

	else{

		for(int x = bar->first_led; x <= (bar->first_led + height); x++){

			set_pixel_color(pixels, x, color->red, color->green, color->blue);


		};

	}

}




//Sets defined bar to a mapped 0-100 value
void place_bar(struct Led_pixels * pixels, struct Bar * bar, struct Color * color, uint8_t brightness, uint8_t range, uint8_t size){

	if(range > 100){

		range = 100;

	}

	if(brightness > 100){

		brightness = 100;

	}

	color->blue  *= (brightness/100);
	color->green *= (brightness/100);
	color->red   *= (brightness/100);

	uint8_t output_range = fabs(bar->first_led - bar->last_led);

	int height = map(range, 0, 100, 0, output_range);


	if(bar->first_led > bar->last_led){

		uint8_t start_led = bar->first_led - height;

		for(int x = start_led; x >= bar->last_led && x > start_led - size; x--){

			set_pixel_color(pixels, x, color->red, color->green, color->blue);


		};

	}

	else{

		uint8_t first_led = bar->first_led + height;


		for(int x = first_led; x <= bar->last_led && x < first_led + size; x++){

			set_pixel_color(pixels, x, color->red, color->green, color->blue);


		};

	}

}
