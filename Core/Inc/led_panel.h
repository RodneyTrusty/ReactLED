/*
 * led_panel.h
 *
 *  Created on: Oct 11, 2020
 *      Author: rtrusty
 */

#ifndef INC_LED_PANEL_H_
#define INC_LED_PANEL_H_
#include "stm32f4xx_hal.h"
#include "main.h"
#include "adc.h"
#include "gpio.h"
#include "ws2812_spi_lib.h"

#define TXRX_TIMEOUT		100
#define INT_WAIT_TIMEOUT	100

#define HEADER_SIZE		 	4
#define RESP_OK				0
#define RESP_ERR			4
#define RESP_CODE			0xBF

#define COMMS_DELAY				10

#define PANEL_PIXEL_NUM		16

#define FADE_TIME  700
/*
 * 0: Command
 * 1: Sender's address
 * 2:Payload MSB
 * 3:Payload LSB
 *
 * POS payload
 * 0:num panels
 * 1:x
 * 1:y
 * */

//Header defines
#define HEADER_CMD			0
#define HEADER_TX_ADDY		1
#define PAYLOAD_HIGH		2
#define PAYLOAD_LOW			3

//Get pos defines
#define PANEL_GET_POS_CMD	 	0x0A
#define PANEL_SET_PIXEL_CMD	 	0x0B
#define PANEL_SET_AUTO_CMD		0x0C
#define PANEL_GET_PIX_CMD	 	0x0D

#define PANEL_UPSTREAM			0x01
#define PANEL_DOWNSTREAM		0x00

#define PANEL_CNT	0
#define TABLE_X		1
#define TABLE_Y		2

//Panel interrupt sources
#define CS_LEFT		0x01
#define CS_RIGHT	0x02
#define CS_DOWN		0x03
#define CS_UP		0x04
#define LAST_PANEL 	0x05

#define UNSELECT_LEFT()			GPIOB->ODR |= LEFT_CS_Pin;
#define UNSELECT_RIGHT()		GPIOB->ODR |= RIGHT_CS_Pin
#define UNSELECT_UP()			GPIOA->ODR |= UP_CS_Pin;
#define UNSELECT_DOWN()			GPIOB->ODR |= DOWN_CS_Pin;

#define SELECT_LEFT()			GPIOB->ODR &= ~LEFT_CS_Pin;
#define SELECT_RIGHT()			GPIOB->ODR &= ~RIGHT_CS_Pin
#define SELECT_UP()				GPIOA->ODR &= ~UP_CS_Pin;
#define SELECT_DOWN()			GPIOB->ODR &= ~DOWN_CS_Pin;

#define LED1_ON()				GPIOA->ODR &= ~(LED4_Pin);
#define LED2_ON()				GPIOB->ODR &= ~(LED3_Pin);
#define LED3_ON()				GPIOA->ODR &= ~(LED2_Pin);
#define LED4_ON()				GPIOB->ODR &= ~(LED1_Pin);

#define LED1_OFF()				GPIOA->ODR |= (LED4_Pin);
#define LED2_OFF()				GPIOB->ODR |= (LED3_Pin);
#define LED3_OFF()				GPIOA->ODR |= (LED2_Pin);
#define LED4_OFF()				GPIOB->ODR |= (LED1_Pin);

#define RX1_ON()				GPIOA->ODR &= ~(RX4_Pin);
#define RX2_ON()				GPIOA->ODR &= ~(RX3_Pin);
#define RX3_ON()				GPIOA->ODR &= ~(RX2_Pin);
#define RX4_ON()				GPIOB->ODR &= ~(RX1_Pin);

#define RX1_OFF()				GPIOA->ODR |=  (RX4_Pin);
#define RX2_OFF()				GPIOA->ODR |=  (RX3_Pin);
#define RX3_OFF()				GPIOA->ODR |=  (RX2_Pin);
#define RX4_OFF()				GPIOB->ODR |=  (RX1_Pin);


#define SPI_SETUP_TIME			55u
uint16_t ir_values[4][4];


enum pixel_colors{
	red = 1,
	green,
	blue,
	yellow,
	purple,
	cyan,
	orange,
	magenta,
	lime,
	dull_green,
	dull_blue,
	violet
};

enum color_positions{

	red_pos,
	green_pos,
	blue_pos

};

enum Panel_mode{

	auto_mode,
	application_mode

};

struct Color display[4][4];



struct panel_profile{

	uint16_t active_pixels;
	uint16_t commanded_pixels;
	uint8_t commanded_colors[8];
	uint8_t red;
	uint8_t green;
	uint8_t blue;
	uint8_t mode;

};

struct panel_assembly{

	uint8_t num_panels;
	uint8_t x;
	uint8_t y;
	uint16_t led_sensitivity;
	uint8_t * colors_tx;
	uint8_t * all_active_pixels;
	struct panel_profile * panel;

};

struct Panel_stream{
	uint8_t panel_id;
	uint8_t stream_dir;
	uint8_t upstream_dir;
	uint8_t downstream_dir;
	uint8_t try_first;
	uint8_t try_second;
	uint8_t * header;
	uint8_t * payload;
};

extern struct Panel_stream panel_stream;


extern struct panel_assembly table;

extern uint8_t panel_brightness;
extern uint8_t clut_ram[8];
extern uint8_t rx_header[4];
extern uint8_t rx_data[800];
//Call back for DMA Transfers
void HAL_ADC_ConvCpltCallback (ADC_HandleTypeDef * hadc);

void read_ir_sensor(uint8_t x, uint8_t y, uint16_t values[4][4]);

void set_panel_pixels(struct Led_pixels * pixels, struct Color display[4][4]);

void determine_panel_positions(uint8_t mode);

uint8_t panel_position_ping(void);

void set_collective_pixels(uint32_t x, uint32_t y, uint8_t color, struct panel_assembly * assembly);

void set_panel_pixel_tx(uint8_t * colors_tx, uint8_t x, uint8_t y, uint8_t color);

void set_panel_clut(struct Led_pixels * pixels, uint8_t * clut);

void get_panel_header(uint8_t * header);

void get_panel_payload(uint16_t datalen, uint8_t * rx_buffer);

void panel_get_pos_cmd(uint8_t * header, uint8_t * payload, uint8_t source);

void panel_set_pixel_cmd(uint8_t * header, uint8_t * payload, uint8_t source);

void panel_set_auto_cmd(uint8_t * header, uint8_t * payload, uint8_t source);

uint8_t send_panel_header(uint8_t * header, uint8_t direction);

uint8_t stream_panel(struct Panel_stream * panel);

uint8_t send_data_down(uint8_t *header, uint8_t * payload, uint8_t dir);

uint8_t send_data_up(uint8_t *header, uint8_t * payload, uint8_t dir);

void set_panel_it(uint8_t dir);

void set_panel_output(void);

void panel_dir_config(uint8_t up, uint8_t down);

uint8_t master_get_data(uint8_t * header, uint8_t * payload, uint8_t dir);

uint8_t get_panel_cmd(uint8_t source);

uint8_t get_panel_structure(void);

void update_panel_pixels(void);

void panel_chain_pixel_cmd(uint8_t * header, uint8_t * data);

void panel_get_pix_cmd(uint8_t * header, uint8_t * data);

uint8_t get_active_pixels(void);

void panel_set_auto(uint8_t * header, uint8_t * data);

void set_ir_color(uint8_t * panel_bg);

void set_panel_bg(uint8_t panel_num, uint8_t * bg, uint8_t red, uint8_t green, uint8_t blue);

void play_pong(void);

void get_local_ir(void);

void show_ir(void);

void clear_collective();

uint8_t  inPaddle(int x, int y, int rectX, int rectY, int rectWidth, int rectHeight);

uint8_t is_pixel_active(struct panel_assembly * table, uint16_t x, uint16_t y);

void set_cs_output(uint8_t csPin);

void get_header_payload(uint8_t source);

#endif /* INC_LED_PANEL_H_ */
