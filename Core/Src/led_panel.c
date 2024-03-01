/*
 * led_panel.c
 *
 *  Created on: Oct 11, 2020
 *      Author: rtrusty
 */

#include "led_panel.h"
#include "stm32f4xx_it.h"
struct Led_pixels pixels[NUM_PIXELS];
uint8_t conv = 0;
//
//uint8_t panel_clut[][3] = {
//
//		{255, 0, 0},	//red
//		{0, 255, 0},	//green
//		{0, 0, 255},	//blue
//		{255, 255, 0},	//yellow
//		{255, 0, 255},	//purple
//		{0, 255, 255},	//cyan
//		{255, 127, 0},	//orange
//		{255, 0, 127},	//magenta
//		{127, 255, 0},	//lime
//		{0, 255, 127},	//dull green
//		{0, 127, 255},	//dull blue
//		{127, 0, 255}	//violet
//};


uint8_t panel_clut[][3] = {

		{30, 0, 0},	//red
		{0, 30, 0},	//green
		{0, 0, 30},	//blue
		{30, 30, 0},	//yellow
		{30, 0, 30},	//purple
		{0, 30, 30},	//cyan
		{30, 15, 0},	//orange
		{30, 0, 21},	//magenta
		{15, 30, 0},	//lime
		{0, 30, 15},	//dull green
		{0, 15, 30},	//dull blue
		{15, 0, 30}	//violet
};

uint8_t panel_brightness = 0;


void read_ir_sensor(uint8_t x, uint8_t y, uint16_t values[4][4]){

	//array for analog read
	uint32_t a_reads[4];

	uint32_t ava[4];

	//Set appropriate I/0 to activate selected sensor

	switch(y){

		case 0: RX1_ON();
				RX2_OFF();
				RX3_OFF();
				RX4_OFF();

				LED1_OFF();
				LED2_OFF();
				LED3_OFF();
				LED4_OFF();
			break;

		case 1:	RX1_OFF();
				RX2_ON();
				RX3_OFF();
				RX4_OFF();

				LED1_OFF();
				LED2_OFF();
				LED3_OFF();
				LED4_OFF();
			break;

		case 2:	RX1_OFF();
				RX2_OFF();
				RX3_ON();
				RX4_OFF();

				LED1_OFF();
				LED2_OFF();
				LED3_OFF();
				LED4_OFF();
			break;

		case 3:	RX1_OFF();
				RX2_OFF();
				RX3_OFF();
				RX4_ON();

				LED1_OFF();
				LED2_OFF();
				LED3_OFF();
				LED4_OFF();

			break;

		default:
			break;

	}

	switch(x){

		case 0: HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET);

			break;
		case 1: HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET);
			break;
		case 2: HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);
			break;
		case 3: HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_SET);
			break;
		default:
			break;

	}
	//HAL_Delay(1);
	delay_us(10);

	//Read with LEDs off
	int ret = HAL_ADC_Start_DMA(&hadc1, a_reads, 4);

	//Poll for completion
	//Polling is fine in this case as it meets timing requirements
	while(!conv){}
	conv = 0;


	//Turn LED ON
	switch(y){

		case 0: LED1_ON();
			break;

		case 1:	LED2_ON();
			break;

		case 2: LED3_ON();
			break;

		case 3:	LED4_ON();

			break;

		default:
			break;

	}

	delay_us(10);
	//HAL_Delay(1);

	//Read with LEDs ON
	HAL_ADC_Start_DMA(&hadc1, ava, 4);

	//Poll for completion
	//Polling is fine in this case as it meets timing requirements
	while(!conv){}
	conv = 0;
	//Store conversion data in array

	//Deactivate Sensor
	//Set appropriate I/0 to activate selected sensor

	switch(y){

		case 0: HAL_GPIO_WritePin(GPIOA, RX3_Pin|RX2_Pin|LED2_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOB, RX1_Pin|LED1_Pin|LED3_Pin, GPIO_PIN_RESET);
			break;

		case 1:	HAL_GPIO_WritePin(GPIOA, RX4_Pin|RX2_Pin|LED2_Pin|LED4_Pin, GPIO_PIN_RESET);
		  	  	HAL_GPIO_WritePin(GPIOB, RX1_Pin|LED1_Pin, GPIO_PIN_RESET);
			break;

		case 2:	HAL_GPIO_WritePin(GPIOA, RX4_Pin|LED4_Pin|RX3_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOB, RX1_Pin|LED1_Pin|LED3_Pin, GPIO_PIN_RESET);
			break;

		case 3: HAL_GPIO_WritePin(GPIOA, RX4_Pin|RX2_Pin|LED2_Pin|LED4_Pin|RX3_Pin, GPIO_PIN_RESET);
		    	HAL_GPIO_WritePin(GPIOB, LED3_Pin, GPIO_PIN_RESET);
			break;

		default:
			break;

	}

	switch(x){

		case 0: HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET);

			break;
		case 1: HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET);
			break;
		case 2: HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);
			break;
		case 3: HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_RESET);
			break;
		default:
			break;

	}


	if(a_reads[3 - y] > ava[3 - y]){

		values[x][y] = 0;
	}
	else{

		values[x][y] = ava[3 - y] - a_reads[3 - y];

	}

}


void HAL_ADC_ConvCpltCallback (ADC_HandleTypeDef * hadc){



	  HAL_ADC_Stop_DMA(&hadc1);
	  conv = 1;

}


void set_panel_pixels(struct Led_pixels * pixels, struct Color display[4][4]){

	int pix = 0;
	int why = 0;


	for(int x = 0; x < 4; x++){

		for(int y = 0; y < 4; y++){

			if(x % 2 == 0){//If x is even

				why = 3-y;

			}
			else{
				why = y;
			}
			//Get pixel based on panel layout
			pix = (x * 4) + why;

			set_pixel_color(pixels, pix, display[x][y].red, display[x][y].green, display[x][y].blue);
		}

	}



}

void set_panel_clut(struct Led_pixels * pixels, uint8_t * clut){



for(int x = 0; x < 8; x++){


	if((clut[x] & 0xF) != 0){


		set_pixel_color(pixels, (x*2) ,
				panel_clut[(clut[x] & 0xF)-1][red_pos],
				panel_clut[(clut[x] & 0xF)-1][green_pos],
				panel_clut[(clut[x] & 0xF)-1][blue_pos]);

	}

	if((clut[x] >> 4) != 0){

	set_pixel_color(pixels, (x*2)+1 ,
			panel_clut[(clut[x] >> 4)-1][red_pos],
			panel_clut[(clut[x] >> 4)-1][green_pos],
			panel_clut[(clut[x] >> 4)-1][blue_pos]);

	}


//	if((clut[x] & 0xF) == 0xF){
//
//	set_pixel_color(pixels, (x*2) , 10, 0, 0);
//
//	}else{
//
//	//set_pixel_color(pixels, (x*2) , 0, 0, 0);
//
//	}
//
//	if((clut[x] >> 4) == 0xF){
//
//	set_pixel_color(pixels, (x*2)+1 , 10, 0, 0);
//
//	}
//	else{
//
//	//set_pixel_color(pixels, (x*2)+1 , 0, 0, 0);
//
//	}
}

}

void determine_panel_positions(uint8_t mode){

if(mode == 1){

	panel_position_ping();

}

else if(mode == 0){


}


};

uint8_t panel_position_ping(void){

	uint8_t msg[10] = {0};
	msg[0] = 0xA;
	msg[1] = 0xB;
	msg[2] = 0xFF;
	msg[3] = 0xFF;
	msg[4] = 0xFF;
	msg[5] = 0xFF;
	msg[6] = 0xFF;
	msg[7] = 0xFF;
	msg[8] = 0xFF;
	msg[9] = 0xFF;
	uint8_t pRxData[10] = {0};

	//Set RIGHT CS
	GPIOB->ODR |= RIGHT_CS_Pin;

	//Send MSG
	//HAL_SPI_Transmit(&hspi2, msg, 5, 1000);
	//uint8_t ret = HAL_SPI_TransmitReceive (&hspi2, msg, pRxData, 10, 1000);

	uint8_t ret = HAL_SPI_Transmit(&hspi2, msg, 2, TXRX_TIMEOUT);
	//HAL_Delay(1);
    delay_us(SPI_SETUP_TIME);
	HAL_SPI_TransmitReceive(&hspi2, &msg[2], pRxData, 4, TXRX_TIMEOUT);
	return pRxData[0];
}

void set_collective_pixels(uint32_t x, uint32_t y, uint8_t color, struct panel_assembly * assembly){
//7, 5, -> 8rd byte in 3rd
//9, 1
	int panel_x = x/4; //1  2
	int panel_y = y/4; //1  0

	int pix_x = x % 4; //3  1
	int pix_y = y % 4; //1  1

	//as x = 3
	//as y = 1

	int panel_num = (panel_y * assembly->x); //2  0


	if(panel_y % 2 == 0){

		panel_num += panel_x;

	}
	else{

		panel_num += ((assembly->x - 1) - panel_x);

	}

	set_panel_pixel_tx(&assembly->colors_tx[panel_num * 8], pix_x, pix_y, color);



}

void set_panel_pixel_tx(uint8_t * colors_tx, uint8_t x, uint8_t y, uint8_t color){

	int pix_num;

	//If x is odd
	if(x % 2){

		pix_num = (x * 4) + y;

	}
	else{

		pix_num = (x * 4) + (3 - y);

	}

	//If odd
	if(pix_num % 2){

		colors_tx[(pix_num/2)] &= ~(0xF << 4);
		colors_tx[(pix_num/2)] |=  (color << 4);

	}
	else{

		colors_tx[(pix_num/2)] &= ~(0xF);
		colors_tx[(pix_num/2)] |=  (color);


	}

}


void get_panel_header(uint8_t * header){


	SPI2->DR = RESP_CODE;
	SPI2->DR = RESP_CODE;

	header[0] = SPI2->DR;
    header[0] = SPI2->DR;

    GPIOB->PUPDR &= ~(0x3 << 26);
    GPIOB->PUPDR &= ~(0x3 << 28);
    GPIOB->PUPDR &= ~(0x3 << 30);

	HAL_SPI_TransmitReceive(&hspi2, resp_code, header, HEADER_SIZE, 5);
//
//	SELECT_UP();
//	UNSELECT_UP();
//
//	for(int j = 0; j < HEADER_SIZE; j++){
//
//		SPI2->DR = RESP_CODE;
//		while(!(SPI2->SR & 0x02)){}
//		while(!(SPI2->SR & 0x01)){}
//
//		header[j] = SPI2->DR;
//
////		SELECT_UP();
////		UNSELECT_UP();
//
//	}


}

void get_panel_payload(uint16_t datalen, uint8_t * rx_buffer){


	SPI2->DR = RESP_CODE;

	HAL_SPI_TransmitReceive(&hspi2, resp_code,rx_buffer, datalen, 5);

//	uint32_t to = 0;
//	for(int j = 0; j < datalen; j++){
//
//		SPI2->DR = RESP_CODE;
//		while(!(SPI2->SR & 0x01)){}
//		rx_buffer[j] = SPI2->DR;
//
//	}


//	uint8_t msg[4] = {0};
//
//	msg[0] = RESP_CODE;
//	msg[1] = RESP_CODE;
//	msg[2] = RESP_CODE;
//	msg[3] = RESP_CODE;
//
//	//TODO: Error handling
//	HAL_SPI_TransmitReceive(&hspi2, msg,rx_buffer, datalen, 1000);


}



void panel_get_pos_cmd(uint8_t * header, uint8_t * payload, uint8_t source){


	panel_stream.header = header;
	panel_stream.payload = payload;

	//If data upstream increment panel tx number & total panels
	if((source == CS_LEFT) && (panel_stream.payload[TABLE_Y] == 1)){

		panel_stream.payload[TABLE_X]++;

	}

	//If initial position calibration
	panel_stream.panel_id = 0;

	if((panel_stream.panel_id == 0)){



		panel_stream.downstream_dir = source;

		//Increment tx address, panel cnt and store self addy
		panel_stream.header[HEADER_TX_ADDY]++;

		panel_stream.payload[PANEL_CNT]++;

		panel_stream.panel_id = panel_stream.header[HEADER_TX_ADDY];

		panel_stream.stream_dir = PANEL_UPSTREAM;

	}

	//If not initial upstream
	else if((panel_stream.header[HEADER_TX_ADDY]) < panel_stream.panel_id){

		//Set panel ID
		panel_stream.header[HEADER_TX_ADDY] = panel_stream.panel_id;

		//Increment panel cnt
		payload[PANEL_CNT]++;

		panel_stream.stream_dir = PANEL_UPSTREAM;

	}

	//If downstream
	else if((panel_stream.header[HEADER_TX_ADDY]) > panel_stream.panel_id){

		panel_stream.header[HEADER_TX_ADDY] = panel_stream.panel_id;

		panel_stream.stream_dir = PANEL_DOWNSTREAM;

	}



	//If interrupted from left
	if(source == CS_LEFT){



		panel_stream.try_first = CS_RIGHT;
		panel_stream.try_second = CS_UP;

	}
	else if(source == CS_RIGHT){

		panel_stream.try_first = CS_LEFT;
		panel_stream.try_second = CS_UP;

	}

	else if(source == CS_DOWN){

		//Increment Y
		payload[TABLE_Y]++;
		panel_stream.try_first = CS_RIGHT;
		panel_stream.try_second = CS_LEFT;

	}


	else if(source == CS_UP){

		panel_stream.try_first = CS_RIGHT;
		panel_stream.try_second = CS_LEFT;


	}

//Debug
//	panel_stream.payload[TABLE_X] = 3;
//	panel_stream.payload[TABLE_Y] = 2;
//	panel_stream.payload[PANEL_CNT] = 5;

	stream_panel(&panel_stream);



}


void panel_set_pixel_cmd(uint8_t * header, uint8_t * payload, uint8_t source){



}

void panel_set_auto_cmd(uint8_t * header, uint8_t * payload, uint8_t source){



}


uint8_t send_panel_header(uint8_t * header, uint8_t direction){

	uint8_t buff[HEADER_SIZE] = {0};

	//Change to SPI_Master
	set_spi2_mode(SPI2_MASTER);
	SPI2->CR1 |= SPI_CR1_SPE;

	//Set chip select
	switch(direction){

	case CS_LEFT:	SELECT_LEFT();
		break;
	case CS_DOWN:	SELECT_DOWN();
		break;
	case CS_RIGHT:	SELECT_RIGHT();
		break;
	case CS_UP:		SELECT_UP();
		break;

	}


	//TODO: delay??
	//Send Header
	//HAL_Delay(1);
    delay_us(SPI_SETUP_TIME);
	uint8_t ret = HAL_SPI_TransmitReceive(&hspi2, header, (uint8_t *)buff, HEADER_SIZE, TXRX_TIMEOUT);


	switch(direction){

	case CS_LEFT:	UNSELECT_LEFT();
		break;
	case CS_DOWN:	UNSELECT_DOWN();
		break;
	case CS_RIGHT:	UNSELECT_RIGHT();
		break;
	case CS_UP:		UNSELECT_UP();
		break;

	}

	//Throw error if code does not match
	for(int i = 0; i < HEADER_SIZE; i++){

		if(header[i] != RESP_CODE){

			ret |= RESP_ERR;

		}

	}

	SPI2->CR1 &= ~SPI_CR1_SPE;
	set_spi2_mode(SPI2_SLAVE);

	return ret;

}

uint8_t stream_panel(struct Panel_stream * panel){

	uint8_t send = 0;

	if(panel->stream_dir == PANEL_UPSTREAM){

		panel->upstream_dir = 0;
		//Determine upstream dir if not already save
		if(panel->upstream_dir == 0){

			set_spi2_mode(SPI2_MASTER);

			send = send_data_up(panel->header, panel->payload, panel->try_first);

			//TODO: delay??
			//If failed
			if(send){

				send = send_data_up(panel->header, panel->payload, panel->try_second);

				//If send fails again
				if(send){

					panel->upstream_dir = LAST_PANEL;
					//Return Data
					panel_dir_config(panel->upstream_dir, panel->downstream_dir);

					send = send_data_down(panel->header, panel->payload, panel->downstream_dir);

				}

				else{

					panel->upstream_dir = panel->try_second;

					//config stream directions
					panel_dir_config(panel->upstream_dir, panel->downstream_dir);

					master_get_data(panel->header, panel->payload, panel->upstream_dir);

					send_data_down(panel->header, panel->payload, panel->downstream_dir);

					//set_spi2_mode(SPI2_SLAVE);

				}

			}

			else{

				panel->upstream_dir = panel->try_first;

				panel_dir_config(panel->upstream_dir, panel->downstream_dir);

				master_get_data(panel->header, panel->payload, panel->upstream_dir);

				send_data_down(panel->header, panel->payload, panel->downstream_dir);

				//set_spi2_mode(SPI2_SLAVE);

			}

		}
		//Else use stored upstream dir
		else if(panel->upstream_dir < 5){

			set_spi2_mode(SPI2_MASTER);

			send = send_data_up(panel->header, panel->payload, panel->upstream_dir);

			panel_dir_config(panel->upstream_dir, panel->downstream_dir);

			master_get_data(panel->header, panel->payload, panel->upstream_dir);

			send_data_down(panel->header, panel->payload, panel->downstream_dir);

		}
		//Else this is the last panel
		else if(panel->upstream_dir == LAST_PANEL){

			//Start return process
			send = send_data_down(panel->header, panel->payload, panel->downstream_dir);


		}

	}

	else if(panel->stream_dir == PANEL_DOWNSTREAM){

		send = send_data_down(panel->header, panel->payload, panel->downstream_dir);

	}



	return send;
}

uint8_t send_data_down(uint8_t *header, uint8_t * payload, uint8_t dir){


	 // HAL_Delay(1);

//delay_us(5);
	uint32_t cs_pin = 0;
	int ret = 0;

	//Disable interrupt for trigger pin
	switch(dir){

	case CS_LEFT: cs_pin = LEFT_CS_Pin;
				  GPIOB->PUPDR |=  (0x01 << 2);
				  GPIOB->OSPEEDR |=  (0x03 << 2);
				  GPIOB->MODER |= (0x01 << 2);
		break;

	case CS_RIGHT: cs_pin = RIGHT_CS_Pin;
				   GPIOB->PUPDR |=  (0x01 << 20);
				   GPIOB->OSPEEDR |=  (0x03 << 20);
				   GPIOB->MODER |= (0x01 << 20);
		break;

	case CS_DOWN: cs_pin = DOWN_CS_Pin;
				  GPIOB->PUPDR |=  (0x01 << 4);
				  GPIOB->OSPEEDR |=  (0x03 << 4);
				  GPIOB->MODER |= (0x01 << 4);

		break;
	default:
		break;

	}

//	  SELECT_UP();
//	  UNSELECT_UP();
//	  SELECT_UP();
//	  UNSELECT_UP();
//	  SELECT_UP();
//	  UNSELECT_UP();
//	  SELECT_UP();
//	  UNSELECT_UP();
	 //Configure trigger gpio for output



//	  GPIO_InitTypeDef GPIO_InitStruct = {0};
//
//	  HAL_GPIO_DeInit(GPIOB, cs_pin);
//
//	  GPIO_InitStruct.Pin = cs_pin;
//	  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
//	  GPIO_InitStruct.Pull = GPIO_NOPULL;
//	  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
//	  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

//	  SELECT_UP();
//	  UNSELECT_UP();
//	  SELECT_UP();
//	  UNSELECT_UP();
//	  SELECT_UP();
//	  UNSELECT_UP();
//	  SELECT_UP();
//	  UNSELECT_UP();

	//Pulse transfer request

		set_spi2_mode(SPI2_SLAVE);
		//Note: This allows the downstream panel time to set up it's interrupt before we pulse it

		delay_us(10);

		SPI2->CR1 |=  SPI_CR1_SPE;
		//Pulse transfer request
		switch(dir){

		case CS_LEFT: EXTI->IMR &= ~(0x02);
					  SELECT_LEFT();
					  EXTI->IMR |= (0x02);
					  break;

		case CS_RIGHT:EXTI->IMR &= ~(0x400);
		  	  	  	  SELECT_RIGHT();
					  EXTI->IMR |= (0x400);
					  break;

		case CS_UP:   EXTI->IMR &= ~(0x400);
		  	  	  	  SELECT_UP();
					  EXTI->IMR |= (0x400);
					  break;

		case CS_DOWN: EXTI->IMR &= ~(0x04);
		  	  	  	  SELECT_DOWN();
					  EXTI->IMR |= (0x04);
					  break;

		}

	  //Clear pending ints
	  EXTI->PR = 0x02 | 0x04 | 0x400;

	//  EXTI->IMR &= ~(0x02 | 0x04 | 0x400);

	  SYSCFG->EXTICR[0] &= ~(0xF << 8);

	//Prepare data transmission
	uint8_t dumb[10];

	int datalen = ((header[PAYLOAD_HIGH] << 8) | header[PAYLOAD_LOW]);

	if(datalen > 800){

		datalen = 800;

	}




//    uint16_t dummy = 0;
//	for(int j = 0; j < HEADER_SIZE; j++){
//
//		SPI2->DR = header[j];
//		while(!(SPI2->SR & 0x01)){}
//		dummy = SPI2->DR;
//
//	}
//
//	for(int j = 0; j < datalen; j++){
//
//		SPI2->DR = payload[j];
//		while(!(SPI2->SR & 0x01)){}
//		dummy = SPI2->DR;
//
//	}

	ret = HAL_SPI_TransmitReceive(&hspi2, header, dumb, HEADER_SIZE, TXRX_TIMEOUT);

	ret = HAL_SPI_TransmitReceive(&hspi2, payload, dumb, datalen, TXRX_TIMEOUT);

	SPI2->CR1 &=  ~SPI_CR1_SPE;

	//Note: This setup time is required to allow downstream master to transition states
	delay_us(10);
	switch(dir){

	case CS_LEFT: EXTI->IMR &= ~(0x02);
				  UNSELECT_LEFT();
				  EXTI->IMR |= (0x02);
				  break;

	case CS_RIGHT:EXTI->IMR &= ~(0x400);
				  UNSELECT_RIGHT();
				  EXTI->IMR |= (0x400);
				  break;

	case CS_UP:   EXTI->IMR &= ~(0x400);
				  UNSELECT_UP();
				  EXTI->IMR |= (0x400);
				  break;

	case CS_DOWN: EXTI->IMR &= ~(0x04);
				  UNSELECT_DOWN();
				  EXTI->IMR |= (0x04);
				  break;

	}

	set_panel_it(panel_stream.downstream_dir);

	//set_panel_it(CS_RIGHT);
	//SYSCFG->EXTICR[2] &= (0x01 << 8);

return ret;
}


uint8_t send_data_up(uint8_t *header, uint8_t * payload, uint8_t dir){

		SPI2->CR1 |= SPI_CR1_SPE;

	  //Activate chip select
		switch(dir){

		case CS_LEFT: EXTI->IMR &= ~(0x02);
				      SELECT_LEFT();
					  break;

		case CS_RIGHT:EXTI->IMR &= ~(0x400);
	      	          SELECT_RIGHT();
					  break;

		case CS_UP:    EXTI->IMR &= ~(0x400);
	                  SELECT_UP();
					  break;

		case CS_DOWN: EXTI->IMR &= ~(0x04);
	                  SELECT_DOWN();
					  break;

		}

	    uint8_t dumb[10];
	    uint8_t dumb2[10];
		//HAL_Delay(1);
	    delay_us(SPI_SETUP_TIME);	    //Send header
	    uint8_t tmp[4];
	    tmp[0] = SPI2->DR;
	    tmp[0] = SPI2->DR;
	    tmp[0] = SPI2->SR;
	    tmp[0] = SPI2->SR;
	    tmp[0] = (0x01 << 7);
	    tmp[1] = (0x01 << 6);
	    tmp[2] = (0x01 << 5);
	    tmp[3] = (0x01 << 4);

		uint8_t ret = HAL_SPI_TransmitReceive(&hspi2,header, (uint8_t *)dumb, HEADER_SIZE, TXRX_TIMEOUT);
//
//	    for(uint16_t i=0; i<1; i++)
//	    {
//	    	SPI2->DR = tmp[i];
//	      while (!(SPI2->SR & SPI_SR_TXE_Msk ));
//	    	dumb[i] = SPI2->DR;
//	    }
		//If header response ok then send payload

		for(int i = 0; i < HEADER_SIZE; i++){

			if(dumb[i] != RESP_CODE){

				ret |= RESP_ERR;

			}

		}

		if(ret == 0){
		//Send Payload
			//HAL_Delay(1);
		    delay_us(SPI_SETUP_TIME);
		ret = HAL_SPI_TransmitReceive(&hspi2,payload, (uint8_t *)dumb2, 3, TXRX_TIMEOUT);

		}


		switch(dir){

		case CS_LEFT: EXTI->IMR |= (0x02);
					  UNSELECT_LEFT();
					  break;

		case CS_RIGHT:EXTI->IMR |= (0x400);
		  	  	  	  UNSELECT_RIGHT();
					  break;

		case CS_UP:   EXTI->IMR |= (0x400);
		  	  	  	  UNSELECT_UP();
					  break;

		case CS_DOWN: EXTI->IMR |= (0x04);
		  	  	  	  UNSELECT_DOWN();
		  	  	  	  break;

		}


		SPI2->CR1 &= ~SPI_CR1_SPE;

		return ret;

}




void panel_dir_config(uint8_t up, uint8_t down){

	//downstream pin as interrupt
	//set_panel_it(down);

	HAL_NVIC_DisableIRQ(EXTI1_IRQn);
	HAL_NVIC_DisableIRQ(EXTI1_IRQn);
	HAL_NVIC_DisableIRQ(EXTI15_10_IRQn);
	//Set upstream priority to 2

	//Downstream priority to 3

	switch(up){

		case CS_LEFT:	HAL_NVIC_SetPriority(EXTI1_IRQn, 2, 0);
					 	HAL_NVIC_EnableIRQ(EXTI1_IRQn);
			break;

		case CS_RIGHT:	HAL_NVIC_SetPriority(EXTI15_10_IRQn, 2, 0);
	 					HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

			break;

		case CS_UP:		HAL_NVIC_SetPriority(EXTI15_10_IRQn, 2, 0);
	 					HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

			break;

		case CS_DOWN:	HAL_NVIC_SetPriority(EXTI2_IRQn, 2, 0);
	 					HAL_NVIC_EnableIRQ(EXTI2_IRQn);

			break;

		default:
			break;

	}

	switch(down){

		case CS_LEFT:	HAL_NVIC_SetPriority(EXTI1_IRQn, 3, 0);
					 	HAL_NVIC_EnableIRQ(EXTI1_IRQn);
			break;

		case CS_RIGHT:	HAL_NVIC_SetPriority(EXTI15_10_IRQn, 3, 0);
	 					HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

			break;

		case CS_UP:		HAL_NVIC_SetPriority(EXTI15_10_IRQn, 3, 0);
	 					HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

			break;

		case CS_DOWN:	HAL_NVIC_SetPriority(EXTI2_IRQn, 3, 0);
	 					HAL_NVIC_EnableIRQ(EXTI2_IRQn);

			break;

		default:
			break;

	}


}


void set_panel_it(uint8_t dir){

	//Right CS pin interrupt


	switch(dir){

	case CS_LEFT:
					//very high ospeed
					GPIOB->OSPEEDR |= 0x3 << 2;

					//Pull up
					GPIOB->PUPDR |= 0x01 <<	2;

					//EXTI 10 to pb10
					SYSCFG->EXTICR[0] |= 0x01 << 4;

					//Unmask IT
					EXTI->IMR |= 0x01 << 1;

					//Falling Trigger
					EXTI->FTSR |= 0x01 << 1;

					HAL_NVIC_EnableIRQ(EXTI1_IRQn);

					//Input mode
					GPIOB->MODER &= ~(0x3 << 2);

		break;

	case CS_RIGHT:
					//very high ospeed
					GPIOB->OSPEEDR |= 0x3 << 20;

					//Pull up
					GPIOB->PUPDR |= 0x01 <<	20;

					SYSCFG->EXTICR[2] |= 0x01 << 8;

					//Unmask IT
					EXTI->IMR |= 0x01 << 10;

					//Faling Trigger
					EXTI->FTSR |= 0x01 << 10;

					HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

					//Input mode
					GPIOB->MODER &= ~(0x3 << 20);
		break;

	case CS_UP:
					//very high ospeed
					GPIOA->OSPEEDR |= 0x3 << 20;

					//Pull up
					GPIOA->PUPDR |= 0x01 <<	20;

					HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

					//EXTI 10 to pa10
					SYSCFG->EXTICR[2] &= ~(0x03 << 8);

					//Unmask IT
					EXTI->IMR |= 0x01 << 10;

					//Faling Trigger
					EXTI->FTSR |= 0x01 << 10;

					//Input mode
					GPIOA->MODER &= ~(0x3 << 20);
		break;

	case CS_DOWN:
					//very high ospeed
					GPIOB->OSPEEDR |= 0x3 << 4;

					//Pull up
					GPIOB->PUPDR |= 0x01 <<	4;

					HAL_NVIC_EnableIRQ(EXTI2_IRQn);
					//EXTI 10 to pb10
					SYSCFG->EXTICR[0] |= 0x01 << 8;

					//Unmask IT
					EXTI->IMR |= 0x01 << 2;

					//Faling Trigger
					EXTI->FTSR |= 0x01 << 2;

					//Input mode
					GPIOB->MODER &= ~(0x3 << 4);
		break;

	default:
	break;

	}


}

void set_panel_output(void){

	//Turn off all interrupts

//	HAL_NVIC_DisableIRQ(EXTI15_10_IRQn);
//	HAL_NVIC_DisableIRQ(EXTI1_IRQn);
//	HAL_NVIC_DisableIRQ(EXTI2_IRQn);

	EXTI->IMR &= ~(0x02 | 0x04 | 0x400);

//	SYSCFG->EXTICR[0] &= 0xF << 4;
//	SYSCFG->EXTICR[0] &= 0xF << 8;
//	SYSCFG->EXTICR[2] &= 0xF << 8;

	//Turn on GPIO Clocks
	RCC->AHB1ENR |= 0x03;



	//Set Pullup
	GPIOB->PUPDR |= ((0x01) << 2) | ((0x01) << 4) | ((0x01) << 20);
	GPIOA->PUPDR |= (0x01 << 20);

	GPIOB->OSPEEDR |= ((0x03) << 2) | ((0x03) << 4) | ((0x03) << 20);
	GPIOA->OSPEEDR |= (0x03 << 20);

	//Set ODR
	GPIOB->ODR |= ((0x01) << 1) | ((0x01) << 2) | ((0x01) << 10);
	GPIOA->ODR |= (0x01 << 10);

	//Set Mode
	GPIOB->MODER |= ((0x01) << 2) | ((0x01) << 4) | ((0x01) << 20);
	GPIOA->MODER |= (0x01 << 20);

	//Pull down SPI Clock
    GPIOB->PUPDR |= 0x02 << 26;


}

uint8_t master_get_data(uint8_t * header, uint8_t * payload, uint8_t dir){

	//Set upstream pin to interrupt
		delay_us(25);
		set_panel_it(dir);


		int ret = 0xFF;

		uint32_t tx_start = HAL_GetTick();

		SPI2->CR1 &= ~SPI_CR1_SPE;

		int_wait = 0;

		//Wait for int to trigger
		while((int_wait == 0) && ((HAL_GetTick() - tx_start) < INT_WAIT_TIMEOUT)){}

//		SELECT_UP();
//		UNSELECT_UP();
//		SELECT_UP();
//		UNSELECT_UP();
//		int_wait = 1;
//		delay_us(20);
//		while((GPIOB->IDR & (0x01 << 10)) == 1){}
//		SELECT_UP();
//		UNSELECT_UP();
//		SELECT_UP();
//		UNSELECT_UP();



		if(int_wait == 1){

		int_wait = 0;

		SPI2->CR1 |= SPI_CR1_SPE;

//		set_spi2_mode(SPI2_MASTER);
//		set_panel_output();
//		SYSCFG->EXTICR[2] &= ~(0xF << 8);

//			GPIOB->ODR |= (dir);
//			GPIOB->ODR &= ~(dir);

			uint8_t holder[800];
			holder[0] = 0xFF;
			holder[1] = 0xFF;
			holder[2] = 0xFF;
			holder[3] = 0xFF;


//HAL_Delay(1);
delay_us(SPI_SETUP_TIME);
			int ret = HAL_SPI_TransmitReceive(&hspi2,holder, header, HEADER_SIZE, TXRX_TIMEOUT);

			int datalen = (header[PAYLOAD_HIGH] << 8 | header[PAYLOAD_LOW]);

			if(datalen > 800){

				datalen = 800;

			}

			if(header[HEADER_CMD] == PANEL_GET_POS_CMD ||
			   header[HEADER_CMD] == PANEL_GET_PIX_CMD	){

		    //HAL_Delay(1);
		    delay_us(SPI_SETUP_TIME);

			ret = HAL_SPI_TransmitReceive(&hspi2,holder, payload, datalen, TXRX_TIMEOUT);

			}
			else{

				header[PAYLOAD_HIGH] = 0;
				header[PAYLOAD_LOW]  = 0;
			}
//			GPIOB->ODR |= (dir);

			SPI2->CR1 |= SPI_CR1_SPE;
		}

		//Note: Logical NOT circuit pulls down CS
		//This is a minor design flaw
		//As a result the CS must always be driven because the
		//internal pullup does not overcome the external pulldown
		set_cs_output(dir);


	return ret;

}



uint8_t get_panel_cmd(uint8_t source){

	set_spi2_mode(SPI2_SLAVE);


	//Turn on SPI
	SPI2->CR1 |= SPI_CR1_SPE;

	//Get Header
	get_panel_header(rx_header);

	//TODO: CRC?

	//Process command from geader
	if((rx_header[HEADER_CMD] == PANEL_GET_POS_CMD)|
	   (rx_header[HEADER_CMD] == PANEL_SET_PIXEL_CMD)|
	   (rx_header[HEADER_CMD] == PANEL_SET_AUTO_CMD)|
	    rx_header[HEADER_CMD] == PANEL_GET_PIX_CMD){

		//Parse data length
		uint16_t datalen = (rx_header[PAYLOAD_HIGH] << 8 | rx_header[PAYLOAD_LOW]);

		if(datalen > 800){

			datalen = 800;

		}

		get_panel_payload(datalen, rx_data);
int useless;
		switch(rx_header[HEADER_CMD]){

			case PANEL_GET_POS_CMD: panel_get_pos_cmd(rx_header, rx_data, source);
				break;

			case PANEL_SET_PIXEL_CMD: panel_chain_pixel_cmd(rx_header, rx_data);
				break;

			case PANEL_SET_AUTO_CMD: panel_set_auto(rx_header, rx_data);
				break;

			case PANEL_GET_PIX_CMD: panel_get_pix_cmd(rx_header, rx_data);
				break;

			default: useless = 0;
				break;

		}

	}
		//Fill buffer with invalid data
	   // SPI2->DR = 0xAA;
		//Turn off SPI
		SPI2->CR1 &= ~(SPI_CR1_SPE);

}


uint8_t get_panel_structure(){

	uint8_t msg[5] = {0};
	msg[0] = PANEL_GET_POS_CMD;
	msg[1] = 0x0;
	msg[2] = 0;
	msg[3] = 3;
//	GPIOB->MODER &= ~(0x3 << 28);
//
//	GPIOB->MODER |= (0x02 << 28);
	SPI2->CR1 |= SPI_CR1_SPE;

    EXTI->IMR &= ~0x02;
    EXTI->IMR &= ~0x400;

    //TODO: Figure out why output at end of this function sometimes does not work
	set_panel_output();

	GPIOB->ODR &= ~(RIGHT_CS_Pin);
	GPIOB->ODR &= ~(LEFT_CS_Pin);

	delay_us(SPI_SETUP_TIME);
    uint8_t dumb[5];
    uint8_t dumb2[3];
    dumb2[0] = 0x1;
    dumb2[1] = 0x1;
    dumb2[2] = 0x1;
//    dumb[0] = SPI2->DR;
//    dumb[0] = SPI2->DR;


//	for(uint16_t i = 0; i < HEADER_SIZE; i++)
//	{
//		SPI2->DR = msg[i];
//	  while (!(SPI2->SR & SPI_SR_TXE_Msk ));
//		SPI2->DR;
//
//		delay_us(20);
//	}


	uint8_t ret = HAL_SPI_TransmitReceive(&hspi2,msg, (uint8_t *)dumb, HEADER_SIZE, TXRX_TIMEOUT);


	for(int i = 0; i < HEADER_SIZE; i++){

		if(dumb[i] != RESP_CODE){

			ret |= RESP_ERR;

		}

	}

	if(ret){

		GPIOB->ODR |= RIGHT_CS_Pin;
		GPIOB->ODR |= LEFT_CS_Pin;

		return ret;

	}

	ret = HAL_SPI_TransmitReceive(&hspi2,dumb2, (uint8_t *)dumb, 3, TXRX_TIMEOUT);
    EXTI->IMR &= ~0x02;

	GPIOB->ODR |= RIGHT_CS_Pin;
	GPIOB->ODR |= LEFT_CS_Pin;
//443-889-0376
	delay_us(25);
	//Set right gpio to interrupt
	  GPIO_InitTypeDef GPIO_InitStruct = {0};
	  GPIO_InitStruct.Pin = RIGHT_CS_Pin;
	  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
	  GPIO_InitStruct.Pull = GPIO_PULLUP;
	  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
	  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

	//Turn off SPI and wait for interrupt
	SPI2->CR1 &= ~SPI_CR1_SPE;

	uint32_t tx_start = HAL_GetTick();

	while((int_wait == 0) && ((HAL_GetTick() - tx_start) < 400)){}

	//TODO: Disable interrupt immediately
	if(int_wait == 1){
	int_wait = 0;

	SELECT_LEFT();
	UNSELECT_LEFT();
	SELECT_LEFT();
	UNSELECT_LEFT();

	SYSCFG->EXTICR[2] &= ~(0xF << 8);
//			GPIOB->ODR |= (RIGHT_CS_Pin);
	GPIOB->ODR &= ~(RIGHT_CS_Pin);
	uint8_t msg[4];
	msg[0] = 0xFF;
	msg[1] = 0xFF;
	msg[2] = 0xFF;
	msg[3] = 0xFF;
	//HAL_Delay(1);
	delay_us(SPI_SETUP_TIME);
	HAL_SPI_TransmitReceive(&hspi2,msg, rx_header, HEADER_SIZE, TXRX_TIMEOUT);
	//HAL_Delay(1);
	delay_us(SPI_SETUP_TIME);
	HAL_SPI_TransmitReceive(&hspi2,msg, rx_data, 3, TXRX_TIMEOUT);



	SELECT_LEFT();
	UNSELECT_LEFT();
	SELECT_LEFT();
	UNSELECT_LEFT();



	set_panel_output();

	UNSELECT_RIGHT();

//	GPIOB->MODER |= (RIGHT_CS_Pin);
//
//	GPIOB->ODR |= (RIGHT_CS_Pin);


	}
	//Disable int and configure GPIO as CS
	//Set left gpio to interrupt
//		HAL_GPIO_DeInit(GPIOB, LEFT_CS_Pin|RIGHT_CS_Pin|DOWN_CS_Pin);
//
//		  GPIO_InitStruct.Pin = LEFT_CS_Pin|RIGHT_CS_Pin|DOWN_CS_Pin;
//
//		  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
//		  GPIO_InitStruct.Pull = GPIO_PULLUP;
//		  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
//		  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
//		  GPIOB->BSRR = RIGHT_CS_Pin;
//		  GPIOB->BSRR = LEFT_CS_Pin;
//		  GPIOB->PUPDR |= 0x02 << 26;



		  return ret;
}


//uint8_t get_active_pixels(void){
//
//	//Send active pixel command
//
//	uint8_t head[4];
//	head[0] = PANEL_GET_PIX_CMD;
//	head[1] = 0;
//	head[2] = 0;
//	head[3] = 1;
//
//	uint8_t payload = RESP_CODE;
//
//SELECT_RIGHT();
//SELECT_LEFT();
//
//	HAL_Delay(1);
//	//delay_us(SPI_SETUP_TIME);
//	uint8_t ret = HAL_SPI_TransmitReceive(&hspi2, head, rx_header, HEADER_SIZE, TXRX_TIMEOUT);
//
//
//	for(int i = 0; i < HEADER_SIZE; i++){
//
//		if(rx_header[i] != RESP_CODE){
//
//			ret |= RESP_ERR;
//
//		}
//
//	}
//	if(ret){
//
//		UNSELECT_RIGHT();
//		UNSELECT_LEFT();
//		return ret;
//
//	};
//
//	ret = HAL_SPI_TransmitReceive(&hspi2,&payload, rx_data, 1, TXRX_TIMEOUT);
//
//	UNSELECT_RIGHT();
//	UNSELECT_LEFT();
//
//	//Get data from upstream
//
//	master_get_data(rx_header, rx_data, CS_RIGHT);
//
//	set_panel_output();
//
//	UNSELECT_RIGHT();
//
//	//Store rx data in appropriate global struct
//	//TODO: Consider the possibility of overflowing the array in this loop
//	for(int x = 0; x < (table.num_panels * 2); x++){
//
//		table.all_active_pixels[x] = rx_data[x];
//
//	}
//
//}




uint8_t get_active_pixels(void){

	//Send active pixel command

	uint8_t head[4];
	head[0] = PANEL_GET_PIX_CMD;
	head[1] = 0;
	head[2] = 0;
	head[3] = 1;

	uint8_t payload = RESP_CODE;


	SPI2->CR1 |= SPI_CR1_SPE;

    EXTI->IMR &= ~0x02;
    EXTI->IMR &= ~0x400;

    //TODO: Figure out why output at end of this function sometimes does not work
	set_panel_output();


SELECT_RIGHT();
SELECT_LEFT();

	//HAL_Delay(1);
	delay_us(SPI_SETUP_TIME);
	uint8_t ret = HAL_SPI_TransmitReceive(&hspi2, head, rx_header, HEADER_SIZE, TXRX_TIMEOUT);


	for(int i = 0; i < HEADER_SIZE; i++){

		if(rx_header[i] != RESP_CODE){

			ret |= RESP_ERR;

		}

	}
	if(ret){

		UNSELECT_RIGHT();
		UNSELECT_LEFT();
		return ret;

	};

	ret = HAL_SPI_TransmitReceive(&hspi2,&payload, rx_data, 1, TXRX_TIMEOUT);

	EXTI->IMR &= ~0x02;
	UNSELECT_RIGHT();
	UNSELECT_LEFT();

	//Set right gpio to interrupt
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin = RIGHT_CS_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
	HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

	//Turn off SPI and wait for interrupt
	SPI2->CR1 &= ~SPI_CR1_SPE;

	uint32_t tx_start = HAL_GetTick();

	while((int_wait == 0) && ((HAL_GetTick() - tx_start) < 400)){}

	//TODO: Disable interrupt immediately
	if(int_wait == 1){

	int_wait = 0;

	SYSCFG->EXTICR[2] &= ~(0xF << 8);

	SELECT_RIGHT();

	uint8_t msg[4];
	msg[0] = 0xFF;
	msg[1] = 0xFF;
	msg[2] = 0xFF;
	msg[3] = 0xFF;

	//HAL_Delay(1);

	delay_us(SPI_SETUP_TIME);

	HAL_SPI_TransmitReceive(&hspi2,msg, rx_header, HEADER_SIZE, TXRX_TIMEOUT);

	//HAL_Delay(1);

	delay_us(SPI_SETUP_TIME);

	int datalen = (rx_header[PAYLOAD_HIGH] << 8 | rx_header[PAYLOAD_LOW]);

	if(datalen > 800){

		datalen = 800;

	}

	HAL_SPI_TransmitReceive(&hspi2,msg, rx_data, datalen, TXRX_TIMEOUT);

	set_panel_output();

	UNSELECT_RIGHT();

	}

	//Note: Logical NOT circuit pulls down CS
	//This is a minor design flaw
	//As a result the CS must always be driven because the
	//internal pullup does not overcome the external pulldown
	set_cs_output(CS_RIGHT);


	//Get data from upstream

	master_get_data(rx_header, rx_data, CS_RIGHT);

	set_panel_output();

	UNSELECT_RIGHT();

	//Store rx data in appropriate global struct
	//TODO: Consider the possibility of overflowing the array in this loop
	for(int x = 0; x < (table.num_panels * 2); x++){

		table.all_active_pixels[x] = rx_data[x];

	}

}






void update_panel_pixels(){

if(table.num_panels > 0){
	//Set up header
	uint8_t msg[4] = {0};
	msg[0] = PANEL_SET_PIXEL_CMD;
	msg[1] = 0x0;
	msg[2] = 0;
	msg[3] = ((table.num_panels - 1) * 8) + 1;

	if(table.num_panels > 25){

		table.num_panels = 25;

	}

	//Turn on SPI
	SPI2->CR1 |= SPI_CR1_SPE;

	//Chip Select
	GPIOB->ODR &= ~(RIGHT_CS_Pin);
	GPIOB->ODR &= ~(LEFT_CS_Pin);

	//HAL_Delay(1);
    delay_us(SPI_SETUP_TIME);	//Send Header
	uint8_t ret = HAL_SPI_TransmitReceive(&hspi2, msg, rx_header, HEADER_SIZE, TXRX_TIMEOUT);


	for(int i = 0; i < HEADER_SIZE; i++){

		if(rx_header[i] != RESP_CODE){

			ret |= RESP_ERR;

		}

	}

	if(ret){
		GPIOB->ODR |= RIGHT_CS_Pin;
		GPIOB->ODR |= LEFT_CS_Pin;
	return;

	}

	//Send Payload
	ret = HAL_SPI_TransmitReceive(&hspi2,&panel_brightness, rx_data, 1, TXRX_TIMEOUT);

//	osStatus_t  mutex_status = osMutexAcquire(local_display_data_mutexHandle, TXRX_TIMEOUT);
//
//	if(mutex_status == osOK){


	ret = HAL_SPI_TransmitReceive(&hspi2,&table.colors_tx[8], rx_data, ((table.num_panels - 1) * 8), TXRX_TIMEOUT);

//	osMutexRelease(local_display_data_mutexHandle);
//
//	}


	GPIOB->ODR |= RIGHT_CS_Pin;
	GPIOB->ODR |= LEFT_CS_Pin;

	//Turn off SPI
	SPI2->CR1 &= ~SPI_CR1_SPE;
}

};

void set_ir_color(uint8_t * panel_bg){

	if(table.num_panels > 0){
		//Set up header
		uint8_t msg[4] = {0};
		msg[0] = PANEL_SET_AUTO_CMD;
		msg[1] = 0x0;
		msg[2] = 0;
		msg[3] = ((table.num_panels - 1) * 3);

		if(table.num_panels > 25){

			table.num_panels = 25;

		}

		//Turn on SPI
		SPI2->CR1 |= SPI_CR1_SPE;

		//Chip Select
		GPIOB->ODR &= ~(RIGHT_CS_Pin);
		GPIOB->ODR &= ~(LEFT_CS_Pin);

		//HAL_Delay(1);
	    delay_us(SPI_SETUP_TIME);	//Send Header
		uint8_t ret = HAL_SPI_TransmitReceive(&hspi2, msg, rx_header, HEADER_SIZE, TXRX_TIMEOUT);

		ret = HAL_SPI_TransmitReceive(&hspi2,&panel_bg[3], rx_data, ((table.num_panels - 1) * 3), TXRX_TIMEOUT);

		GPIOB->ODR |= RIGHT_CS_Pin;
		GPIOB->ODR |= LEFT_CS_Pin;

		//Turn off SPI
		SPI2->CR1 &= ~SPI_CR1_SPE;
	}

}


void panel_chain_pixel_cmd(uint8_t * header, uint8_t * data){


	//Set rx data to panel pixels

	panel_brightness = data[0];




	for(int x = 1; x < 9; x ++){

		table.colors_tx[x -1] = data[x];

	}



	//Shift position of brightness
	data[8] = data[0];

	//Subtract extrated data from header payload count
	uint16_t datalen = (header[PAYLOAD_HIGH] << 8 | header[PAYLOAD_LOW]);

	if(datalen > 800){

		datalen = 800;

	}

	datalen -= 8;
	frame_update = 1;

	if(datalen >= 9){

	header[PAYLOAD_HIGH] = (datalen >> 8);
	header[PAYLOAD_LOW] =  (datalen & 0xFF);


	set_spi2_mode(SPI2_MASTER);
	SPI2->CR1 |= SPI_CR1_SPE;


//	HAL_NVIC_DisableIRQ(EXTI15_10_IRQn);
//	//HAL_NVIC_DisableIRQ(EXTI1_IRQn);
//	HAL_NVIC_DisableIRQ(EXTI2_IRQn);

	//Chip select
	switch(panel_stream.upstream_dir){

	case CS_LEFT:	SELECT_LEFT();
		break;
	case CS_DOWN:	SELECT_DOWN();
		break;
	case CS_RIGHT:	SELECT_RIGHT();
		break;
	case CS_UP:		SELECT_UP();
		break;

	}

	//HAL_Delay(1);
    delay_us(SPI_SETUP_TIME);
	uint8_t holder[800];
	//Send Header

	uint8_t ret = HAL_SPI_TransmitReceive(&hspi2, header, holder, HEADER_SIZE, TXRX_TIMEOUT);

	//Send Payload
	ret = HAL_SPI_TransmitReceive(&hspi2, &data[8], holder, datalen, TXRX_TIMEOUT);


	switch(panel_stream.upstream_dir){

	case CS_LEFT:	UNSELECT_LEFT();
		break;
	case CS_DOWN:	UNSELECT_DOWN();
		break;
	case CS_RIGHT:	UNSELECT_RIGHT();
		break;
	case CS_UP:		UNSELECT_UP();
		break;

	}
	SPI2->CR1 &= ~SPI_CR1_SPE;




	set_spi2_mode(SPI2_SLAVE);

	set_panel_it(panel_stream.downstream_dir);

	frame_update = 1;

}


}




void panel_get_pix_cmd(uint8_t * header, uint8_t * data){

	//Check Payload just for added sanity

	if(data[0] !=  RESP_CODE){

		return;

	}

	//Note: Logical NOT circuit pulls down CS
	//This is a minor design flaw
	//As a result the CS must always be driven because the
	//internal pullup does not overcome the external pulldown
	//if(panel_stream.panel_id > 1){

	set_cs_output(panel_stream.downstream_dir);

	//}

	//Increment header ID
	header[1] = panel_stream.panel_id;

	//Send command up the chain if upstream exists
	if(panel_stream.upstream_dir != LAST_PANEL){

		send_data_up(header, data, panel_stream.upstream_dir);

		//Wait for response back store response in global rx data variable
		master_get_data(header, rx_data, panel_stream.upstream_dir);

	}

	//Add panels data to array
	uint16_t datalen = (header[PAYLOAD_HIGH] << 8 | header[PAYLOAD_LOW]);

	if(datalen > 800){

		datalen = 800;

	}

	rx_data[datalen] 	 = (table.panel->active_pixels) >> 8;

	rx_data[datalen + 1] = (table.panel->active_pixels) & 0xFF;

	//Increase payload size in header

	datalen += 2;

	header[PAYLOAD_HIGH] = datalen >> 8;
	header[PAYLOAD_LOW]  = datalen & 0xF;

	//Send led data down
	send_data_down(header, rx_data, panel_stream.downstream_dir);


}

void panel_set_auto(uint8_t * header, uint8_t * data){


	//If not set auto command return
	if(rx_header[HEADER_CMD] != PANEL_SET_AUTO_CMD){

		return;

	}


	table.panel->red = data[0];
	table.panel->green = data[1];
	table.panel->blue = data[2];

	if(
	data[0] == 0 &&
	data[1] == 0 &&
	data[2] == 0
	){

		table.panel->mode = application_mode;

	}
	else{

		table.panel->mode = auto_mode;

	}


if(table.panel->red > 100){

	table.panel->red = 100;

}

if(table.panel->green > 100){

	table.panel->green = 100;

}

if(table.panel->blue > 100){

	table.panel->green = 100;

}

	//Subtract extrated data from header payload count
	uint16_t datalen = (header[PAYLOAD_HIGH] << 8 | header[PAYLOAD_LOW]);

	if(datalen > 300){

		datalen = 300;

	}

	datalen -= 3;

	if(datalen >= 3){

	header[PAYLOAD_HIGH] = (datalen >> 8);
	header[PAYLOAD_LOW] =  (datalen & 0xFF);

	set_spi2_mode(SPI2_MASTER);

	SPI2->CR1 |= SPI_CR1_SPE;

	//Chip select
	switch(panel_stream.upstream_dir){

	case CS_LEFT:	SELECT_LEFT();
		break;
	case CS_DOWN:	SELECT_DOWN();
		break;
	case CS_RIGHT:	SELECT_RIGHT();
		break;
	case CS_UP:		SELECT_UP();
		break;

	}

	//HAL_Delay(1);
    delay_us(SPI_SETUP_TIME);
	uint8_t holder[800];
	//Send Header

	uint8_t ret = HAL_SPI_TransmitReceive(&hspi2, header, holder, HEADER_SIZE, TXRX_TIMEOUT);

	//Send Payload
	ret = HAL_SPI_TransmitReceive(&hspi2, &data[3], holder, datalen, TXRX_TIMEOUT);


	switch(panel_stream.upstream_dir){

	case CS_LEFT:	UNSELECT_LEFT();
		break;
	case CS_DOWN:	UNSELECT_DOWN();
		break;
	case CS_RIGHT:	UNSELECT_RIGHT();
		break;
	case CS_UP:		UNSELECT_UP();
		break;

	}
	SPI2->CR1 &= ~SPI_CR1_SPE;

	set_spi2_mode(SPI2_SLAVE);

	set_panel_it(panel_stream.downstream_dir);

}


}



void set_panel_bg(uint8_t panel_num, uint8_t * bg, uint8_t red, uint8_t green, uint8_t blue){



	bg[(panel_num * 3)] = red;
	bg[(panel_num * 3)+1] = green;
	bg[(panel_num * 3)+2] = blue;


}











void play_pong(void){

	int paddleX = 0;
	int paddleY = 0;
	int oldPaddleX, oldPaddleY;
	int ballDirectionX = 1;
	int ballDirectionY = 1;
	int score = 0;

	int ballSpeed = 100; // lower numbers are faster

	int ballX = 3;
    int ballY = 0;

	int myWidth = (table.x * 4);
	int myHeight = (table.y * 4);

	uint32_t f_update = 0;
uint8_t padx = 0;
  /* Infinite loop */
  for(;;)
  {

	    get_local_ir();


		get_active_pixels();

		show_ir();
		delay_us(50);

		uint8_t line_h = 0;

		uint8_t line_l = 0xFF;

		for(int x = 0; x < (table.x * 4); x++){

			uint8_t bite = 0;

			int b_read = 0;

			if(is_pixel_active(&table, x, 0)){

				if(x > line_h){

					line_h = x;
				}
				if(x < line_l){

					line_l = x;
				}

			}

		}

	//Clear Paddle Graphic
	set_collective_pixels(paddleX ,paddleY , 0, &table);
	set_collective_pixels(paddleX +1,paddleY , 0, &table);
	set_collective_pixels(paddleX +2,paddleY , 0, &table);
	set_collective_pixels(paddleX +3,paddleY , 0, &table);

	set_collective_pixels(ballX,ballY , 0, &table);

	//Get Paddle position

	paddleX = (line_l+line_h+1)/2;



	paddleX -= 2;

//	padx++;
//
//	if(padx > ((table.x - 1) * 4)){
//
//		padx = 0;
//	}
//
//	paddleX = padx;

	if(paddleX > ((table.x - 1) * 4))
	{
		paddleX = ((table.x - 1) * 4);
	}


	if(paddleX < 0)
	{
		paddleX = 0;
	}

//	uint8_t joystick = 0;
//
//	osStatus_t status = osMessageQueueGet(joystickHandle, &joystick, NULL, 0U);
//
//	if(status == osOK){
//
//				switch(joystick){
//
//				case '1': paddleX++;
//						  if(paddleX > (((table.x - 1) * 4) - 1))(paddleX = (((table.x - 1) * 4) - 1));
//						  break;
//				case '2': paddleX--;
//						  if(paddleX < 0)(paddleX = 0);
//						  break;
//				case '5': osThreadSuspend(pong_taskHandle);
//				          osThreadResume(central_taskHandle);
//
//				default:
//					break;
//				}
//	}

    set_collective_pixels(paddleX ,paddleY , green, &table);
    set_collective_pixels(paddleX +1,paddleY , green, &table);
    set_collective_pixels(paddleX +2,paddleY , green, &table);
    set_collective_pixels(paddleX +3,paddleY , green, &table);




    //Update Ball position and show on screen

    if ((HAL_GetTick() - f_update) > 100) {

    	f_update = HAL_GetTick();

    	  // if the ball goes offscreen, reverse the direction:
    	  if (ballX > (myWidth - 2) || (ballX == 0)) {
    	    ballDirectionX = -ballDirectionX;
    	  }

    	  if (ballY > (myHeight - 2) || (ballY < 0)) {
    	    ballDirectionY = -ballDirectionY;
    	  }

    	  // check if the ball and the paddle occupy the same space on screen
    	  if (inPaddle(ballX, ballY, paddleX, paddleY, 4, 1)) {
    	    if(ballX == paddleX && ballY == paddleY){
    	    ballDirectionX = -ballDirectionX;
    	    ballDirectionY = -ballDirectionY;
    	    }
    	    else if(ballX == paddleX + 3 && ballY == paddleY){
    	      ballDirectionX = ballDirectionX;
    	      ballDirectionY = -ballDirectionY;
    	    }
    	    else if(ballX == paddleX + 1 && ballY == paddleY){
    	      ballDirectionX = -ballDirectionX;
    	      ballDirectionY = -ballDirectionY;
    	    }
    	    else if(ballX == paddleX + 2 && ballY == paddleY){
    	      ballDirectionX = ballDirectionX;
    	      ballDirectionY = -ballDirectionY;
    	    }
    	  }

    	  // update the ball's position
    	  ballX += ballDirectionX;
    	  ballY += ballDirectionY;



    }
	  // draw the ball's current position
	  set_collective_pixels(ballX,ballY , red, &table);

    update_panel_pixels();

    //Get active Pixels

    if(ballY < 0){

      ballX = 3;
      ballY = 0;
    }

	set_panel_clut(pixels, table.colors_tx);

delay_us(50);
show_pixels(pixels);
delay_us(50);
	clear_pixels(pixels);
	HAL_Delay(15);

  }




}











void get_local_ir(void){

	for(int x = 0; x < 4; x ++){

		for(int y = 0; y < 4; y ++){

			read_ir_sensor(x,y, aval);
			if(aval[x][y] > table.led_sensitivity){

				led_active[x][y] = HAL_GetTick();

				table.panel->active_pixels |= (0x01 << ((x *4) + y));

			}
		}

	}

}

void show_ir(void){

	for(int x = 0; x < 4; x ++){

		for(int y = 0; y < 4; y ++){




			int tim = (HAL_GetTick() - led_active[x][y]);

			//if(tim > FADE_TIME){tim = FADE_TIME;}

			if(tim > FADE_TIME){

				led_color[x][y].red = 0;

				led_color[x][y].green = 0;

				led_color[x][y].blue = 0;

				table.panel->active_pixels &= ~(0x01 << ((x *4) + y));

			}
			else{




				uint32_t rMap = map(tim, 0, FADE_TIME, led_color_set[x][y].red, 0);

				uint32_t  gMap = map(tim, 0, FADE_TIME, led_color_set[x][y].green, 0);

				uint32_t bMap = map(tim, 0, FADE_TIME, led_color_set[x][y].blue, 0);


				led_color[x][y].red = rMap;

				led_color[x][y].green = gMap;

				led_color[x][y].blue = bMap;


			}


		}

	}

delay_us(20);


}



uint8_t  inPaddle(int x, int y, int rectX, int rectY, int rectWidth, int rectHeight) {
	uint8_t result = 0;

  if ((x >= rectX && x <= (rectX + rectWidth)) &&
      (y >= rectY && y <= (rectY + rectHeight))) {
    result = 1;
  }

  return result;
}


uint8_t is_pixel_active(struct panel_assembly * table, uint16_t x, uint16_t y){


	//If in first panel
	if(x < 4 && y < 4){

		if(x < 2){

			return bitread(table->panel->active_pixels, (((x)*4)));

		}

		else if(x >= 2 && x < 4){

			return bitread((table->panel->active_pixels >> 8), (((x-2)*4)));

		}


	}
	//Get panel number
	uint32_t panel_num = ((y / 4) * table->x);

	//If odd row
	if((y/4) % 2){

		panel_num += (table->x - (x/4));

	}
	else{

		panel_num += ((x/4) + 1);

	}

	int pandex = (table->num_panels - panel_num) * 2;

	uint8_t xmod = x % 4;
	uint8_t ymod = y % 4;

	uint8_t retval = 0;

	switch(xmod){

	case 0: retval = ((table->all_active_pixels[pandex + 2] >> ymod) & 0x01);

		break;

	case 1: retval = ((table->all_active_pixels[pandex + 2] >> (ymod + 4)) & 0x01);
		break;

	case 2: retval = ((table->all_active_pixels[pandex + 1] >> ymod) & 0x01);
		break;

	case 3: retval = ((table->all_active_pixels[pandex + 1] >> (ymod + 4)) & 0x01);
		break;

	default:
		break;

	}

	return retval;

}

void set_cs_output(uint8_t csPin){

	switch(csPin){

	case CS_LEFT:

		GPIOB->PUPDR |= ((0x01) << 2);

		GPIOB->OSPEEDR |= ((0x03) << 2);

		//Set ODR
		GPIOB->ODR |= ((0x01) << 1);

		//Set Mode
		GPIOB->MODER |= ((0x01) << 2);

		break;

	case CS_RIGHT:

		GPIOB->PUPDR |= ((0x01) << 20);

		GPIOB->OSPEEDR |= ((0x03) << 20);

		//Set ODR
		GPIOB->ODR |= ((0x01) << 10);

		//Set Mode
		GPIOB->MODER |= ((0x01) << 20);

		break;

	case CS_UP:

		GPIOA->PUPDR |= ((0x01) << 20);

		GPIOA->OSPEEDR |= ((0x03) << 20);

		//Set ODR
		GPIOA->ODR |= ((0x01) << 10);

		//Set Mode
		GPIOA->MODER |= ((0x01) << 20);

		break;

	case CS_DOWN:

		GPIOB->PUPDR |= ((0x01) << 4);

		GPIOB->OSPEEDR |= ((0x03) << 4);

		//Set ODR
		GPIOB->ODR |= ((0x01) << 2);

		//Set Mode
		GPIOB->MODER |= ((0x01) << 4);

		break;

	default:
		break;

	}


}


void get_header_payload(uint8_t source){

	set_spi2_mode(SPI2_SLAVE);

	//Turn on SPI
	SPI2->CR1 |= SPI_CR1_SPE;

	//Get Header
	get_panel_header(rx_header);

	//Process command from geader
	if((rx_header[HEADER_CMD] == PANEL_GET_POS_CMD)|
	   (rx_header[HEADER_CMD] == PANEL_SET_PIXEL_CMD)|
	   (rx_header[HEADER_CMD] == PANEL_SET_AUTO_CMD)|
	    rx_header[HEADER_CMD] == PANEL_GET_PIX_CMD){

		//Parse data length
		uint16_t datalen = (rx_header[PAYLOAD_HIGH] << 8 | rx_header[PAYLOAD_LOW]);

		if(datalen > 800){

			datalen = 800;

		}

		get_panel_payload(datalen, rx_data);

		switch(rx_header[HEADER_CMD]){

			case PANEL_GET_POS_CMD: int_source = source;
									int_command = get_position;
				break;

			case PANEL_SET_PIXEL_CMD: int_source = source;
									  int_command = set_pixels;
				break;

			case PANEL_SET_AUTO_CMD: int_source = source;
								     int_command = set_auto;
			break;

			case PANEL_GET_PIX_CMD: int_source = source;
									int_command = get_pixels;
				break;

			default:
				break;

		}

	}

		//Turn off SPI
//		SPI2->CR1 &= ~(SPI_CR1_SPE);



}
