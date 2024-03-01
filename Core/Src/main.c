/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "led_panel.h"
#include "ws2812_spi_lib.h"
#include "stm32f4xx_it.h"

#include "fonts.h"


#define CALIB
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define THIS_IS_MASTER		0
#define STANDALONE_PANEL	1

#if THIS_IS_MASTER == 1

	uint8_t spi2_mode = SPI2_MASTER;
	uint8_t panel_type = PANEL_CENTRAL;

#else

	uint8_t spi2_mode = SPI2_SLAVE;
	uint8_t panel_type = PANEL_PERIPH;

#endif

uint8_t panel_structure[3];
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
//Buffers for TX/RX
uint8_t rx_header[4]   = {0};
uint8_t rx_data[800]   = {0};
uint8_t resp_code[800] = {0};


//Buffer for CLUT Data
uint8_t clut_ram[8] = {0};


//Analog read buffer
uint16_t aval[4][4] = {0};

//Interrupt trigger flag
uint8_t int_trigger = 0;

uint8_t cmd_ready = 0;

//RGB for this panel
struct Color local_panel_colors[4][4] = {0};
//TODO: Replace led_color with local_panel_colors
struct Color led_color[4][4] = {0};
struct Color led_color_set[4][4] = {0};

//Array for last time sensor activated
uint32_t led_last_active[4][4] = {0};
uint32_t led_active[4][4] = {0};

//Binary map of active pixels
uint16_t active_pixel_map = 0;
uint16_t pixel_map = 0;

uint8_t ble_dma = 0;

struct panel_assembly table = {0};

struct Panel_stream panel_stream = {0};

struct panel_profile panel = {0};



uint8_t dummy[100] = {0};


uint8_t ds_int = 0;


uint8_t frame_update = 0;

struct Color ir_color = {0};

uint8_t got_structure = 0;

uint8_t int_source = 0;

uint8_t int_command = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

//Used to switch between SPI Master and Slave

void delay_us(uint16_t us);
int bitread(uint8_t byte, uint8_t bit);

void display_8x8_char(char letter, struct panel_assembly * tab);


void display_eye(uint8_t num);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int colorMode = 0;
uint32_t colorModeTime = 0;

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */

	HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_SPI1_Init();
  MX_USART2_UART_Init();
  MX_ADC1_Init();
  MX_SPI2_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */


  //Set default panel colors
  for(int y = 0; y < 4; y++){

	  for(int x = 0; x < 4; x++){

//		  if(y%2){
//
//			  if(x%2){
//
//				  led_color_set[x][y].red = 30;
//
//			  }
//			  else{
//
//				  led_color_set[x][y].green = 30;
//
//			  }
//
//		  }
//		  else{
//
//			  if(x%2){
//
//				  led_color_set[x][y].green = 30;
//
//			  }
//			  else{
//
//				  led_color_set[x][y].red = 30;
//
//			  }
//
//		  }

		  //led_color_set[x][y].blue = 50;
		  led_color_set[x][y].red = 50;

	  }

  }

  for(int x = 0; x < 800; x++){

	  resp_code[x] = RESP_CODE;

  }

  table.panel = &panel;

  if(panel_type == PANEL_CENTRAL){

	  set_spi2_mode(SPI2_MASTER);

	  //Configure GPIOS fo CS output
	  GPIO_InitTypeDef GPIO_InitStruct = {0};

	  HAL_GPIO_DeInit(GPIOB, LEFT_CS_Pin|RIGHT_CS_Pin|DOWN_CS_Pin);

	  GPIO_InitStruct.Pin = LEFT_CS_Pin|RIGHT_CS_Pin|DOWN_CS_Pin;

	  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	  GPIO_InitStruct.Pull = GPIO_PULLUP;
	  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
	  GPIOB->BSRR = RIGHT_CS_Pin;
	  GPIOB->BSRR = LEFT_CS_Pin;
	  GPIOB->PUPDR |= 0x02 << 26;
	  //HAL_NVIC_DisableIRQ(EXTI0_IRQn);


  }else{

	  	table.colors_tx = clut_ram;
		GPIOB->MODER &= ~(0x3F << 26);
		GPIOB->PUPDR &= ~(0x3F << 26);
		GPIOB->PUPDR |= 0x02 << 28;
		GPIOB->PUPDR |= 0x02 << 26;
	//  HAL_Delay(1000);


  }

  //Set LED sensitivity
  table.led_sensitivity = 120;
  table.panel->red = 0;
  table.panel->green = 10;
  table.panel->blue = 10;
  ir_color.green = 20;
  ir_color.blue = 20;
  //RESET HM11 and enable LEDs
  HAL_GPIO_WritePin(GPIOB, HM_RST___LED_ON_Pin, GPIO_PIN_SET);
  HAL_Delay(500);
  HAL_GPIO_WritePin(GPIOB, HM_RST___LED_ON_Pin, GPIO_PIN_RESET);
  HAL_Delay(500);
  HAL_GPIO_WritePin(GPIOB, HM_RST___LED_ON_Pin, GPIO_PIN_SET);

  HAL_Delay(100);
  struct Led_pixels pixels[NUM_PIXELS];

  clear_pixels(pixels);
	HAL_Delay(10);
	show_pixels(pixels);
	HAL_Delay(10);



uint8_t panel_pix_in[16*3] = {0};
uint8_t panel_pix_out[16*3] = {0};


uint32_t my_tick = 0;
uint16_t aval[4][4];



uint8_t container = 0;
int round = 0;
uint32_t last_update = 0;

HAL_TIM_Base_Start(&htim1);


//Clear current LEDs
clear_pixels(pixels);
show_pixels(pixels);

HAL_Delay(500);

#if THIS_IS_MASTER == 1

HAL_Delay(15000);

#endif

int last_cnt = 0;
int diff = 0;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	  char my_name[] = "RODNEY";

//Read all IR Values
if(panel_type == PANEL_CENTRAL){


HAL_Delay(3000);

//while(1){
	  got_structure = get_panel_structure();

	  table.num_panels = rx_data[0];
	  table.x = rx_data[1];
	  table.y = rx_data[2];

	  if(table.num_panels == 0){

	  	  table.num_panels = 1;

	  }
	  if(table.num_panels > 25 ){

	  	table.num_panels = 25;

	  }

	  //allocate memory for tx pixels
	  int len = 8 * (table.x * table.y);

	  free(table.colors_tx);

	  table.colors_tx = (uint8_t *)malloc(sizeof(uint8_t) * len);

	  free(table.all_active_pixels);

	  table.all_active_pixels = (uint8_t *)malloc(sizeof(uint16_t) * table.num_panels);

	  for(int i = 0; i < sizeof(uint16_t) * table.num_panels; i++)
	  {

		  table.all_active_pixels[i] = 0;

	  }

	  for(int i = 0; i < len; i++){

	  	  table.colors_tx[i] = 0;

	  }



	  for(int x = 0; x < table.num_panels; x++){


		  set_pixel_color(pixels, x, 20, 0, 0);

		  if(last_cnt != table.num_panels){

			  set_pixel_color(pixels, x, 0, 0, 20);

		  }
	  }
	    HAL_Delay(1);
		show_pixels(pixels);
		HAL_Delay(5);
		clear_pixels(pixels);
		HAL_Delay(5);
		if(last_cnt != table.num_panels){

			HAL_Delay(30);
			last_cnt = table.num_panels;

		}


//}

	get_local_ir();

	show_ir();

	set_panel_pixels(pixels, led_color);

	set_panel_clut(pixels, table.colors_tx);
	delay_us(10);
//	HAL_Delay(1);
	show_pixels(pixels);
	delay_us(50);
//	HAL_Delay(1);
	clear_pixels(pixels);
	delay_us(10);

#ifdef CALIB
if(HAL_GetTick() - last_update > 200){

		last_update = HAL_GetTick();




//	uint8_t chara[24];
//
//	for (int j; j < 24; j++){
//
//		chara[j] = table.colors_tx[j];
//	}











if(!got_structure){



while(1){

//	HAL_Delay(1);

//	get_active_pixels();
//
//	for(int x = 0; x < 4; x++){
//
//		for(int y = 0; y < 4; y++){
//
//			uint8_t bite = 0;
//
//			int b_read = 0;
//
//			if(x < 2){
//
//				bite = table.all_active_pixels[2];
//
//				b_read = bitread(bite, ((x*4) + y));
//
//			}
//			else{
//
//				bite = table.all_active_pixels[1];
//
//				b_read = bitread(bite, (((x - 2)*4) + y));
//
//			}
//
//
//			if(b_read){
//
//				set_collective_pixels((x) ,y, 0x5, &table);
//
//			}
//			else{
//
//				set_collective_pixels((x) ,y, 0x0, &table);
//
//			}
//
//
//
//		}
//
//	}

//		for(int i = 0; i < 6; i ++){
//
//	    display_8x8_char(my_name[i], &table);
//		set_panel_clut(pixels, table.colors_tx);
//		show_pixels(pixels);
//		update_panel_pixels();
//		clear_pixels(pixels);
//		HAL_Delay(900);
//		}

	play_pong();

	set_panel_clut(pixels, table.colors_tx);
	delay_us(10);
//	HAL_Delay(1);
	show_pixels(pixels);
	delay_us(50);
//	HAL_Delay(1);
	clear_pixels(pixels);
	delay_us(10);

}
//
//	set_collective_pixels(5,0, 0x2, &table);
//	set_collective_pixels(0,0, 0x2, &table);
//
//	update_panel_pixels();
//
//	set_panel_clut(pixels, table.colors_tx);
//
//	show_pixels(pixels);
//
//	clear_pixels(pixels);



//while(1){
//	HAL_Delay(100);
//
//	for(int i = 0; i < 6; i ++){
//
//  display_8x8_char(my_name[i]);
//	set_panel_clut(pixels, table.colors_tx);
//	show_pixels(pixels);
//	update_panel_pixels();
//	clear_pixels(pixels);
//	HAL_Delay(900);
//	}
//	}







}











}
#endif



show_pixels(pixels);
}


if(panel_type == PANEL_PERIPH){

#if STANDALONE_PANEL == 0

////Wait for int trigger flag
	while(!int_command){}

	switch(rx_header[HEADER_CMD]){

		case PANEL_GET_POS_CMD: panel_get_pos_cmd(rx_header, rx_data, int_source);
			break;

		case PANEL_SET_PIXEL_CMD: panel_chain_pixel_cmd(rx_header, rx_data);
			break;

		case PANEL_SET_AUTO_CMD: panel_set_auto(rx_header, rx_data);
			break;

		case PANEL_GET_PIX_CMD: panel_get_pix_cmd(rx_header, rx_data);
			break;

		default: //useless = 0;
			break;

	}

	SPI2->CR1 &= ~(SPI_CR1_SPE);

	int_command = 0;

	int_source = 0;

#endif

	if( 1){

#if STANDALONE_PANEL == 0

		//Set panel pixels to clut received from master
		set_panel_clut(pixels, table.colors_tx);

#endif

		//Get IR map from THIS panel
		get_local_ir();

		//load the LED_Color array with calculated color values
		show_ir();

#if STANDALONE_PANEL == 1

		//transfer the led_color to the pixels array
		set_panel_pixels(pixels, led_color);

#endif

		HAL_Delay(1);

		//send pixels out to LEDs
		show_pixels(pixels);

		HAL_Delay(1);

		//clear pixels array
		clear_pixels(pixels);

		frame_update = 0;

		if(HAL_GetTick() - colorModeTime > 5000)
		{

			colorModeTime = HAL_GetTick();

			colorMode++;

			if(colorMode >= 6){

				colorMode = 0;

			}

		}

		//Set default panel colors
		for(int y = 0; y < 4; y++){

		  for(int x = 0; x < 4; x++){

			  led_color_set[x][y].red = 0;
			  led_color_set[x][y].green = 0;
			  led_color_set[x][y].blue = 0;

			  switch(colorMode){

				  case 0: led_color_set[x][y].red   = 50;
					  break;

				  case 1: led_color_set[x][y].blue  = 50;
					  break;

				  case 2: led_color_set[x][y].green = 50;
					  break;

				  case 3: led_color_set[x][y].red   = 50;
				  	  	  led_color_set[x][y].green = 50;
					  break;

				  case 4: led_color_set[x][y].red  = 50;
				  	  	  led_color_set[x][y].blue = 50;
					  break;

				  case 5: led_color_set[x][y].green = 50;
				  	  	  led_color_set[x][y].blue  = 50;
					  break;

			  }


		  }

		}

	}

}

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage 
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 96;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef * huart2){


	ble_dma = 1;




}



void HAL_SPI_RxCpltCallback (SPI_HandleTypeDef * hspi){

//	  GPIOA->BSRR = UP_CS_Pin;
////	 HAL_SPI_DMAStop(&hspi2);
//int ret;
//	 if(dummy[0] == 0xC && dummy[1] == 0xD && dma_stopped == 2){
//
//
//		 ret = HAL_SPI_TransmitReceive(&hspi2, (uint8_t *)led_color,(uint8_t *)rx_color, (16*3), 1000);
//		dma_stopped = 1;
//
////	HAL_SPI_Receive_DMA (&hspi2, (uint8_t *)rx_color, (16*3));
//
//	 }
//	 else if(dma_stopped == 1){
//
//		dma_stopped = 0;
//
//		//Turn off SPI
//	//	SPI2->CR1 &= ~(SPI_CR1_SPE);
//
//	 }

//	memcpy(rx_spi, spirx, sizeof(struct Color) * 16);

}


void set_spi2_mode(uint8_t mode){

	//If master mode
	if(mode == SPI2_MASTER){

		spi2_mode = SPI2_MASTER;
//
//		RCC->APB1ENR |= RCC_APB1ENR_SPI2EN;
//
//		RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
//
//		GPIOB->MODER  |= (GPIO_MODER_MODE13_1 | GPIO_MODER_MODE14_1 |  GPIO_MODER_MODE15_1);
//		GPIOB->AFR[1] |= (GPIO_AFRH_AFSEL13_0 | GPIO_AFRH_AFSEL13_2 | GPIO_AFRH_AFSEL14_0 | GPIO_AFRH_AFSEL14_2 | GPIO_AFRH_AFSEL15_0 | GPIO_AFRH_AFSEL15_2);
//
//		SPI2->CR1 = (SPI_CR1_MSTR |(0x03 << 5)| SPI_CR1_SSI | SPI_CR1_SSM);
//		SPI2->CR2 = 0x0;
//		SPI2->CR1 &= ~SPI_CR1_SPE;




		 // HAL_SPI_DeInit(&hspi2);

			__HAL_RCC_SPI2_CLK_ENABLE();

//		  GPIO_InitTypeDef GPIO_InitStruct = {0};
//
//		    GPIO_InitStruct.Pin = SPI2_CLK_Pin|SPI2_MISO_Pin|SPI2_MOSI_Pin;
//		    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
//		    GPIO_InitStruct.Pull = GPIO_NOPULL;
//		    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
//		    GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
//		    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

			//while (!(SPI2->SR & SPI_SR_TXE_Msk ));

			//Reset SPI Peripheral
			RCC->APB1RSTR |= RCC_APB1RSTR_SPI2RST;
			RCC->APB1RSTR &= ~RCC_APB1RSTR_SPI2RST;

			SPI2->CR1 |= ~SPI_CR1_SPE;

			  hspi2.Instance = SPI2;
			  hspi2.Init.Mode = SPI_MODE_MASTER;
			  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
			  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
			  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
			  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
			  hspi2.Init.NSS = SPI_NSS_SOFT;
			  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
			  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
			  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
			  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
			  hspi2.Init.CRCPolynomial = 10;

			  if (HAL_SPI_Init(&hspi2) != HAL_OK)
			  {
			    Error_Handler();
			  }
			  //GPIOB->MODER &= ~(0x3 << 28);
		  set_panel_output();
	}

	else if(mode == SPI2_SLAVE){

		spi2_mode = SPI2_SLAVE;

		RCC->APB1RSTR |= RCC_APB1RSTR_SPI2RST;
		RCC->APB1RSTR &= ~RCC_APB1RSTR_SPI2RST;

		GPIO_InitTypeDef GPIO_InitStruct = {0};

		HAL_SPI_MspDeInit(&hspi2);

		__HAL_RCC_SPI2_CLK_ENABLE();

		GPIO_InitStruct.Pin = SPI2_CLK_Pin|SPI2_MISO_Pin|SPI2_MOSI_Pin;
		GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
		GPIO_InitStruct.Pull = GPIO_PULLDOWN;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
		GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
		HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);


		hspi2.Instance = SPI2;
		hspi2.Init.Mode = SPI_MODE_SLAVE;
		hspi2.Init.Direction = SPI_DIRECTION_2LINES;
		hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
		hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
		hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
		hspi2.Init.NSS = SPI_NSS_SOFT;
		hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
		hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
		hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
		hspi2.Init.CRCPolynomial = 10;
		if (HAL_SPI_Init(&hspi2) != HAL_OK)
		{
		Error_Handler();
		}



	}

}


void delay_us(uint16_t us)
{
	uint32_t timval = 0;
	TIM1->CNT = 0;
	while (TIM1->CNT < us){

		timval = TIM1->CNT;

	};  // wait for the counter to reach the us input in the parameter
}



int bitread(uint8_t byte, uint8_t bit){

	int retval = (byte >> bit) & (0x01);

	return retval;

};


void display_8x8_char(char letter, struct panel_assembly * tab){


	for(int y = 0; y < 8; y++){

		for(int x = 0; x < 8; x++){

			if(bitread(font8x8_basic[((uint8_t)letter)][y], x)){

				set_collective_pixels(x,7-y, 0x1, tab);

			}
			else{

				set_collective_pixels(x,7-y, 0x0, tab);

			}

		}

	}

}

void display_eye(uint8_t num){


	for(int y = 0; y < 8; y++){

		for(int x = 0; x < 8; x++){

			if(bitread(eye_map[((uint8_t)num)][y], x)){

				set_collective_pixels(x,7-y, 0xF, &table);

			}
			else{

				set_collective_pixels(x,7-y, 0x0, &table);

			}

		}

	}

}

/* USER CODE END 4 */

 /**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM2 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM2) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
