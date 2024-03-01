/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "ws2812_spi_lib.h"
//#include "cmsis_os.h"

#define PANEL_CENTRAL	0
#define PANEL_PERIPH	1

#define SPI2_SLAVE		0
#define SPI2_MASTER		1
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */


extern uint8_t spi2_mode;
extern uint8_t panel_type;

extern uint8_t dummy[100];
extern uint8_t panel_pix_in[16*3];
extern uint8_t panel_pix_out[16*3];
extern uint16_t aval[4][4];
extern int dma_stopped;
extern uint32_t led_active[4][4];

struct Color{

	uint8_t red;
	uint8_t green;
	uint8_t blue;

};

extern struct Color led_color[4][4];
extern struct Color led_color_set[4][4];
extern struct Color rx_color[4][4];
extern struct Color ir_color;
extern uint8_t ds_int;
extern uint32_t int_wait;
extern uint8_t frame_update;
extern uint8_t resp_code[800];

enum int_commands{
	get_position = 1,
	set_pixels,
	set_auto,
	get_pixels
};

extern uint8_t int_trigger;

extern uint8_t cmd_ready;

extern uint8_t ble_dma;

extern uint8_t int_source;

extern uint8_t int_command;

//extern osMutexId_t local_display_data_mutexHandle;
//extern const osMutexAttr_t local_display_data_mutex_attributes;
/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
#define UP_CS_Pin_Rst   0x4000000

extern void set_spi2_mode(uint8_t mode);

extern void delay_us(uint16_t us);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define HM_STATUS_Pin GPIO_PIN_0
#define HM_STATUS_GPIO_Port GPIOA
#define HM_STATUS_EXTI_IRQn EXTI0_IRQn
#define A4_Pin GPIO_PIN_1
#define A4_GPIO_Port GPIOA
#define RX4_Pin GPIO_PIN_4
#define RX4_GPIO_Port GPIOA
#define A2_Pin GPIO_PIN_5
#define A2_GPIO_Port GPIOA
#define A3_Pin GPIO_PIN_6
#define A3_GPIO_Port GPIOA
#define RX3_Pin GPIO_PIN_7
#define RX3_GPIO_Port GPIOA
#define A1_Pin GPIO_PIN_0
#define A1_GPIO_Port GPIOB
#define LEFT_CS_Pin GPIO_PIN_1
#define LEFT_CS_GPIO_Port GPIOB
#define LEFT_CS_EXTI_IRQn EXTI1_IRQn
#define DOWN_CS_Pin GPIO_PIN_2
#define DOWN_CS_GPIO_Port GPIOB
#define DOWN_CS_EXTI_IRQn EXTI2_IRQn
#define RIGHT_CS_Pin GPIO_PIN_10
#define RIGHT_CS_GPIO_Port GPIOB
#define RIGHT_CS_EXTI_IRQn EXTI15_10_IRQn
#define HM_RST___LED_ON_Pin GPIO_PIN_12
#define HM_RST___LED_ON_GPIO_Port GPIOB
#define SPI2_CLK_Pin GPIO_PIN_13
#define SPI2_CLK_GPIO_Port GPIOB
#define SPI2_MISO_Pin GPIO_PIN_14
#define SPI2_MISO_GPIO_Port GPIOB
#define SPI2_MOSI_Pin GPIO_PIN_15
#define SPI2_MOSI_GPIO_Port GPIOB
#define POWER3_Pin GPIO_PIN_8
#define POWER3_GPIO_Port GPIOA
#define POWER4_Pin GPIO_PIN_9
#define POWER4_GPIO_Port GPIOA
#define UP_CS_Pin GPIO_PIN_10
#define UP_CS_GPIO_Port GPIOA
#define LED4_Pin GPIO_PIN_11
#define LED4_GPIO_Port GPIOA
#define RX2_Pin GPIO_PIN_12
#define RX2_GPIO_Port GPIOA
#define LED2_Pin GPIO_PIN_15
#define LED2_GPIO_Port GPIOA
#define LED1_Pin GPIO_PIN_4
#define LED1_GPIO_Port GPIOB
#define RX1_Pin GPIO_PIN_6
#define RX1_GPIO_Port GPIOB
#define LED3_Pin GPIO_PIN_7
#define LED3_GPIO_Port GPIOB
#define POWER2_Pin GPIO_PIN_8
#define POWER2_GPIO_Port GPIOB
#define POWER1_Pin GPIO_PIN_9
#define POWER1_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
