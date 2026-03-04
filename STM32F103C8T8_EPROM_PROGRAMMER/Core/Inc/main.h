/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
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
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <inttypes.h>
#include <stdbool.h>
#include "string.h"
#include "stdio.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/

/* USER CODE BEGIN Private defines */
#define SIZE_OF_BUFFER 2048//64

#define LD1_Pin GPIO_PIN_13
#define LD1_GPIO_Port GPIOC

//shift registers (serial input - parallel outputs) for ADDRESS bus WRITING
#define HC595_DS_ADDR_Pin GPIO_PIN_7//SHIFT_DATA
#define HC595_DS_ADDR_Port GPIOA
#define HC595_STCP_ADDR_Pin GPIO_PIN_6//SHIFT_LATCH : latch clock - send bits to Q0 - Q7
#define HC595_STCP_ADDR_Port GPIOA
#define HC595_SHCP_ADDR_Pin GPIO_PIN_5//SHIFT_CLOCK : shift register clock - every rising edge shift of 1 bit
#define HC595_SHCP_ADDR_Port GPIOA

//shift registers (serial input - parallel outputs) for DATA bus WRITING
#define HC595_DS_DATA_Pin GPIO_PIN_12//SHIFT_DATA
#define HC595_DS_DATA_Port GPIOB
#define HC595_STCP_DATA_Pin GPIO_PIN_13//SHIFT_LATCH
#define HC595_STCP_DATA_Port GPIOB
#define HC595_SHCP_DATA_Pin GPIO_PIN_14//SHIFT_CLOCK
#define HC595_SHCP_DATA_Port GPIOB

//shift registers (parallel input - serial outputs) for DATA bus READING
#define HC165__PL_DATA_Pin GPIO_PIN_9//NOT PL
#define HC165__PL_DATA_GPIO_Port GPIOA
#define HC165_DS_DATA_Pin GPIO_PIN_8//pin Q7
#define HC165_DS_DATA_GPIO_Port GPIOA
#define HC165_CLK_DATA_Pin GPIO_PIN_10
#define HC165_CLK_DATA_GPIO_Port GPIOA

#define MAX734__SHDN_Pin GPIO_PIN_10//NOT shutdown - 12v generation from 5v input source
#define MAX734__SHDN_GPIO_Port GPIOB
#define CMD_A9_12V_Pin GPIO_PIN_3//pin number 9 of address bus to 12v to read the memory identification bytes
#define CMD_A9_12V_GPIO_Port GPIOA
//NOT OE output enable active low from UV-EPROM INTEL 27512
#define CMD0__OE_Pin GPIO_PIN_1//EPROM__OE Low if CMD__OE High
#define CMD0__OE_GPIO_Port GPIOA
#define CMD1__OE_Pin GPIO_PIN_0//if CMD1__OE High & CMD0__OE Low EPROM__OE to 12v else if CMD1__OE Low & CMD0__OE Low EPROM__OE to 5v
#define CMD1__OE_GPIO_Port GPIOA

#define EPROM__CE_Pin GPIO_PIN_15//NOT chip select if high (active low)
#define EPROM__CE_GPIO_Port GPIOC
#define CMD_6V_VDD_EPROM_Pin GPIO_PIN_14//5v for reading if high either 6v to write if low
#define CMD_6V_VDD_EPROM_GPIO_Port GPIOC

#define MEMORY_SIZE_kb 512//kbits (512kb bits = 64kB bytes)
#define NUMBER_BYTES_PER_ROW 16//from 0x00 to 0x0F
#define MEMORY_ROW MEMORY_SIZE_kb/8*1024/NUMBER_BYTES_PER_LINE //should equal 4096 row (from 0x0000 to 0xFFF0)

typedef enum{Idle = 0, Event = 1}Flag;
extern Flag flag_rx_usb;
extern TIM_HandleTypeDef htim3;

void enter_and_get_out_sleep_mode(void);
void low_pulse_CE_1us(void);
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
