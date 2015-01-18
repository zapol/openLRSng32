//#pragma once

// for roundf()
#define __USE_C99_MATH

#include <stdbool.h>
#include <stdlib.h>
#include <stdint.h>
#include <math.h>
#include <ctype.h>
#include <string.h>
#include <stdio.h>

#include "hw_defines.h"
#include "stm32f10x_conf.h"
#include "core_cm3.h"
#include "printf.h"

#ifndef M_PI
#define M_PI       3.14159265358979323846f
#endif /* M_PI */

#define RADX10 (M_PI / 1800.0f)                  // 0.001745329252f

#define min(a, b) ((a) < (b) ? (a) : (b))
#define max(a, b) ((a) > (b) ? (a) : (b))
#define abs(x) ((x) > 0 ? (x) : -(x))
#define constrain(amt, low, high) ((amt) < (low) ? (low) : ((amt) > (high) ? (high) : (amt)))

// Chip Unique ID on F103
#define U_ID_0 (*(uint32_t*)0x1FFFF7E8)
#define U_ID_1 (*(uint32_t*)0x1FFFF7EC)
#define U_ID_2 (*(uint32_t*)0x1FFFF7F0)

#define digitalHi(p, i)     { p->BSRR = i; }
#define digitalLo(p, i)     { p->BRR = i; }
#define digitalToggle(p, i) { p->ODR ^= i; }

#define LEDR_GPIO GPIOB
#define LEDR_PIN  GPIO_Pin_2
#define LEDR_ON()   digitalHi(LEDR_GPIO,LEDR_PIN)
#define LEDR_OFF()  digitalLo(LEDR_GPIO,LEDR_PIN)
#define LEDG_GPIO GPIOB
#define LEDG_PIN  GPIO_Pin_8
#define LEDG_ON()   digitalHi(LEDG_GPIO,LEDG_PIN)
#define LEDG_OFF()  digitalLo(LEDG_GPIO,LEDG_PIN)
// #define LEDB_GPIO GPIOB
// #define LEDB_PIN  GPIO_Pin_13
// #define LEDB_ON()   digitalHi(LEDB_GPIO,LEDB_PIN)
// #define LEDB_OFF()  digitalLo(LEDB_GPIO,LEDB_PIN)

#ifndef FLASH_PAGE_COUNT
#define FLASH_PAGE_COUNT 128
#endif

#define FLASH_PAGE_SIZE  ((uint16_t)0x400)
#define FLASH_WRITE_ADDR (0x08000000 + (uint32_t)FLASH_PAGE_SIZE * (FLASH_PAGE_COUNT - 1)) // use the last page
#define FLASH_FSWRITE_ADDR (0x08000000 + (uint32_t)FLASH_PAGE_SIZE * (FLASH_PAGE_COUNT - 2))
#define FLASH_RXWRITE_ADDR (0x08000000 + (uint32_t)FLASH_PAGE_SIZE * (FLASH_PAGE_COUNT - 3))

#include "drv_system.h"         // timers, delays, etc
#include "drv_adc.h"
#include "drv_uart.h"
#include "drv_pwm.h"
#include "drv_spi.h"
#include "drv_i2c.h"
#include "drv_rfm.h"
#include "drv_gpio.h"
#include "drv_mpu6050.h"
#include "binding.h"
#include "mixer.h"
#include "autopilot.h"
#include "version.h"
