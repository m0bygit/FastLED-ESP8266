#ifndef __INC_LED_SYSDEFS_XTENSA_ESP8266_H
#define __INC_LED_SYSDEFS_XTENSA_ESP8266_H

#include <stdint.h>
#include "spi_register.h"

#define FASTLED_XTENSA

#ifndef INTERRUPT_THRESHOLD
#define INTERRUPT_THRESHOLD 1
#endif

typedef volatile uint32_t RoReg;
typedef volatile uint32_t RwReg;

// Default to allowing interrupts
#ifndef FASTLED_ALLOW_INTERRUPTS
#define FASTLED_ALLOW_INTERRUPTS 1
#endif

#define FASTLED_USE_PROGMEM 0
#define USE_GET_MILLISECOND_TIMER

#if FASTLED_ALLOW_INTERRUPTS == 1
#define FASTLED_ACCURATE_CLOCK
#endif

#define FASTLED_NO_PINMAP

// SPI defines

//Define SPI hardware modules
#define XTENSA_SPI 0
#define XTENSA_HSPI 1

#define XTENSA_SPI_TYPE XTENSA_HSPI

#define SPI_CLK_USE_DIV 0
#define SPI_CLK_80MHZ_NODIV 1
#define XTENSA_SPI_CLK_DIV SPI_CLK_USE_DIV


//	SPI_BYTE_ORDER_HIGH_TO_LOW (1) 
//	Data is read in starting with Bit31 and down to Bit0
//
//	SPI_BYTE_ORDER_LOW_TO_HIGH (0)
//	Data is read in starting with the lowest BYTE, from 
//	MSB to LSB, followed by the second lowest BYTE, from
//	MSB to LSB, followed by the second highest BYTE, from
//	MSB to LSB, followed by the highest BYTE, from MSB to LSB
//	0xABCDEFGH would be read as 0xGHEFCDAB
#define SPI_BYTE_ORDER_HIGH_TO_LOW 1
#define SPI_BYTE_ORDER_LOW_TO_HIGH 0

#define SPI_BYTE_ORDER SPI_BYTE_ORDER_HIGH_TO_LOW

#ifndef CPU_CLK_FREQ //Should already be defined in eagle_soc.h
#define CPU_CLK_FREQ 80*1000000
#endif

//Define some default SPI clock settings
#define SPI_CLK_PREDIV 8
#define SPI_CLK_CNTDIV 2
#define SPI_CLK_FREQ CPU_CLK_FREQ/(SPI_CLK_PREDIV*SPI_CLK_CNTDIV) // 80 / 8*2 = 5 MHz



// reusing/abusing cli/sei defs for due
//#define cli()  __disable_irq(); __disable_fault_irq();
//#define sei() __enable_irq(); __enable_fault_irq();


#endif
