#pragma once
#include "Utils.h"
#include <stdint.h>

// 1. find memory space for PC13 in memory map. addr: for PC = 0x4002 0800 - 0x4002 0BFF GPIOC
// 2. get the address offset from manual e.g. MODER
/*
8.4.1 GPIO port mode register (GPIOx_MODER) (x = A..E and H)
Address offset: 0x00
Reset values:
� 0xA800 0000 for port A
� 0x0000 0280 for port B
� 0x0000 0000 for other ports

1. take memory map address : PC : 0x4002 0800 - 0x4002 0BFF
2. apply offset: 


32-bit configuration registers (GPIOx_MODER,
GPIOx_OTYPER, GPIOx_OSPEEDR and GPIOx_PUPDR), two 32-bit data registers
(GPIOx_IDR and GPIOx_ODR), a 32-bit set/reset register (GPIOx_BSRR), a 32-bit locking
register (GPIOx_LCKR) and two 32-bit alternate function selection register (GPIOx_AFRH
and GPIOx_AFRL).

// Registers used in order: Base address in mem space for C GPIOs: 0x40020800
//GPIOx_MODER GPIO MODER off 0x00
//GPIOx_OTYPER GPIO TYPER off 0x04
//GPIOx_OSPEEDR GPIO SPEED off 0x08
//GPIOx_PUPDR GPIO PULL UP PULL DOWN off 0x0C
//GPIOx_IDR GPIO input data off 0x10
//GPIOx_ODR GPIO output data off 0x014 ==> force change value of the output
//GPIOx_BSRR GPIO Bit set/reset off 0x18 ==> set value to high or low.
//GPIOx_LCKR GPIO configuration lock off 0x1C 
//GPIOx_AFRH GPIO Alternate function low register off 0x20
//GPIOx_AFRL GPIO Alternate function high register off 0x24

// 3 Categories + bit set: Status (READ) + Configuration(W) + Alternate function(W)
// Configuration: MODER, TYPER, SPEEDR, PUDR, BSRR BIT SET/RESET.
// Status: IDR : Input Data Register + ODR Output Data Register.
// Bit SET/RESET: SEt value to HIGH or LOW

// ===> Refer to the GPIO REgister map for all of this ! PAGE 
// configure PC13 ==> GPIO C13

*/
// see 6.3.22 RCC register map => offset GPIOxEN
#define RCC_AHB1_ENR_CPIO_C_RANGE 2
#define RCC_AHB1_ENR_CPIO_B_RANGE 1


#define RCC_AHB1 (0x40023800)
#define RCC_AHB1ENR (RCC_AHB1 + 0x30) // Enable the BUS 1st
// look for GPIOCEN in docs. ==> Bit 2 GPIOCEN: IO port C clock enable


//see memory map table
#define GPIO_C_BASE  (0x40020800)
#define GPIOC_MODER_RESET_VALUE 0x00000000

#define GPIO_B_BASE  (0x40020400) // memory map
// section 8 GPIO page 158
#define GPIOB_MODER_RESET_VALUE   0x00000280 
#define GPIOB_OSPEEDR_RESET_VALUE 0x000000C0
#define GPIOB_PUPDR_RESET_VALUE   0x00000100



#define GPIO_OFF_MODER (GPIO_C_BASE + 0x00 )
#define GPIO_OFF_TYPER (GPIO_C_BASE + 0x04)
#define GPIO_OFF_SPEEDR (GPIO_C_BASE + 0x08)
#define GPIO_OFF_PUDR (GPIO_C_BASE + 0x0C)
#define GPIO_OFF_BSRR (GPIO_C_BASE + 0x18)
#define GPIO_PC13_BIT_SHIFT 26
#define BIT(x) (1U << x)

typedef struct
{
	uint32_t MODER; //0x0
	uint32_t OTYPER; //0x4
	uint32_t OSPEEDR; //0x8
	uint32_t PUPDR; //0xC
	uint32_t IDR; //0x10
	uint32_t ODR; //0x14
	uint32_t BSRR; //0x18
	uint32_t LCKR; //0x1C
	uint32_t AFRL; //0x20
	uint32_t AFRH; //0x24
} GPIOGeneralRegister;