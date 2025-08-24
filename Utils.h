#pragma once
#include "stm32f411xe.h"
#include "core_cm4.h"
typedef uint32_t reg32v_t;
#define REG32_GET(x) (*((reg32v_t*) (x))) 
#define REG32_INT(x)  (x)

	
#define SET_BIT_REG32(adr, position) \
((adr) |= (reg32v_t)(1U << (position)))

#define SET_BITS_HEX(adr, start, hex) \
((adr) |= (reg32v_t)(hex << (start)))

#define RESET_BIT_REG32(adr, position) \
((adr) &= (reg32v_t)(~(1U << (position))))


#define RESET_BITS_00(adr, start_pos) \
    ((adr) &= ~((1U << (start_pos)) | (1U << (start_pos+1))))

#define SET_BITS_01(adr, start_pos) \
	SET_BIT(adr, start_pos); \
	RESET_BIT_REG32(adr, start_pos+1);

#define SET_BITS_10(adr, start_pos) \
	RESET_BIT_REG32(adr, start_pos); \
	SET_BIT(adr, start_pos+1);
	
#define SET_BITS_11(adr, start_pos) \
	((adr) |= (reg32v_t)((1U << (start_pos)) | (1U << (start_pos+1))))

	\
#include "stm32f4xx.h"

void SysTick_Delay_ms(uint32_t ms) {
	// Assuming HCLK is 16MHz for example. Adjust according to your clock configuration.
	// SysTick->LOAD = (SystemCoreClock / 1000) * ms - 1; // For 1ms tick
	SysTick->LOAD = (SystemCoreClock /1000 - 1) * ms;
	SysTick->VAL = 0; // Clear current value
	SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_ENABLE_Msk; // Use HCLK, enable SysTick

	while (!(SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk)); // Wait for COUNTFLAG
	SysTick->CTRL = 0; // Disable SysTick after delay
}


uint32_t updateAndGetClock()
{
	SystemCoreClockUpdate();
	// Now, SystemCoreClock holds the current system clock frequency
	return SystemCoreClock;
}


void SystemClock_Config(void) {
    // Enable HSE (if using external crystal) 25MHZ on F411
    RCC->CR |= RCC_CR_HSEON;
    while (!(RCC->CR & RCC_CR_HSERDY));

	RCC->APB1ENR |= RCC_APB1ENR_PWREN;
	PWR ->CR |= PWR_CR_VOS;

    // Configure Flash Latency (example for high speed)
	FLASH -> ACR |= FLASH_ACR_ICEN | FLASH_ACR_PRFTEN | FLASH_ACR_DCEN  |FLASH_ACR_LATENCY_3WS;
    // Configure PLL (example values, replace with your calculations)
	//Put PLL_M = 25, PLL_P = 0, PLL_N = 192, PLL_M = 25 and PLL_Q = 4 (Main PLL prescaler)
	//Select PLL source as HSE oscillator by setting the RCC_PLLCFGR_PLLSRC bit.

	//formula https://kunalsalvi63.medium.com/bare-metal-rcc-setup-for-stm32f411cex-23d2ef2d706d
	// Input HSE freq => 25MHZ for F411 / PLL_M x PLL_N / PLL_P == 
	// 25/25 *192 / 2 == 96 MHZ clock
	// Then ensure to set AHB, APB1 and APB2. APB1 MAx 50 MHZ ! , APB2 MAX =96MHZ
	
	//Put PLL_M = 25, PLL_P = 0 (to get / 2), PLL_N = 192, PLL_M = 25 and PLL_Q = 4. 
    RCC->PLLCFGR = (25U << RCC_PLLCFGR_PLLM_Pos) | // PLLM = 25 MHZ for HSE oscillo
                   (192U << RCC_PLLCFGR_PLLN_Pos) | // PLLN = 192 MHZ
                   (0U << RCC_PLLCFGR_PLLP_Pos) | // PLLP = 2 (0b00)
                   (4U << RCC_PLLCFGR_PLLQ_Pos) | // PLLQ = 4 so 192/4 e.g.
                   RCC_PLLCFGR_PLLSRC_HSE; // PLL Source is HSE

    // Enable PLL
    RCC->CR |= RCC_CR_PLLON;
    while (!(RCC->CR & RCC_CR_PLLRDY));

    // Configure AHB, APB1, APB2 prescalers
    RCC->CFGR |= RCC_CFGR_HPRE_DIV1 | // AHB prescaler = 1 => ,max AHB == 96
                 RCC_CFGR_PPRE1_DIV2 | // APB1 prescaler = 2 max APB1 50MHZ
                 RCC_CFGR_PPRE2_DIV1; // APB2 prescaler = 1 max APB2 96MHZ
	// 25mhz fixed PLL max 100 AHB96 APB1 50 APB2 96/
	// HSE >>> PLL >>> SYSCLCK >>> AHB/APB1/APB2
    // Select PLL as system clock source
    RCC->CFGR |= RCC_CFGR_SW_PLL;
    while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL);

    // Update SystemCoreClock variable (part of CMSIS)
    SystemCoreClockUpdate();
}