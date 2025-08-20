#pragma once
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
	SysTick->LOAD = (16000 - 1) * ms; // For a 1ms delay with 16MHz HCLK
	SysTick->VAL = 0; // Clear current value
	SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_ENABLE_Msk; // Use HCLK, enable SysTick

	while (!(SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk)); // Wait for COUNTFLAG
	SysTick->CTRL = 0; // Disable SysTick after delay
}
