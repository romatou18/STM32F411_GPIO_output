#include <stdint.h>
#include "GPIO.h"
#include "Utils.h"

int main()
{
	
	// mode to output set bit 26-27 to b01/0x1	
	// Enable C GPIO RANGE : 6.3.9 RCC AHB1 peripheral clock enable register ==> bit 2 for range C
	SET_BIT(REG32_GET(RCC_AHB1ENR), RCC_AHB1_ENR_CPIO_C_RANGE);
	
	GPIOGeneralRegister* gPIOC13;
	gPIOC13 = (GPIOGeneralRegister*) GPIO_C_BASE;
	
	//SET bit26-27: 01 => output GPIO
	
	gPIOC13->MODER |= (1U << 26);
	gPIOC13->OTYPER &= ~(uint32_t) (1U << 13);
	gPIOC13->OSPEEDR &= ~(uint32_t) ((1U << 26) | (1U << 27));
	
	gPIOC13->PUPDR |= (1U << 27);
	gPIOC13->PUPDR &= ~(uint32_t) (1U << 26);
	
	/*
	//SET_BIT(REG32_GET(GPIO_OFF_MODER), 26);
	SET_BITS_01(REG32_GET(GPIO_OFF_MODER), 26);
	
	//type push/pull bit 13 to b0
	RESET_BIT_REG32(REG32_GET(GPIO_OFF_TYPER), 13); // RESET bit13 to 0 ==> var |= ~(1U << xx)
	
	// Speed : low speed 0b00
	// lookup I/O ports charactertic in the DATASHEET PDF
	RESET_BITS_00(REG32_GET(GPIO_OFF_SPEEDR), 26);
	
	//set to pull down state, bits26-27 to b10/0x2
	SET_BITS_10(REG32_GET(GPIO_OFF_PUDR), 26);
	*/
	
	// BSSR HIGH and LOW
	SET_BIT(REG32_GET(GPIO_OFF_BSRR), 13); // LOW
	SET_BIT(REG32_GET(GPIO_OFF_BSRR), 29); // HIGH
	
	SET_BIT(REG32_GET(GPIO_OFF_BSRR), 13); // LOW
	SET_BIT(REG32_GET(GPIO_OFF_BSRR), 29); // HIGH
	
	gPIOC13->BSRR |= (1U << 13);
	gPIOC13->BSRR |= (1U << 29);
	
	gPIOC13->BSRR |= (1U << 13);
	gPIOC13->BSRR |= (1U << 29);
	

	
	while(1)
	{
	}
}