#include <stdint.h>
#include <stdbool.h>
#include "GPIO.h"
#include "Utils.h"

void gpioC13_output_on_off()
{
	// mode to output set bit 26-27 to b01/0x1	
	// Enable C GPIO RANGE : 6.3.9 RCC AHB1 peripheral clock enable register ==> bit 2 for range C
	SET_BIT_REG32(REG32_GET(RCC_AHB1ENR), RCC_AHB1_ENR_CPIO_C_RANGE);
	
	GPIOGeneralRegister* gPIOC13;
	gPIOC13 = (GPIOGeneralRegister*) GPIO_C_BASE;
	
	//SET bit26-27: 01 => output GPIO
	
	gPIOC13->MODER |= (BIT(26));
	gPIOC13->OTYPER &= ~(uint32_t) (BIT(13));
	gPIOC13->OSPEEDR &= ~(uint32_t) ((BIT(26)) | (BIT(27)));
	
	gPIOC13->PUPDR |= (BIT(27));
	gPIOC13->PUPDR &= ~(uint32_t) (BIT(26));
	
	/*
	//SET_BIT_REG32(REG32_GET(GPIO_OFF_MODER), 26);
	SET_BITS_01(REG32_GET(GPIO_OFF_MODER), 26);
	
	//type push/pull bit 13 to b0
	RESET_BIT_REG32(REG32_GET(GPIO_OFF_TYPER), 13); // RESET bit13 to 0 ==> var |= ~(BIT(xx)
	
	// Speed : low speed 0b00
	// lookup I/O ports charactertic in the DATASHEET PDF
	RESET_BITS_00(REG32_GET(GPIO_OFF_SPEEDR), 26);
	
	//set to pull down state, bits26-27 to b10/0x2
	SET_BITS_10(REG32_GET(GPIO_OFF_PUDR), 26);
	*/
	
	// BSSR HIGH and LOW
	SET_BIT_REG32(REG32_GET(GPIO_OFF_BSRR), 13); // LOW
	SET_BIT_REG32(REG32_GET(GPIO_OFF_BSRR), 29); // HIGH
	
	SET_BIT_REG32(REG32_GET(GPIO_OFF_BSRR), 13); // LOW
	SET_BIT_REG32(REG32_GET(GPIO_OFF_BSRR), 29); // HIGH
	
	gPIOC13->BSRR |= (BIT(13));
	SysTick_Delay_ms(500);
	gPIOC13->BSRR |= (BIT(29));
	SysTick_Delay_ms(500);

	
	gPIOC13->BSRR |= (BIT(13));
	SysTick_Delay_ms(500);

	gPIOC13->BSRR |= (BIT(29));
	SysTick_Delay_ms(500);

}

void gpio_voltage_measurment()
{
	GPIOGeneralRegister* GPIO_B7;
	SET_BIT_REG32(REG32_GET(RCC_AHB1ENR), RCC_AHB1_ENR_CPIO_B_RANGE);

	GPIO_B7 = (GPIOGeneralRegister*) GPIO_C_BASE;
		
	GPIO_B7->MODER |= ~(uint32_t) (BIT(14) | BIT(15)); //INPUT MODE
	GPIO_B7->OTYPER &= ~(uint32_t) BIT(7); // PUSH+PULL out

	GPIO_B7->OSPEEDR |= (BIT(14));  // Medium speed 01.
	GPIO_B7->OSPEEDR &= ~(uint32_t) (BIT(15)); // 15 0
	
	GPIO_B7->PUPDR |= BIT(15); // Pull down 10
	GPIO_B7->PUPDR &= ~(uint32_t)  BIT(14);

	//detect use IDR if is set. will be an int variable value, then converted to boolean.
	volatile bool pinPB7Status = ((GPIO_B7->IDR) & BIT(7));
	if(!pinPB7Status)
	{
		gpioC13_output_on_off();
	}
}


int main()
{
	gpio_voltage_measurment();
	
	while(1)
	{
	}
}