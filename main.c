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
	SysTick_Delay_ms(200);
	
	gPIOC13->BSRR |= (BIT(13));
	SysTick_Delay_ms(500);

}

void gpio_voltage_measurment_b7()
{
	GPIOGeneralRegister* GPIO_B7;
	SET_BIT_REG32(REG32_GET(RCC_AHB1ENR), RCC_AHB1_ENR_CPIO_B_RANGE);

	GPIO_B7 = (GPIOGeneralRegister*) GPIO_B_BASE;
	GPIO_B7->MODER = GPIOC_MODER_RESET_VALUE;	
	GPIO_B7->MODER &= ~(uint32_t) (BIT(14) | BIT(15)); //INPUT MODE 00

	GPIO_B7->OTYPER &= ~(uint32_t) BIT(7); // PUSH+PULL out 1

	GPIO_B7->OSPEEDR = GPIOB_OSPEEDR_RESET_VALUE;
	GPIO_B7->OSPEEDR |= (BIT(14));  // Medium speed 01.
	
	GPIO_B7->PUPDR = GPIOB_PUPDR_RESET_VALUE;
	GPIO_B7->PUPDR |= BIT(15); // Pull up 01 / pull down 10.

	//detect use IDR if is set. will be an int variable value, then converted to boolean.
	//volatile bool pinPB7Status;
	volatile uint16_t val;
	gpioC13_output_on_off();
	val = ((GPIO_B7->IDR) & BIT(7));
	
	if(val > 0)
	{
		gpioC13_output_on_off();
	}
}

void gpio_voltage_measurment_C15()
{
	GPIOGeneralRegister* GPIO_C15;
	SET_BIT_REG32(REG32_GET(RCC_AHB1ENR), RCC_AHB1_ENR_CPIO_C_RANGE);

	GPIO_C15 = (GPIOGeneralRegister*) GPIO_C_BASE;
	// GPIO_C15->MODER = 0x00000000;
	GPIO_C15->MODER &= ~(uint32_t) (BIT(30) | BIT(31)); //INPUT MODE

	// GPIO_C15->MODER = 0x00000000;
	GPIO_C15->OTYPER &= ~(uint32_t) BIT(15); // PUSH+PULL out

	//GPIO_C15->OSPEEDR |= (BIT(bit2));  // Medium speed 01.
	// GPIO_C15->OSPEEDR = 0x00000000;
	GPIO_C15->OSPEEDR |= (uint32_t) (BIT(30)); // 15 0
	//GPIO_C15->OSPEEDR &= ~(uint32_t) (BIT(31)); // 15 0

	// GPIO_C15->PUPDR = 0x00000000;
	GPIO_C15->PUPDR |= BIT(31); // Pull up 01/ pull down 10
	//GPIO_C15->PUPDR &= ~(uint32_t)  BIT(14);

	//detect use IDR if is set. will be an int variable value, then converted to boolean.
	//gpioC13_output_on_off();
	uint16_t val =((GPIO_C15->IDR) & BIT(15));

	if(val > 0)
	{
		gpioC13_output_on_off();
	}
}


int main()
{
	gpio_voltage_measurment_C15();
	
	while(1)
	{
	}
}