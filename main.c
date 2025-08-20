#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include "GPIO.h"
#include "Utils.h"
#include "syscfg.h"

static volatile int pinPB7Counter = 0;

//defining register pointers
static GPIOGeneralRegister* GPIO_C15;
static GPIOGeneralRegister* GPIO_B7;
static SYSCFG_TypeDef* syscfg;
static EXTI_TypeDef* exti;

void count_pin_high()
{
	pinPB7Counter++;
}

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
	
}

void gpio_voltage_measurment_C15()
{
	//SET_BIT_REG32(REG32_GET(RCC_AHB1ENR), RCC_AHB1_ENR_CPIO_C_RANGE);
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
	GPIO_C15->PUPDR |= BIT(31); // Pull up 01/ pull down 10 OK
	//GPIO_C15->PUPDR &= ~(uint32_t)  BIT(30);
	//GPIO_C15->PUPDR &= ~(uint32_t)  BIT(31);

	//detect use IDR if is set. will be an int variable value, then converted to boolean.
	//gpioC13_output_on_off();
	
}



void EXTI9_5_IRQHandler()
{
	count_pin_high();
	SET_BIT_REG32(REG32_GET(EXTI_ADDRESS + 0x14), 7); // Clear interrupt.
}

int main()
{
	if(RCC_AHB1ENR_GPIOCEN_Pos != RCC_AHB1_ENR_CPIO_C_RANGE)
	{
		return 1;
	}
	
	if(GPIO_B_BASE != GPIOB_BASE)
	{
		return(4);
	}
	

	// ENABLE clocks in RCC Register. To enable BUSES that depend on it
	// enable bus for syscfg 6.3.12 RCC APB2 peripheral clock enable register
	SET_BIT_REG32(REG32_GET(RCC_APB2ENR), 14); // => syscfg is enabled on it.

	//SETTING UP SYSCFG
	syscfg = (SYSCFG_TypeDef*)(SYSCFG_ADR);
	syscfg->EXTICR[1] |= BIT(12); // configure external Interrupts config register on PBx pins  // EXT7 bits 12-15
	
	// SETTING UP EXTI INTERRUPT for PB7
	// Configure external pin interrupt, using EXT7 for PB7, IRQ handler EXTI9_5
	exti = (EXTI_TypeDef*)(EXTI_ADDRESS);
	exti->IMR  |= BIT(7); // Pin7 is not Masked. 
	exti->RTSR |= BIT(7); // rising trigger selection events get watched
	
	
	//Enabling Interrupts NVIC ! 
	NVIC_EnableIRQ(EXTI9_5_IRQn);
  //FTSR
	
	//printf("hello");
	gpio_voltage_measurment_b7();
	gpio_voltage_measurment_C15();
	//volatile bool PB7Status = false;
	//volatile bool PC15Status = false;

	gpioC13_output_on_off();
	while(1)
	{
		/*
		PB7Status = ((GPIO_B7->IDR) & BIT(7));
		PC15Status = ((GPIO_C15->IDR) & BIT(15));
		if(PC15Status )
		{
			count_pin_high();
		}
		
		if(PB7Status )
		{
			count_pin_high();
		}
		*/
		
		//SysTick_Delay_ms(100);
	}
}