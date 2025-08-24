#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include "GPIO.h"
#include "Utils.h"
#include "syscfg.h"
#include "NVIC.h"
#include "timer.h"

#ifdef __cplusplus
extern "C" {
#endif

int _write(int file, char *ptr, int len) {
  int DataIdx;
  for (DataIdx = 0; DataIdx < len; DataIdx++) { // 96 84 80 50 48 32 24 25 16
    ITM_SendChar(*ptr++);
  }
  return len;
}

#ifdef __cplusplus
}
#endif

static volatile int pinPB7Counter = 0;
static volatile bool pinIncEvent = false;
//defining register pointers
static GPIOGeneralRegister* GPIO_C15;
static GPIOGeneralRegister* GPIO_B7;
static GPIOGeneralRegister* GPIO_B12;
static SYSCFG_TypeDef* syscfg;
static EXTI_TypeDef* exti;

static TIM_TypeDef_timers* timer3;

void count_pin_high()
{
	pinPB7Counter++;
	pinIncEvent = true;
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

void TIM3InterruptLEDB12Toggle()
{
	// enable GPIOB
	GPIO_B12 = (GPIOGeneralRegister*) GPIOB;
	RCC->AHB1ENR |= RCC_AHB1LPENR_GPIOBLPEN;
	// set up GPIO D12.
	GPIO_B12->MODER   |= BIT((24)); // output mode
  /* Configure PDx pins speed to 50 MHz */  
  GPIO_B12->OSPEEDR |= BIT((25)); // 10 high speed
  /* Configure PDx pins Output type to push-pull */  
  GPIO_B12->OTYPER  |= BIT((25)); // pull down 10
	
	//Enable TIM3 then create timer3 struct.
	TIM3EnableAPB1ENR();
	timer3  = (TIM_TypeDef_timers*) TIM3_BASE;
	timer3->PSC = 1999; // prescaler to cpu clock = CPU F/(1+ PSC)  e.g. 16MHZ / 2000 = 8000.
	timer3->ARR = 8000; // triggers event every 8000 ticks => if PSC == 8000 /8000 = every 1seconds.
	timer3->CR1 &= ~((unsigned int)BIT(1)); // enable the update event if set to 0.
	
	// Enable trigger interrupt when event occurs
	// trigger TIM3 interrupt. Enable it on NVIC.
	timer3->DIER |= BIT(0);
	NVIC_EnableIRQ(TIM3_IRQn);
	
	// Enable counter upcounter or down counter
	// 0 down 1 up counter. 0 ==> ARR value.
	// nothing to do leave to 0
	
	// CEN counter enable
	timer3->CR1 |= BIT(0);
	
	// Enable the one pulse mode test later
	//timer3->CR1 |= BIT(3);
	
	// finally use TIM3_IRQHandler() to toggle GPIOD12
	
}

void TIM3_IRQHandler()
{
	static volatile uint8_t t3_cnt = 0;
	
	if(t3_cnt % 2)
	{
		GPIO_B12->BSRR |= (GPIO_BSRR_BR12); // reset
	}
	else
	{
		GPIO_B12->BSRR |= (GPIO_BSRR_BS12); //set
	}
	t3_cnt++;
	
	//clear UIF update interrupt flag in TIMx_SR register
	// Check if the Update Interrupt Flag (UIF) is set
  if (TIM3->SR & (1U << 0)) {
    // Clear the Update Interrupt Flag by writing zero to its bit
    TIM3->SR &= ~(1U << 0);
    // Handle the update interrupt event
  }
}

int main()
{
	//SystemClock_Config();
	printf("clock speed %d, STM32!\n", SystemCoreClock);

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
	//NVIC_EnableIRQ(EXTI9_5_IRQn);

	// custom example
	_NVIC_enable_(EXTI9_5_IRQn);
	
	
  //FTSR
	
	//printf("hello");
	gpio_voltage_measurment_b7();
	gpio_voltage_measurment_C15();
	//volatile bool PB7Status = false;
	//volatile bool PC15Status = false;

	gpioC13_output_on_off();
	
	TIM3InterruptLEDB12Toggle();
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
		
		SysTick_Delay_ms(100);
		if(pinIncEvent)
		{
			printf("pin high count INC event = %d", pinPB7Counter);
			SysTick_Delay_ms(500);
			pinIncEvent = false;
		}
		
		
	}
}