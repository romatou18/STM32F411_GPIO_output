
/*
#include <stdio.h>
    #include <rt_misc.h>
    #include <stdint.h>
    #include "stm32f4xx.h" // Include your specific STM32 header
		
		#ifdef __GNUC__
    #define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
    #else
    #define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
    #endif

    PUTCHAR_PROTOTYPE
    {
        HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, HAL_MAX_DELAY); // Replace huartX with your UART handle
        return ch;
    }
		
		 extern int _write(int file, char *ptr, int len) {
        for (int i = 0; i < len; i++) {
            ITM_SendChar(*ptr++);
        }
        return len;
    }
		
		  extern uint32_t ITM_SendChar(uint32_t ch); // Declare this function
		//	__STATIC_INLINE uint32_t ITM_SendChar (uint32_t ch)

		int fputc(int ch, FILE *f) {
        ITM_SendChar(ch);
        return(ch);
    }
  //  #pragma import(__use_no_semihosting_swi)

   // struct __FILE {
    //    int handle;
    //};

    FILE __stdout;
    FILE __stdin;

    volatile int32_t ITM_RxBuffer;

    int fputc(int ch, FILE *f) {
        ITM_SendChar(ch);
        return(ch);
    }

    void _sys_exit(int return_code) {
        return ;
    }
		
		*/