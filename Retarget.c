#include <stdint.h>
#include "stm32f4xx.h"

#include <stdio.h>
#include "core_cm4.h" // or core_cm3.h depending on your core

int fputc(int ch, FILE *f)
{
  ITM_SendChar(ch);
  return(ch);
}
