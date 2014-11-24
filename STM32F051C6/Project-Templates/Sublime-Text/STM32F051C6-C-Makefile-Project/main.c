/*  
    
    STM32F051C6 Default C Template used with
    with makefile and with Sublime GDB
    debugging included

    Author: Sean Wood
    Lasted Updated: 23/11/2014

*/


#include "woodboxstm32f051C6.h"

// Declarations


// Variables


int main(void) {

  // Code!

  while(1) {  // Infinite loop
    }
  return 0;
}

void TIM6_DAC_IRQHandler() { // Interrupt Service Routine for TIM6
}

void EXTI0_1_IRQHandler() { // Interrupt Service Routine for EXTI lines 0 and 1 (PB 0 and 1)
}

void EXTI2_3_IRQHandler() { // Interrupt Service Routine for EXTI lines 2 and 3 (PB 2 and 3)
}

void HardFault_Handler() { // Hardfault Handler Routine
  GPIOB -> ODR = 0xA5;
}