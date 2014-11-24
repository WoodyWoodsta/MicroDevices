/*
 * woodboxstm32f051C6.c
 *
 *  Created on: Oct 3, 2014
 *      Author: Sean Wood - WDXSEA003
 *      		University of Cape Town
 *
 *  This is the source file.
 *
 *  This WoodBox is a collection of commonly used and generally
 *  pesky things that one wants to do with the STM32F051C6, but
 *  don't want to have to place in one's source! Yay!
 *
 *  [REV 1 - 23/11/2014]
 *    - Implented a better way to grab the status of a pushbutton ( getPB() )
 *            
 */

#include "woodboxstm32f051C6.h"

void programError() {
  GPIOB -> ODR = 0xE7;

  while(1);
}

//=====================
//====== TIMING ======= (Only for 8 MHz CPU Speed)
//=====================

void delayms(uint32_t length) {
  uint32_t i;
  length = length * 615; // Scaling to get 1 ms input parameter (length)
  for (i = 0; i <= length;) { // Do nothing really
	i++;
  }
}

void delaypointms(uint32_t length) {
  uint32_t i;
  length = length * 62; // Scaling to get 0.1 ms input parameter (length)
  for (i = 0; i <= length;) { // Do nothing really
    i++;
  }
}

//=====================
//==== PERIPHERALS ====
//=====================

//==== LEDS ====

void initLEDs(void) { //== Initialise LEDs
  RCC -> AHBENR |= RCC_AHBENR_GPIOBEN;  // Enable clock for GPIOB
  GPIOB -> MODER |= GPIO_MODER_MODER0_0;  // Set OUTPUT mode for pins 0-7 (LEDS)
  GPIOB -> MODER |= GPIO_MODER_MODER1_0;
  GPIOB -> MODER |= GPIO_MODER_MODER2_0;
  GPIOB -> MODER |= GPIO_MODER_MODER3_0;
  GPIOB -> MODER |= GPIO_MODER_MODER4_0;
  GPIOB -> MODER |= GPIO_MODER_MODER5_0;
  GPIOB -> MODER |= GPIO_MODER_MODER6_0;
  GPIOB -> MODER |= GPIO_MODER_MODER7_0;

}

void incrementLEDs(int8_t amount) { // Increment the LEDs by the specified value
  int8_t present = GPIOB -> ODR;
  GPIOB -> ODR = amount + present;
  // eeprom_write_to_address(0x0, amount + present); // If we want to "remember" the value after powerdown
}

//==== PUSH BUTTONS ====

void initPB(void) { //== Initialise pusbuttons
  RCC -> AHBENR |= RCC_AHBENR_GPIOAEN;  // Enable clock for GPIOA
  GPIOA -> PUPDR |= GPIO_PUPDR_PUPDR0_0; // Set PULLUP for pins 0-3
  GPIOA -> PUPDR |= GPIO_PUPDR_PUPDR1_0;
  GPIOA -> PUPDR |= GPIO_PUPDR_PUPDR2_0;
  GPIOA -> PUPDR |= GPIO_PUPDR_PUPDR3_0;
}

// int8_t getPB(int8_t button) {  // Check for state of a specific button
//   int8_t result = 0;
//   if (button == 0) {
//     result = ~(GPIOA -> IDR) & 0x1;  // SW0
//   }
//   else if (button == 1) {
//     result = ~(GPIOA -> IDR) & 0x2;  // SW1
//   }
//   else if (button == 2) {
//     result = ~(GPIOA -> IDR) & 0x4;  // SW2
//   }
//   else if (button == 3) {
//     result = ~(GPIOA -> IDR) & 0x8;  // SW3
//   }

//   return result;
// }

int8_t getPB(int8_t button) {  // Check for state of a specific button
  int8_t result = 0;
  result = (~(GPIOA -> IDR)) & (1 << button);

  return result;
}

void enableInterruptPB(int8_t button, int8_t mode) {
    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGCOMPEN; // Enable the clock for the system controller

  if (button == 0) {
    SYSCFG -> EXTICR[0] |= SYSCFG_EXTICR1_EXTI0_PA; // Set the EXTI line in the system controller
    EXTI -> IMR |= EXTI_IMR_MR0;  // Unmask the EXTI line
    if (mode == RISE) {
      EXTI -> RTSR |= EXTI_RTSR_TR0; // Set the RISE trigger
    } else if (mode == FALL) {
      EXTI -> FTSR |= EXTI_FTSR_TR0; // Set the FALL trigger
    } else if (mode == R_AND_F) {
      EXTI -> RTSR |= EXTI_RTSR_TR0; // Set the RISE trigger
      EXTI -> FTSR |= EXTI_FTSR_TR0; // Set the FALL trigger
    }

  } else if (button == 1) {
    SYSCFG -> EXTICR[0] |= SYSCFG_EXTICR1_EXTI1_PA; // Set the EXTI line in the system controller
    EXTI -> IMR |= EXTI_IMR_MR1;  // Unmask the EXTI line
    if (mode == RISE) {
      EXTI -> RTSR |= EXTI_RTSR_TR1; // Set the RISE trigger
    } else if (mode == FALL) {
      EXTI -> FTSR |= EXTI_FTSR_TR1; // Set the FALL trigger
    } else if (mode == R_AND_F) {
      EXTI -> RTSR |= EXTI_RTSR_TR1; // Set the RISE trigger
      EXTI -> FTSR |= EXTI_FTSR_TR1; // Set the FALL trigger
    }

  } else if (button == 2) {
    SYSCFG -> EXTICR[0] |= SYSCFG_EXTICR1_EXTI2_PA; // Set the EXTI line in the system controller
    EXTI -> IMR |= EXTI_IMR_MR2;  // Unmask the EXTI line
    if (mode == RISE) {
      EXTI -> RTSR |= EXTI_RTSR_TR2; // Set the RISE trigger
    } else if (mode == FALL) {
      EXTI -> FTSR |= EXTI_FTSR_TR2; // Set the FALL trigger
    } else if (mode == R_AND_F) {
      EXTI -> RTSR |= EXTI_RTSR_TR2; // Set the RISE trigger
      EXTI -> FTSR |= EXTI_FTSR_TR2; // Set the FALL trigger
    }

  } else if (button == 3) {
    SYSCFG -> EXTICR[0] |= SYSCFG_EXTICR1_EXTI3_PA; // Set the EXTI line in the system controller
    EXTI -> IMR |= EXTI_IMR_MR3;  // Unmask the EXTI line
    if (mode == RISE) {
      EXTI -> RTSR |= EXTI_RTSR_TR3; // Set the RISE trigger
    } else if (mode == FALL) {
      EXTI -> FTSR |= EXTI_FTSR_TR3; // Set the FALL trigger
    } else if (mode == R_AND_F) {
      EXTI -> RTSR |= EXTI_RTSR_TR3; // Set the RISE trigger
      EXTI -> FTSR |= EXTI_FTSR_TR3; // Set the FALL trigger
    }
  }

  if ((button == 0)||(button == 1)) {
    NVIC_EnableIRQ(EXTI0_1_IRQn);
  } else if ((button == 2)||(button == 3)) {
    NVIC_EnableIRQ(EXTI2_3_IRQn);
  }
}


//==== POTENTIOMETERS (ADC) ====


void initADCPot(int8_t POT, uint8_t RES) { //== Initialise the ADC for Potentiometers (PA5 and PA6) [POT, NUMBER OF BITS]
  RCC -> AHBENR |= RCC_AHBENR_GPIOAEN;  // Enable clock for GPIOA
  RCC -> APB2ENR |= RCC_APB2ENR_ADCEN;  // Enable the clock for ADC1

// POT Selection
  if (POT == BOTH) {  // Setup for both POT0 and POT1
    GPIOA -> MODER |= (GPIO_MODER_MODER5 + GPIO_MODER_MODER6);  // Set PA5 and PA6 to ANALOG MODE
  
  } else if (POT == POT0) {  // Setup for POT0 (PA5)
    GPIOA -> MODER |= GPIO_MODER_MODER5;  // Set PA5 to ANALOG MODE
  
  } else if (POT == POT1) { // Setup for POT1 (PA6)
    GPIOA -> MODER |= GPIO_MODER_MODER6;  // Set PA6 to ANALOG MODE
  
  }

// Resolution Selection
  if (RES == 12) { // Nothing to set

  } else if (RES == 10) {
    ADC1 -> CFGR1 |= (ADC_CFGR1_RES_0);

  } else if (RES == 8) {
    ADC1 -> CFGR1 |= (ADC_CFGR1_RES_1);

  } else if (RES == 6) {
    ADC1 -> CFGR1 |= (ADC_CFGR1_RES_0 + ADC_CFGR1_RES_1);

  }
  
  ADC1 -> CR |= ADC_CR_ADEN;  // Enable the ADC1
  while (!(ADC1 -> ISR && ADC_ISR_ADRDY));  // Wait for ADC to become ready

}


uint16_t getPot(int8_t POT) { //== Get the pot value - for both pots, run this twice - if speed is needed, configure to grab the data in between conversions
  uint16_t result = 0;
  ADC1 -> CR &= ~ADC_CR_ADEN;  // Disable the ADC for channel selection change
  if (POT == POT0) {
    ADC1 -> CHSELR &= ~ADC_CHSELR_CHSEL6;  // Make sure channel is not on CHANNEL 6  
    ADC1 -> CHSELR |= ADC_CHSELR_CHSEL5;  // Select ADC channel as CHANNEL 5

  } else if (POT == POT1) {
    ADC1 -> CHSELR &= ~ADC_CHSELR_CHSEL5;  // Make sure channel is not on CHANNEL 5  
    ADC1 -> CHSELR |= ADC_CHSELR_CHSEL6;  // Select ADC channel as CHANNEL 6

  }
  ADC1 -> CR |= ADC_CR_ADEN;  // Enable the ADC1
  while (!(ADC1 -> ISR && ADC_ISR_ADRDY));  // Wait for ADC to become ready
  ADC1 -> CR |= ADC_CR_ADSTART; // Start a conversion
  while (ADC1 -> IER && ADC_IER_EOCIE); // Wait for end of conversion
  result = ADC1 -> DR;  // Collect the input!

  return result;
}


//==== TIMER TIM6 ====

void initTIM6(uint16_t userARR, uint16_t userPSC, int8_t interruptEnable, int8_t bufferEnable) { // Initialise the TIM6 given ARR and PSC
  RCC -> APB1ENR |= RCC_APB1ENR_TIM6EN; // Enable the RCC for TIM6

  if (bufferEnable == YES) {
    TIM6 -> CR1 |= (1 << 7); // Enable ARR buffering if specified
  }
  
  TIM6 -> ARR = userARR; // Set the ARR to the value specified
  TIM6 -> PSC = userPSC; // Set the prescalar to the value specified

  if (interruptEnable == YES) {
    TIM6 -> DIER |= (1 << 0); // Enable the interrupt update request if specified
    NVIC_EnableIRQ(TIM6_DAC_IRQn); // Enable the TIM6 interrupt handling in the NVIC                              // NEED TO CHECK THIS WORKS!
  }


  TIM6 -> CR1 |= (1 << 0); // Enable the timer!
}


void clearInterruptTIM6(void) {
  TIM6 -> SR &= ~(1 << 0); // Clear the interrupt flag bit
}


//==== DAC ====

void initDAC(void) {
  RCC -> APB1ENR |= (1 << 29); // Enable the RCC for the DAC
  RCC -> AHBENR |= RCC_AHBENR_GPIOAEN;
  GPIOA-> MODER |= GPIO_MODER_MODER4;// PA4 as analogue
}

void enableDAC(void) { // Highly unlikely this will EVER be used!
  DAC->CR |= DAC_CR_EN1;
  DAC->CR |= DAC_CR_BOFF1; //disable the buffer to increase voltage swing
}

void writeDAC(uint32_t value) {
  DAC -> DHR12R1 = value; // Set the data register with the value specified
}