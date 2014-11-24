/*
 * woodboxstm32f051C6.h
 *
 *  Created on: Oct 3, 2014
 *      Author: Sean Wood - WDXSEA003
 *      		University of Cape Town
 *
 *  This is the header file.
 *
 *  This WoodBox is a collection of commonly used and generally
 *  pesky things that one wants to do with the STM32F051C6, but
 *  don't want to have to place in one's source! Yay!
 */

#ifndef WOODBOXSTM32F051C6_H_
#define WOODBOXSTM32F051C6_H_

#include <stdint.h>
#include "stm32f0xx.h"
#include "eeprom_lib.h"
#include "temp_sensor_lib.h"
#include "lcd_stm32f0.h"

enum POTSEL {POT0, POT1, BOTH};
enum BOOL {NO, YES};
enum TRIGGER {RISE, FALL, R_AND_F};

void programError(); // For when things get tight and I want to see what is going on

void initLEDs(void);
void initPB(void);
void initADCPot(int8_t POT, uint8_t RES); // Initialise the ADC for Potentiometers (PA5 and PA6) [POT, NUMBER OF BITS]
void initDAC(void);
                                         // POT selection can be a little redundant since both pots are by default set to analog input it seems?
void initTIM6(uint16_t userARR, uint16_t userPSC, int8_t interruptEnable, int8_t bufferEnable); // Initialise Timer 6 [ARR, PSC, INTERUPT ENABLE (YES/NO), BUFFER ENABLE (YES/NO)]

void delayms(uint32_t length); // Millisecond delay function [Delay Length]                 | ONLY FOR 8 MHz
void delaypointms(uint32_t length); // Point 1 millisecond delay function [Delay Length]    | CPU SPEED!

uint16_t getPot(int8_t POT); // Get the pot value [POT#]
int8_t getPB(int8_t button); // Get the button state [SW#]

void incrementLEDs(int8_t amount); // Increment the LEDs by a given value [Increment Value]
void clearInterruptTIM6(void); // Clears the interrupt flag bit for TIM6
void enableInterruptPB(int8_t button, int8_t mode); // Enables the interrupt on the specified PB [PB, trigger MODE (RISE, FALL, R_ANDF)]
void enableDAC();
void writeDAC(uint32_t value); // Write the value specified to the DAC (12 bit, right aligned register) and enable the DAC [VALUE (0~4095 mV)]

#endif /* WOODBOXSTM32F051C6_H_ */
