#pragma once

#include "avr/io.h"

/** initialize the timer peripheral for delay counting */
void delay_init();

/** timer/counter0 based delay in milliseconds */
void delay_ms(int ms);
