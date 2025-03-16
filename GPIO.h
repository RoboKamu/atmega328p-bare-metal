#pragma once
#include <stdint.h>
#include "avr/io.h"

typedef enum{
    GPIO_MODE_OUT,
    GPIO_MODE_IN
} gpio_mode_t;

typedef enum{
    GPIO_PULLUP_OFF,
    GPIO_PULLUP_ON
} gpio_state_t;

/**
 * Initilize GPIO pin. pu_state can only be ON for input pin
 * @param ddr DDRx, pointer to DDR port
 * @param pin DDRxn, pin number to set direction
 * @param dir GPIO_MODE_OUT or GPIO_MODE_IN for output resp. input
 * @param pu_state GPIO_PULLUP_ON for on and GPIO_PULLUP_OFF for off
 * @see gpio_init_multi_pin() for initializing multiple pins
 */
void gpio_init(volatile uint8_t *ddr, gpio_mode_t dir, uint8_t pin, gpio_state_t pu_state);

/**
 * Alternativt way of initlializing multiple GPIO pins. pu_state can only be ON for input pin. 
 * exmaple usage: gpio_init_multi_pin(DDRB, GPIO_MODE_OUT, GPIO_PULLUP_OFF, PB5, PB2, PB7)
 * @param ddr DDRx, pointer to DDR port
 * @param pin DDRxn, pin number to set direction
 * @param dir GPIO_MODE_OUT or GPIO_MODE_IN for output resp. input
 * @param pu_state GPIO_PULLUP_ON for on and GPIO_PULLUP_OFF for off
 * @see gpio_init() for initializing singular pins
 */
void gpio_init_multi_pin(volatile uint8_t *ddr, gpio_mode_t dir, gpio_state_t pu_state, uint8_t pin_count, ...);

/**
 * Denitialize pin
 * @param ddr DDRx, pointer to DDR port
 * @param pin DDRxn, pin number to set direction
 * @param dir GPIO_MODE_OUT or GPIO_MODE_IN for output resp. input
 */
void gpio_deinit(volatile uint8_t *ddr, gpio_mode_t dir, uint8_t pin, gpio_state_t pu_state);

/**
 * Read GPIO pin value
 * @param port PINx
 * @param pin PINxn
 * @retval uint8_t 
 */
uint8_t gpio_read(volatile uint8_t *port, uint8_t pin);

/**
 * Write to GPIO pin
 * @param port PINx
 * @param pin PINxn
 * @param val 1 for HIGH, 0 for LOW
 */
void gpio_write(volatile uint8_t *port, uint8_t pin, uint8_t val);
