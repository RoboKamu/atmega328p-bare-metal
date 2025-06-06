#include "gpio.h"
#include <stdarg.h>

void gpio_init(volatile uint8_t *ddr, gpio_mode_t dir, uint8_t pin, gpio_state_t pu_state){
    if (dir == GPIO_MODE_OUT){
        *ddr |= (1 << pin);
        return;
    }
    if (dir != GPIO_MODE_IN) return;
    
    *ddr |= (0 << pin);
    
    uint8_t state = pu_state;
    if (state == GPIO_PULLUP_OFF) return;

    // write logic one to PORTxn for pin configured as input pin to setup pullup
    if (*ddr == DDRB) {
        PORTB |= (1 << pin);
    }
    else if (*ddr == DDRC){
        PORTC |= (1 << pin);
    }
    else if (*ddr== DDRD){
        PORTC |= (1 << pin);
    }
}

void gpio_init_multi_pin(volatile uint8_t *ddr, gpio_mode_t dir, gpio_state_t pu_state, uint8_t pin_count, ...){
    va_list args;
    va_start(args, pin_count);

    for (uint8_t i=0; i<pin_count; i++){
        uint8_t pin = va_arg(args, int);
        gpio_init(ddr, dir, pin, pu_state);
    }
    va_end(args);
}

void gpio_deinit(volatile uint8_t *ddr, gpio_mode_t dir, uint8_t pin, gpio_state_t pu_state){
    
    if (dir == GPIO_MODE_IN){
        // Atmega has default mode as input, see table 14-1 in the datasheet
        // just turn the pullup to OFF if already input 
        // write logic one to PORTxn for pin configured as input pin to setup pullup
        if (pu_state == GPIO_PULLUP_OFF) return; 

        if (*ddr == DDRB) {
            PORTB &= ~(1 << pin);
        }
        else if (*ddr == DDRC){
            PORTC &= ~(1 << pin);
        }
        else if (*ddr== DDRD){
            PORTC &= ~(1 << pin);
        }
        return; 
    }
    // reset direction to original (input) 
    *ddr &= ~(1 << pin);
}


uint8_t gpio_read(volatile uint8_t *port, uint8_t pin){
    return ( (*port) & (1 << pin) ) != 0;
}

void gpio_write(volatile uint8_t *port, uint8_t pin, uint8_t val){
    if (val){
        (*port) |= (1 << pin);
    } else {
        (*port) &= ~(1 << pin);
    }
}
