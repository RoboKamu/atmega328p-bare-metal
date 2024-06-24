#include "avr/io.h"             // include the IO definitions for the defined microcontroller
#include "util/delay.h"         // delay file 

int main(void){
    // set PORT5 as an output
    DDRB = DDRB | (1 << DDB5);

    // forever...
    while(1){
        // set PORTB5
        PORTB = PORTB | (1 << PORT5);

        // wait 
        _delay_ms(1000);

        // unset PORTB5
        PORTB = PORTB & ~(1 << PORTB5);

        // wait somemore
        _delay_ms(1000);
    }
}