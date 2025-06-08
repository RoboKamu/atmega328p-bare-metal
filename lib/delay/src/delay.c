#include "delay.h"

void delay_init(){
    // write zero to PRTIMO bit to enable Timer/Counter0 module 
    PRR &= ~(1 << PRTIM0);
    // configure timer to normal mode 
    TCCR0A = 0x0;
    // set prescaler to 64 for the 16 MHz external clk 
    TCCR0B |= (1 << CS00) | (1 << CS01) ; 
}

void delay_ms(int ms){
    // with a timer0 being 8 bits the percision of one overflow is 1.024 ms
    // this accuracy is enough for the intended purposes of the delay function  
    while (--ms >= 0) {
        // wait for overflow (256 cycles)
        while (!(TIFR0 & (1 << TOV0))); 
        // reset flag
        TIFR0 |= (1 << TOV0);
    }    
}