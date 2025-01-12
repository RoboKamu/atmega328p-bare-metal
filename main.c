/**
 *  The following is a simple example code of the rotary encoder KY-040
 *  Simple IF for handeling Falling edge
 *  Falling edge CLK: DT == LOW => CW
 *  Falling edge CLK: DT == HIGH => CCW
 *  SW is blocking, while holding down SW the program keeps LEDs ON
 *  CLK and DT connected to pulldown resistor (50kOhm)
 *  SW connected to pullup resistor (50kOhm)
 * 
 */


#include "avr/io.h"             // include the IO definitions for the defined microcontroller

#define LED_G   PORTB3
#define LED_Y   PORTB2
#define CLK     PORTB5
#define DT      PORTB4
#define SW      PORTB0

int main(void){

    // init output pins 3-2 (pins are input by default)
    DDRB = (1 << LED_G) | (1 << LED_Y);

    unsigned pinALast = PINB & (1 << CLK);
    unsigned aVal;
    int8_t bCW=-1;

    // forever...
    while(1){
        
        aVal = PINB & (1 << CLK);             // read new CLK
        if (aVal != pinALast && aVal == 0) {  // change in state occured (falling edge)
            if ((PINB & (1 << DT)) != aVal)   // ..CLK changed first 
                bCW = 1;                      // ....clockwise!
            else {                            // ..DT changed fist
                bCW = 0;                      // ....C-Clocwise!
            }

            if (bCW==1){                      // CW...  
                PORTB |= (1 << LED_G);        // ...turn green LED ON
                PORTB &= ~(1 << LED_Y);       // ...and yellow off!!
            } 
            else if (bCW==0) {                // CCW... 
                PORTB |= (1 << LED_Y);        // ...turn yellow LED ON
                PORTB &= ~(1 << LED_G);       // ...and green off!!
            }
        }
        if ((PINB & (1 << SW)) == 0){         // switch pushed, both LEDS on 
            do{                               // keep on while pushed
                PORTB |= (1 << LED_G) | (1 << LED_Y);
            } while ((PINB & (1 << SW)) == 0);
            PORTB = 0x00;   // reset LEDs after SW open again
        }

        bCW=-1;
        pinALast = aVal;
    }
}
