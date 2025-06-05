/**
 *  The following is a simple example code of the rotary encoder KY-040
 *  Simple IF for handeling Falling edge
 *  Falling edge CLK: DT == LOW => CW
 *  Falling edge CLK: DT == HIGH => CCW
 *  SW is blocking, while holding down SW the program keeps LEDs ON
 *  CLK and DT connected to pulldown resistor (50kOhm)
 *  SW connected to pullup resistor (50kOhm)
 *  DATASHEET: https://www.handsontec.com/dataspecs/module/Rotary%20Encoder.pdf 
 */

#include "GPIO.h"

#define LED_G   PORTB3
#define LED_Y   PORTB2
#define CLK     PORTB5
#define DT      PORTB4
#define SW      PORTB0

int main(void){
    // init output pins 3-2 (pins are input by default)
    gpio_init_multi_pin(&DDRB, GPIO_MODE_OUT, GPIO_PULLUP_OFF, 2, LED_G, LED_Y);
    gpio_init(&DDRB, GPIO_MODE_IN, SW, GPIO_PULLUP_ON);

    uint8_t pinALast = gpio_read(&PINB, CLK);
    uint8_t aVal;
    int8_t bCW=-1;

    while(1){
        aVal = gpio_read(&PINB, CLK);             // read new CLK
        if (aVal != pinALast && aVal == 0) {      // change in state occured (falling edge)
            if (gpio_read(&PINB, DT) != aVal)     // ..CLK changed first 
                bCW = 1;                          // ....clockwise!
            else {                                // ..DT changed fist
                bCW = 0;                          // ....C-Clocwise!
            }

            if (bCW==1){                      // CW...  
                gpio_write(&PORTB, LED_G, 1);
                gpio_write(&PORTB, LED_Y, 0);
            } 
            else if (bCW==0) {                // CCW... 
                gpio_write(&PORTB, LED_Y, 1);
                gpio_write(&PORTB, LED_G, 0);
            }
        }
        if (gpio_read(&PINB, SW) == 0){         // switch pushed, both LEDS on 
            do{                               // keep on while pushed
                PORTB |= (1 << LED_G) | (1 << LED_Y);
            } while (gpio_read(&PINB, SW) == 0);
            PORTB = 0x00;   // reset LEDs after SW open again
        }

        bCW=-1;
        pinALast = aVal;
    }
}
