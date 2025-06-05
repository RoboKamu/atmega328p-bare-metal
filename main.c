#include "gpio.h"

int main(){
	gpio_init(&DDRB, GPIO_MODE_OUT, PORTB5, GPIO_PULLUP_OFF);
	while(1){
		gpio_write(&DDRB, PORTB5, 1);
		for(volatile unsigned int i=0; i<0xFFFF; i++) { /* delay.. */ }
		gpio_write(&DDRB, PORTB5, 0);
		for(volatile unsigned int i=0; i<0xFFFF; i++) { /* delay.. */ }		
	}
}
