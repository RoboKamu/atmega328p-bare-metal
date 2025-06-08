#include "gpio.h"
#include "delay.h"

int main(){
	gpio_init(&DDRB, GPIO_MODE_OUT, PORTB5, GPIO_PULLUP_OFF);
	delay_init();
	while(1){
		// toggle LED every second 
		gpio_write(&PORTB, PORTB5, !gpio_read(&PORTB, PINB5));
		delay_ms(1000);
	}
}