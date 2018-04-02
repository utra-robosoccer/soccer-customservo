#include <avr/io.h>
#include "init.h"

void port_init() {
	DDRC = 0b111; // output C0-C2 (motor phases)
	DDRB = 0x0F | (1 << 5); // output all motor phases, indicator and on pin 13
	PORTB &= ~(1 << 5);
	
	EICRA = 0b01 << 2 | // logic-level change on INT1 generates INT1
	        0b01;  // logic-level change on INT0 generates INT0
	EIMSK |= 0b11; // enable INT1 and INT0
}
void timer_init() {
}
void ac_init() {
	// ADC0 implicitly from ADMUX & 0xF = 0
	ADCSRB = 1 << 6 | // ADC0 input to comparator
	         0b000; // free-running mode
	ACSR = // 1 << 3 | // enable interrupts
	       1 << 2; // | // enable AC input capture
	       // 0b11; // toggle triggers interrupt
}