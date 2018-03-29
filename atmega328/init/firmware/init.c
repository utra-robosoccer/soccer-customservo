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
	TCCR0A = 0b00 << 6 | // don't touch OC0A // 0b11 << 6 | // OC0A inverted PWM (NMOS control)
	         0b00 << 4 | // don't touch OC0B
	         0b11; // fast PWM (needed to detect on-off cycles)
	TCCR0B = 0 << 3 | // fast PWM (continued)
	         0b011; // 64x prescaling (1024Hz PWM)
	         // 0b101; // 1024x prescaling (64Hz)
	OCR0A = 0;
	TIMSK0 = 0b011; // enable compare match A; enable overflow interrupt
	
	TCCR1A = 0b00 << 6 | // don't touch OC0A
	         0b00 << 4 | // don't touch OC0B
	         0b00000; // Normal mode
	TCCR1B |= 0b11 << 6 | // input capture: rising edge with noise cancellation
	          0b01; // no prescaling
	// TIMSK1 = 1 << 5; // enable input capture interrupt
}
void ac_init() {
	// ADC0 implicitly from ADMUX & 0xF = 0
	ADCSRB = 1 << 6 | // ADC0 input to comparator
	         0b000; // free-running mode
	ACSR = // 1 << 3 | // enable interrupts
	       1 << 2; // | // enable AC input capture
	       // 0b11; // toggle triggers interrupt
}