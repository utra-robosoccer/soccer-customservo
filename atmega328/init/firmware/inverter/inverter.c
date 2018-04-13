#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <util/atomic.h>
#include <stdlib.h>

#include "../constants.h"
#include "inverter.h"

// volatile uint8_t* PHASE_CMP[3];

ISR(TIMER2_COMPA_vect) {
	// for(uint8_t i = 0; i < 3; i++) {
	// 	*(PHASE_CMP[i]) = 0xFF - HALF_SINE[(phase_idx[i]++) & 0xFF];
	// }
	uint8_t phase_masks[3] = { (phase_idx[0] >> 8) & 1, (phase_idx[1] >> 8) & 1, (phase_idx[2] >> 8) & 1 };
	OCR0A = HALF_SINE[(phase_idx[0]++) & 0xFF];
	OCR0B = HALF_SINE[(phase_idx[1]++) & 0xFF];
	OCR1B = HALF_SINE[(phase_idx[2]++) & 0xFF];
	
	TCCR0A = (TCCR0A & 0b00001111) | (-(!phase_masks[0]) & 0b11000000) | (-(!phase_masks[1]) & 0b00110000);
	TCCR1A = (TCCR1A & 0b11001111) | (-(!phase_masks[2]) & 0b00110000); // phase_masks[2]
	
	PORTC = (PORTC & 0b11000111) | (phase_masks[2] << 5) | (phase_masks[1] << 4) | (phase_masks[0] << 3);
	PORTB |= 1 << 5;
	// if(OCR2A > 3)
	// 	OCR2A = OCR2A - 1;
	
	// OCR2A = OCR2A >> 1 | ; // = 0.9f * OCR2A;
	// PORTB = (PORTB & ~(1 << 3)) | ((phase_idx[0] & 0x100) >> 5);
}

void port_init() {
	// DDRB = 0x3F;
	// DDRD = 0b111 << 5;
	
	// PWM pins
	DDRD = 1 << 5 | 1 << 6;
	DDRB = 1 << 2 | 1 << 5;
	
	// control pins
	DDRC |= 0b111 << 3;
	PORTB = 0;
	PORTD = 0;
}
void timer_init() {
	GTCCR = 1 << 7 | 0b11;
	
	TCCR0A = 0b11;
	TCCR0B = 0b010; // 8x prescaling (47kHz PWM)
	TCNT0 = 0;
	// OCR0A = 250; // temp, to test one phase
	TIMSK0 = 0;
	// TIMSK0 = 0b111; // enable all interrupts, including overflow (this will beat the drum)
	
	TCCR1A = 0b01;
	TCCR1B = 0b01010; // 8x prescaling // 0b01 << 3 | // CTC
	OCR1A = 0xFF; // reduce to 8-bit timer
	TCNT1 = 0;
	TIMSK1 = 0;
	// TIMSK1 = 0b100; // enable CMPB interrupt
	
	TCCR2A = 0b10;
	TCCR2B = 0b100; // OCR2A sets top + 256x prescaling, 732Hz (for 512-valued sine, motor phase is 1.4Hz to begin with)
	OCR2A = 0x20; // 0xFF; // start with max delay
	TIMSK2 = 0b010;
	// TIMSK2 = 0b010; // enable TOV interrupts only (will do the work of condensing OCR2A)
	
	GTCCR &= ~(1 << 7);
}
int main(void) {
	// PHASE_CMP[0] = (volatile uint8_t*)0x27; // &OCR0A; // 0x27;// &OCR0A;
	// PHASE_CMP[1] = (volatile uint8_t*)0x28; // &OCR0B; // 0x28;// &OCR0B;
	// PHASE_CMP[2] = (volatile uint8_t*)0x8A; // &OCR1BL; // 0x8A;// &OCR1B;
	
	port_init();
	timer_init();
	sei();
	
	for(;;);
}