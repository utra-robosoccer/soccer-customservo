#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <util/atomic.h>
#include <stdlib.h>

#include "constants.h"
#include "inverter.h"

volatile uint8_t* PHASE_CMP[3];

const uint8_t PHASES[6] = {
	0b010011,
	0b000111,
	0b001101,
	0b011100,
	0b110100,
	0b110001
};

ISR(TIMER0_COMPA_vect) {
	if((phase_idx[0] >> 8) & 1)
		PORTC |= 0b11;
	else
		PORTC &= 0b111100;
	// put_commutation(OFF_MASK);
	// put_commutation(PHASES[(phase_idx[0] >> 8) % 6]);
	// TIFR0 |= 0b10;
}
ISR(TIMER0_COMPB_vect) {
	// uint8_t next_pattern = ((PORTD & 0x11000000) >> 2) | (-((phase_idx[1] >> 8) & 1) & 0b1100) | (PORTC & 0b11); // crosses port boundaries: have to be a bit more careful
	// put_commutation(next_pattern);
	// PORTC |= (PORTC & 0b110011) | (-((phase_idx[1] >> 8) & 1) & 0b1100);
	if((phase_idx[1] >> 8) & 1)
		PORTC |= 0b1100;
	else
		PORTC &= 0b110011;
	// TIFR0 |= 0b100;
}
ISR(TIMER1_COMPB_vect) {
	// PORTC |= (PORTC & 0b001111) | (-((phase_idx[2] >> 8) & 1) & 0b110000); // abuse two's complement
	if((phase_idx[2] >> 8) & 1)
		PORTC |= 0b110000;
	else
		PORTC &= 0b001111;
	// TIFR1 |= 0b100;
}

ISR(TIMER0_OVF_vect) {
	PORTC &= OFF_MASK; // keep grounded nodes grounded, open upper nodes
	// PHASES[(phase_idx[0] >> 8) % 6] & 
	// TIFR0 |= 0b1;
}

ISR(TIMER2_COMPA_vect) {
	// for(uint8_t i = 0; i < 3; i++) {
	// 	*(PHASE_CMP[i]) = 0xFF - HALF_SINE[(phase_idx[i]++) & 0xFF];
	// }
	OCR0A = 0xFF - (HALF_SINE[(phase_idx[0]++) & 0xFF]);
	OCR0B = 0xFF - (HALF_SINE[(phase_idx[1]++) & 0xFF]);
	OCR1B = 0xFF - (HALF_SINE[(phase_idx[2]++) & 0xFF]);
	if(OCR2A > 3)
		OCR2A = OCR2A - 1;
	
	// OCR2A = OCR2A >> 1 | ; // = 0.9f * OCR2A;
	// PORTB = (PORTB & ~(1 << 3)) | ((phase_idx[0] & 0x100) >> 5);
}

void port_init() {
	// DDRB = 0x3F;
	// DDRD = 0b111 << 5;
	DDRC = 0x3F;
}
void timer_init() {
	GTCCR = 1 << 7 | 0b11;
	
	TCCR0A = 0b11;
	TCCR0B = 0b10; // 8x prescaling (47kHz PWM)
	TCNT0 = 0;
	// OCR0A = 250; // temp, to test one phase
	TIMSK0 = 0b111;
	// TIMSK0 = 0b111; // enable all interrupts, including overflow (this will beat the drum)
	
	TCCR1A = 0b00;
	TCCR1B = 0b01 << 3 | 0b010; // CTC + 8x prescaling
	OCR1A = 0xFF; // reduce to 8-bit timer
	TCNT1 = 0;
	TIMSK1 = 0b100; // enable CMPB interrupt
	
	TCCR2A = 0b10;
	TCCR2B = 0b100; // OCR2A sets top + 256x prescaling, 732Hz (for 512-valued sine, motor phase is 1.4Hz to begin with)
	OCR2A = 0xFF; // 0xFF; // start with max delay
	TIMSK2 = 0b000;
	TIMSK2 = 0b010; // enable TOV interrupts only (will do the work of condensing OCR2A)
	
	GTCCR &= ~(1 << 7);
}

int main(void) {
	PHASE_CMP[0] = (volatile uint8_t*)0x27; // &OCR0A; // 0x27;// &OCR0A;
	PHASE_CMP[1] = (volatile uint8_t*)0x28; // &OCR0B; // 0x28;// &OCR0B;
	PHASE_CMP[2] = (volatile uint8_t*)0x8A; // &OCR1BL; // 0x8A;// &OCR1B;
	
	port_init();
	timer_init();
	sei();
	
	for(;;);
}