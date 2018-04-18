#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <util/atomic.h>
#include <stdlib.h>
#include <avr/eeprom.h>

#include "../constants.h"
#include "../pin_util.h"
#include "inverter.h"
#include "../Encoder.h"

// volatile uint8_t* PHASE_CMP[3];

#define SET_PHASE(phase) {\
	uint8_t phase_masks[3] = { (phase >> 8) & 1, ((phase + 171) >> 8) & 1, ((phase + 341) >> 8) & 1 };\
	OCR0B = HALF_SINE[phase & 0xFF];\
	OCR0A = HALF_SINE[(phase + 171) & 0xFF];\
	OCR2A = HALF_SINE[(phase + 341) & 0xFF];\
	TCCR0A = (TCCR0A & 0b00001111) | (-(!phase_masks[1]) & 0b11000000) | (-(!phase_masks[0]) & 0b00110000);\
	TCCR2A = (TCCR2A & 0b00111111) | (-(!phase_masks[2]) & 0b11000000);\
	PORTC = (PORTC & 0b11000111) | (phase_masks[2] << 5) | (phase_masks[0] << 4) | (phase_masks[1] << 3);\
}

void port_init() {
	// DDRB = 0x3F;
	// DDRD = 0b111 << 5;
	
	// PWM pins
	DDRD = 1 << 5 | 1 << 6;
	DDRB = 1 << 3 | 1 << 5;
	
	// control pins
	DDRC |= 0b111 << 3;
	PORTB = 0;
	PORTD = 0;
	
	EICRA = // 0b01 << 2 | // logic-level change on INT1 generates INT1
	        0b01;  // logic-level change on INT0 generates INT0
	EIMSK |= 0b01; // enable INT1 and INT0
}
void timer_init() {
	// GTCCR = 1 << 7 | 0b11;
	
	TCCR0A = 0b11;
	TCCR0B = 0b010; // 8x prescaling (47kHz PWM)
	TCNT0 = 0;
	// OCR0A = 250; // temp, to test one phase
	TIMSK0 = 0;
	// TIMSK0 = 0b111; // enable all interrupts, including overflow (this will beat the drum)
	
	// TCCR1A = 0b01;
	// TCCR1B = 0b01010; // 8x prescaling // 0b01 << 3 | // CTC
	// OCR1A = 0xFF; // reduce to 8-bit timer
	// TCNT1 = 0;
	// TIMSK1 = 0;
	// TIMSK1 = 0b100; // enable CMPB interrupt
	
	TCCR2A = 0b11;
	TCCR2B = 0b010; // OCR2A sets top + 256x prescaling, 732Hz (for 512-valued sine, motor phase is 1.4Hz to begin with)
	TCNT2 = 0;
	TIMSK2 = 0;
	
	// GTCCR &= ~(1 << 7);
}
int main(void) {
	// PHASE_CMP[0] = (volatile uint8_t*)0x27; // &OCR0A; // 0x27;// &OCR0A;
	// PHASE_CMP[1] = (volatile uint8_t*)0x28; // &OCR0B; // 0x28;// &OCR0B;
	// PHASE_CMP[2] = (volatile uint8_t*)0x8A; // &OCR1BL; // 0x8A;// &OCR1B;
	
	uint16_t encoder_bias = ENCODER_DIV_MOTOR_GCD << 3;
	Encoder::position = encoder_bias;
	
	port_init();
	timer_init();
	sei();
	
	uint16_t last_encoder = 0;
	uint16_t current_encoder;
	uint16_t last_time = TCNT1;
	uint16_t last_period = 1024;
	uint8_t majority = 0;
	uint16_t i = 0;
	
	PUT(0, 0);
	// PORTC |= 1 << 3;
	// TCCR2A |= 0b11 << 6;
	// OCR2A = 20;
	// for(;;);
		
	for(;;) {
		current_encoder = Encoder::position;
		if(current_encoder > encoder_bias + ENCODER_DIV_MOTOR_GCD || current_encoder < encoder_bias) {
			// PORTB ^= 1 << 5;
			ATOMIC_BLOCK(ATOMIC_FORCEON) {
				current_encoder = encoder_bias + (Encoder::position % ENCODER_DIV_MOTOR_GCD);
				Encoder::position = current_encoder;
			}
		}
		
		if(current_encoder != last_encoder) {
			if(++majority > MAJORITY_THRESHOLD) {
				// uint16_t current_T1;
				// ATOMIC_BLOCK(ATOMIC_FORCEON) {
				// 	current_T1 = TCNT1;
				// }
				// last_encoder = current_encoder;
				
				// last_period = current_T1 - last_time;
				// last_time = current_T1;
				
				uint16_t next_position = ((uint32_t)current_encoder * NUM_HALF_SINE) * MOTOR_PHASES_PER_ENCODER_TICK;
				// if(i < 512)
				// 	eeprom_write_word((uint16_t*)((i++) << 1), next_position);
				// else
				// 	PORTB |= 1 << 5;
				
				ATOMIC_BLOCK(ATOMIC_FORCEON) {
					SET_PHASE(next_position);
				}
				majority = 0;
				last_encoder = current_encoder;
			}
		}
		else {
			majority--;
		}
	}
}