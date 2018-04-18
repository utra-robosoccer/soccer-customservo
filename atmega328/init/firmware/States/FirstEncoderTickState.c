#include <util/delay.h>
#include <stdlib.h>

#include "../State.h"
#include "../Encoder.h"
#include "../constants.h"
#include "FirstEncoderTickState.h"

FirstEncoderTickState::FirstEncoderTickState(void) {}
void FirstEncoderTickState::setup(const State& previous_state) {
	Encoder::position = 1;
	
	// setup TIMER0 to push PWM
	TCCR0A = 0b11 << 4 | 0b11; // fast-PWM
	TCCR0B = 0b0010; // 8x prescale, normal mode continued
	TIMSK0 = 0;
	PORTB &= ~(1 << 5);
	PUT(OFF_MASK_P, OFF_MASK_N);
	// TIMSK0 = 0b011; // overflow and compa
}
void FirstEncoderTickState::spin(void) {
	uint8_t sine_idx = 127;
	while(abs(Encoder::position) < 18) {
		if(sine_idx == 127) {
			// give up on this phase if we've made it to
			// the peak of the sine w/o moving
			sine_idx = 0;
			this->phase = !this->phase; // (this->phase + 1) % 3;
			// PORTB = (PORTB & 0b11011111) | (this->phase << 5);
			// TCCR0A = (TCCR0A & 0b00001111) | (-(!this->phase) & 0b11000000) | (-(this->phase) & 0b00110000);
			PORTC = (PORTC & 0b11000111) | (this->phase << 5) | (!this->phase << 3);
		}
		OCR0A = HALF_SINE[sine_idx]; // max 25% duty
		OCR0B = HALF_SINE[sine_idx++]; // max 25% duty
		_delay_ms(4);
	}
	this->direction = Encoder::position > 0; // hope it doesn't jitter back to 0 here! with our high-res encoder this isn't a problem, but there is some robustness (debouncing) issues with the true, lower-res encoders
	PROGRAM_STATE = INDUCTANCE_ANGLE;
}
void FirstEncoderTickState::ISR_entry(uint8_t vect) {}
// void FirstEncoderTickState::teardown(void) {}