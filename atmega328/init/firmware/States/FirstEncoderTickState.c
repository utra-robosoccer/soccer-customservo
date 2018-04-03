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
	TCCR0A = 0b00; // normal mode
	TCCR0B = 0b0010; // 8x prescale, normal mode continued
	TIMSK0 = 0b011; // overflow and compa
}
void FirstEncoderTickState::spin(void) {
	uint8_t sine_idx = 0;
	while(abs(Encoder::position) < 18) { // one tick on a 64 CPR encoder
		if(sine_idx == 127) {
			// give up on this phase if we've made it to
			// the peak of the sine w/o moving
			sine_idx = 0;
			this->phase = (this->phase + 1) & 1;
		}
		OCR0A = HALF_SINE[sine_idx++]; // max 25% duty
		_delay_ms(50);
	}
	this->direction = Encoder::position > 0; // hope it doesn't jitter back to 0 here! with our high-res encoder this isn't a problem, but there is some robustness (debouncing) issues with the true, lower-res encoders
	PROGRAM_STATE = INDUCTANCE_ANGLE;
}
void FirstEncoderTickState::ISR_entry(uint8_t vect) {
	switch(vect) {
		case TIMER0_OVF_vect_num:
			this->t0_ovf();
			return;
		case TIMER0_COMPA_vect_num:
			this->t0_compa();
			return;
	}
	
	// switch(vect) {
	// 	case TIMER0_OVF_vect:
			
	// 		break;
	// }
}
// void FirstEncoderTickState::teardown(void) {}