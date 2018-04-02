#include <util/delay.h>

#include "../main.h"
#include "../State.h"
#include "FirstEncoderTickState.h"


const uint8_t FirstEncoderTickState::PHASES[3] = {
	0b000111,
	0b011100,
	0b110001
};
FirstEncoderTickState::FirstEncoderTickState(void) {}
void FirstEncoderTickState::setup(const State& previous_state) {
	Encoder::position = 0;
}
void FirstEncoderTickState::spin(void) {
	uint8_t sine_idx = 0;
	uint8_t phase = 0;
	while(abs(Encoder::position) < 18) { // one tick on a 64 CPR encoder
		if(sine_idx == 127) {
			// give up on this phase if we've made it to
			// the peak of the sine w/o moving
			sine_idx = 0;
			phase = (phase + 1) % 3;
		}
		OCR0A = HALF_SINE[sine_idx++] >> 2; // max 25% duty
		_delay_ms(50);
	}
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