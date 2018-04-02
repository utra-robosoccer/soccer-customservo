#include "../main.h"
#include "../State.h"
#include "InductanceAngleState.h"

const uint8_t InductanceAngleState::adc[3] = {
	4, 4, 5 // NEEDS CORRECTING
};
const uint8_t InductanceAngleState::PHASES[3] = {
	0b01110100,
	0b01110001,
	0b01010011
};
const uint8_t InductanceAngleState::RESTING[3] = {
	0b01010100,
	0b01000101,
	0b01000101
};

InductanceAngleState::InductanceAngleState(void) {
	// register_ISR(TIMER0_OVF_vect_num, INDUCTANCE_ANGLE, &this->t0_ovf);
	// register_ISR(TIMER0_COMPA_vect_num, INDUCTANCE_ANGLE, &this->t0_compa);
	// register_ISR(TIMER1_CAPT_vect_num, INDUCTANCE_ANGLE, &this->t1_capt);
}
void InductanceAngleState::setup(const State& previous) {
	// clear timing buffers
	for(uint8_t i = 0; i < 3; i++)
		sat_timing_buffers[i].counter = 0;
	
	// initialize AC
	// ADC0 implicitly from ADMUX & 0xF = 0
	ADCSRB |= 1 << 6; // ADC0 input to comparator
	ADCSRB = (ADCSRB & ~(0b111)) | 0b000; // free-running mode
	ACSR |= 1 << 3 | // enable interrupts
	        1 << 2; // | // enable AC input capture
	       // 0b11; // toggle triggers interrupt
	
	// setup TIMER1 for timing the threshold crossings
	TCCR1A = 0b00 << 6 | // don't touch OC0A
	         0b00 << 4 | // don't touch OC0B
	         0b00000; // Normal mode
	TCCR1B |= 0b11 << 6; // input capture: rising edge with noise cancellation
   TCCR1B = (TCCR1B & ~(0b11)) | 0b01; // no prescaling
}
void InductanceAngleState::spin(void) {
	uint8_t buffers_full;
	do {
		buffers_full = 1;
		for(uint8_t i = 0; i < 3; i++) {
			if(sat_timing_buffers[i].counter < INDUCTANCE_ANGLE_SAMPLES) {
				buffers_full = 0;
				break;
			}
		}
	}
	while(!buffers_full);
	
	PROGRAM_STATE = RUNNING;
}

void InductanceAngleState::ISR_entry(uint8_t vect) {
	switch(vect) {
		case TIMER0_OVF_vect_num:
			this->t0_ovf();
			return;
		case TIMER0_COMPA_vect_num:
			this->t0_compa();
			return;
		case TIMER1_CAPT_vect_num:
			this->t1_capt();
			return;
	}
}

void InductanceAngleState::t0_ovf(void) {
	phase = (phase + 1) % 3;
	triggered = 0;
	
	TCNT1 = 0; // reset threshold timer
	
	TIMSK1 &= ~(1 << 5); // disable input capture interrupt
	ADMUX = (ADMUX & ~(0b11)) | this->adc[this->phase];
	
	put_commutation(OFF_MASK); // de-energize for a bit to avoid short
	put_commutation(PHASES[phase]);
	TIMSK1 |= 1 << 5; // re-enable input capture interrupt
}
void InductanceAngleState::t0_compa(void) {
	put_commutation(RESTING[phase]);
}
void InductanceAngleState::t1_capt(void) {
	if(!triggered && sat_timing_buffers[phase].counter < INDUCTANCE_ANGLE_SAMPLES) {
		// PORTB ^= 0b1000;
		sat_timing_buffers[phase].buffer[sat_timing_buffers[phase].counter++] = ICR1;
		triggered = 1;
	}
	ACSR |= 1 << 4;
}
