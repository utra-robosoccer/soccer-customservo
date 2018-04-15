#include "../main.h"
#include "../State.h"
#include "../Encoder.h"
#include "InductanceAngleState.h"

#include <math.h>
#include <avr/eeprom.h>

const uint8_t InductanceAngleState::adc[2] = {
	2, 2
};
const commutation_t InductanceAngleState::PHASES[2] = {
	{ .p = 0b100, .n = 0b010 },
	{ .p = 0b100, .n = 0b001 }
};
const commutation_t InductanceAngleState::RESTING[2] = {
	{ .p = 0b000, .n = 0b010 },
	{ .p = 0b000, .n = 0b001 }
};

InductanceAngleState::InductanceAngleState(void) {
	// register_ISR(TIMER0_OVF_vect_num, INDUCTANCE_ANGLE, &this->t0_ovf);
	// register_ISR(TIMER0_COMPA_vect_num, INDUCTANCE_ANGLE, &this->t0_compa);
	// register_ISR(TIMER1_CAPT_vect_num, INDUCTANCE_ANGLE, &this->t1_capt);
}
void InductanceAngleState::setup(const State& previous) {
	if(previous.state_type() == FIRST_ENCODER_TICK) {
		// this->FirstEncoderTick_phase = ((const FirstEncoderTickState&)previous).phase;
		this->FirstEncoderTick_direction = ((const FirstEncoderTickState&)previous).direction;
	}
	else {
		// this->FirstEncoderTick_phase = -1;
		this->FirstEncoderTick_direction = -1;
	}
	// clear timing buffers
	for(uint8_t i = 0; i < 3; i++)
		sat_timing_buffers[i].counter = 0;
		
	// disable encoder interrupts
	EIMSK &= ~(0b11);
	
	// initialize AC
	// ADC0 from ADMUX & 0xF = 0
	ADMUX &= 0xF0;
	ADCSRB |= 1 << 6; // ADC0 input to comparator
	ADCSRB = (ADCSRB & ~(0b111)) | 0b000; // free-running mode
	ACSR |= // 1 << 3 | // enable interrupts
	        1 << 2; // | // enable AC input capture
	       // 0b11; // toggle triggers interrupt
	
	// setup TIMER0 for pushing square waves through the motor
	TCCR0A = 0b0000; // normal mode
	TCCR0B = 0b011; // 64x prescaling
	TIMSK0 = 0b011; // COMPA and OVF interrupts
	OCR0A = 40;
	
	// setup TIMER1 for timing the threshold crossings
	TCCR1A = 0b00 << 6 | // don't touch OC0A
	         0b00 << 4 | // don't touch OC0B
	         0b00000; // Normal mode
	TCCR1B |= 0b11 << 6; // input capture: rising edge with noise cancellation
   TCCR1B = (TCCR1B & ~(0b11)) | 0b01; // no prescaling
   TIMSK1 = 1 << 5; // only enable input capture interrupt
}
void InductanceAngleState::spin(void) {
	uint8_t buffers_full;
	do {
		buffers_full = 1;
		for(uint8_t i = 0; i < 2; i++) {
			if(sat_timing_buffers[i].counter < INDUCTANCE_ANGLE_SAMPLES) {
				buffers_full = 0;
				break;
			}
		}
	}
	while(!buffers_full);
	
	int16_t fwd_avg = 0, back_avg = 0;
	for(uint8_t i = 0; i < INDUCTANCE_ANGLE_SAMPLES; i++) {
		back_avg += (int16_t)(sat_timing_buffers[0].buffer[i]) - TIMING_MU;
		fwd_avg += (int16_t)(sat_timing_buffers[1].buffer[i]) - TIMING_MU;
		// eeprom_write_word((uint16_t*)(i << 2), sat_timing_buffers[0].buffer[i]);
		// eeprom_write_word((uint16_t*)((i << 2) + 2), sat_timing_buffers[1].buffer[i]);
	}
	back_avg /= -INDUCTANCE_ANGLE_SAMPLES;
	fwd_avg /= INDUCTANCE_ANGLE_SAMPLES;
	
	this->A_phase_angle = abs((uint16_t)(acos(SQRT3_OVER_2 * fwd_avg / sqrt(fwd_avg * fwd_avg + fwd_avg * back_avg + back_avg * back_avg)) * 256))
	                      + ((!this->FirstEncoderTick_direction) << 8)
	                      - 512 / 12; // 30deg, hope I got the sign right :S
	// eeprom_write_word((uint16_t*)(INDUCTANCE_ANGLE_SAMPLES << 2), 0xFFC0);
	// eeprom_write_word((uint16_t*)((INDUCTANCE_ANGLE_SAMPLES << 2) + 2), 0x00EE);
	// eeprom_write_byte((uint8_t*)((INDUCTANCE_ANGLE_SAMPLES << 2) + 4), this->A_phase_angle);
	
	// for(;;);
	
	// PORTD |= 1 << 4;
	PUT(OFF_MASK_P, OFF_MASK_N);
	// PORTB |= OFF_MASK;
	TIMSK1 = 0;
	TIMSK0 = 0;
	// for(;;);
		
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
	phase = (phase + 1) & 1;
	triggered = 0;
	
	TCNT1 = 0; // reset threshold timer
	
	TIMSK1 &= ~(1 << 5); // disable input capture interrupt
	ADMUX = (ADMUX & ~(0b11)) | this->adc[this->phase];
	
	PUT(OFF_MASK_P, OFF_MASK_N); // de-energize for a bit to avoid short
	PUT(PHASES[phase].p, PHASES[phase].n);
	TIMSK1 |= 1 << 5; // re-enable input capture interrupt
}
void InductanceAngleState::t0_compa(void) {
	PUT(RESTING[phase].p, RESTING[phase].n);
}
void InductanceAngleState::t1_capt(void) {
		
	if(!triggered && sat_timing_buffers[phase].counter < INDUCTANCE_ANGLE_SAMPLES) {
		// PORTB ^= 0b1000;
		sat_timing_buffers[phase].buffer[sat_timing_buffers[phase].counter++] = ICR1;
		triggered = 1;
	}
	ACSR |= 1 << 4;
}
