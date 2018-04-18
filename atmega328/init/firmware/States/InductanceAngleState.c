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
	{ .p = 0b010, .n = 0b100 },
	{ .p = 0b010, .n = 0b001 }
};
const commutation_t InductanceAngleState::RESTING[2] = {
	{ .p = 0b000, .n = 0b100 },
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
	for(uint8_t i = 0; i < 2; i++)
		this->sat_timing_buffers[i].counter = 0;
	
	// disable encoder interrupts
	EIMSK &= ~(0b11);
	
	TCCR0A &= 0x0F;
	TCCR1A &= 0x0F;
	TCCR2A &= 0x0F;
	PUT(OFF_MASK_P, OFF_MASK_N);
	
	// // initialize AC
	// // ADC0 from ADMUX & 0xF = 0
	// ADMUX &= 0xF0;
	// ADCSRB |= 1 << 6; // ADC0 input to comparator
	// ADCSRB = (ADCSRB & ~(0b111)) | 0b000; // free-running mode
	// ACSR |= // 1 << 3 | // enable interrupts
	//         1 << 2; // | // enable AC input capture
	//        // 0b11; // toggle triggers interrupt
	
	// setup TIMER0 for pushing square waves through the motor
	TCCR0A = (0b11 << 4) | 0b11; // fast-PWM
	TCCR0B = 0b100; // 64x prescaling
	TIMSK0 = 0b101; // COMPB and OVF interrupts
	OCR0B = 2;
	
	// setup TIMER1 for timing the threshold crossings
	TCCR1A = 0b00 << 6 | // don't touch OC0A
	         0b00 << 4 | // don't touch OC0B
	         0b00000; // Normal mode
	TCCR1B |= 0b10 << 6; // input capture: falling edge with noise cancellation
   TCCR1B = (TCCR1B & ~(0b11)) | 0b01; // no prescaling
   TIMSK1 = 1 << 5; // only enable input capture interrupt
}
void InductanceAngleState::spin(void) {
	// for(uint16_t i = 0; i < 512; i++) {
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
		// PORTB |= 1 << 5;
		
		int16_t fwd_avg = 0, back_avg = 0;
		for(uint8_t j = 0; j < INDUCTANCE_ANGLE_SAMPLES; j++) {
			back_avg += sat_timing_buffers[0].buffer[j];
			fwd_avg += sat_timing_buffers[1].buffer[j];
			// eeprom_write_word((uint16_t*)(((i * INDUCTANCE_ANGLE_SAMPLES) << 2) + (j << 2)), back_avg);
			// eeprom_write_word((uint16_t*)(((i * INDUCTANCE_ANGLE_SAMPLES) << 2) + (j << 2) + 2), fwd_avg);
		}
		back_avg = -back_avg / INDUCTANCE_ANGLE_SAMPLES + TIMING_MU;
		fwd_avg = fwd_avg / INDUCTANCE_ANGLE_SAMPLES - TIMING_MU;
		
		this->A_phase_angle = (int16_t)(asin(SQRT3_OVER_2 * fwd_avg / sqrt(fwd_avg * fwd_avg + fwd_avg * back_avg + back_avg * back_avg)) * NUM_HALF_SINE / M_PI)
		                      + ((!this->FirstEncoderTick_direction) << 8)
		                      - (NUM_HALF_SINE << 1) / 12; // 30deg, hope I got the sign right :S
		// eeprom_write_word((uint16_t*)(i << 1), this->A_phase_angle);
		
		// for(uint8_t j = 0; j < 2; j++)
		// 	this->sat_timing_buffers[j].counter = 0;
	// }
	// eeprom_write_word((uint16_t*)((i << 2) + 2), back_avg);
	
		                      
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
		case TIMER0_COMPB_vect_num:
			this->t0_compb();
			return;
		case TIMER1_CAPT_vect_num:
			this->t1_capt();
			return;
	}
}

void InductanceAngleState::t0_ovf(void) {
	triggered = 0;
	TCNT1 = 0; // reset threshold timer
	// TIMSK1 |= 1 << 5; // re-enable input capture interrupt
}
void InductanceAngleState::t0_compb(void) {
	// PORTB |= 1 << 5;
	// phase = (phase + 1) & 3; // how it should be done
	phase = !phase; // how we're testing for the meantime
	// PUT(OFF_MASK_P, OFF_MASK_N); // de-energize for a bit to avoid short
	// PUT(PHASES[phase].p, PHASES[phase].n);
	PORTC = (PORTC & 0b11000111) | (PHASES[phase].n << 3);
}
void InductanceAngleState::t1_capt(void) {
	if(!triggered && sat_timing_buffers[phase].counter < INDUCTANCE_ANGLE_SAMPLES) {
		// PORTB ^= 0b1000;
		sat_timing_buffers[phase].buffer[sat_timing_buffers[phase].counter++] = ICR1;
		triggered = 1;
	}
	ACSR |= 1 << 4;
}
