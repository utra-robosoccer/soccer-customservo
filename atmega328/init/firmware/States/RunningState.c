#include <util/atomic.h>

#include "../math_util.h"
#include "../State.h"
#include "../Encoder.h"
#include "RunningState.h"
#include "InductanceAngleState.h"

RunningState::RunningState(void) {
	// register_ISR(TIMER0_OVF_vect_num, STATE_ENUM, this->t0_ovf);
	// register_ISR(TIMER0_COMPA_vect, STATE_ENUM, this->t0_compa);
	// register_ISR(TIMER0_COMPB_vect, STATE_ENUM, this->t0_compb);
	// register_ISR(TIMER1_COMPB_vect, STATE_ENUM, this->t1_compb);
}
void RunningState::setup(const State& previous_state) {
	this->valid_speed_buffer_samples = 0; // invalidate the whole speed buffer, e.g. if we had to recalibrate the position
		
	if(previous_state.state_type() == INDUCTANCE_ANGLE) {
		// this->FirstEncoderTick_phase = ((const FirstEncoderTickState&)previous).phase;
		this->A_phase_angle = ((const InductanceAngleState&)previous_state).A_phase_angle;
	}
	
	// setup TIMER0 COMPA and COMPB + TIMER1 COMPB to publish to phases,
	// setup TIMER2 to estimate the velocity
	
	EIMSK |= 0b11; // enable INT1 and INT0
	
	// GTCCR = 1 << 7 | 0b11; // synchronize TIMER0 and TIMER1
	
	TCCR0A = 0b11;
	TCCR0B = 0b10; // 8x prescaling (47kHz PWM)
	TCNT0 = 0;
	// OCR0A = 250; // temp, to test one phase
	TIMSK0 = 0b101;
	// TIMSK0 = 0b111; // enable all interrupts, including overflow (this will beat the drum)
	
	// use TIMER2 and OC2A for the third phase: this is totally
	// okay aside from some minute noise in the harmonics because
	// the PWM edges don't really need to be aligned; the
	// grounded nodes are always grounded
	
	TCCR2A = 0b11;
	TCCR2B = 0b10; // 8x prescaling
	// TIMSK2 = 0b000;
	TIMSK2 = 0b000; // enable TOV interrupts only (will do the work of condensing OCR2A)
	
	// use TIMER1 as a high-resolution clock for velocity
	// estimation 
	TCCR1A = 0b00;
	TCCR1B = 0b010; // normal + 8x prescaling
	TCNT1 = 0;
	
	EICRA &= ~(0b11 << 2);
	EIMSK &= ~(1 << 1);
	
	// TEMP
	SET_PHASE(this->A_phase_angle);
	// PORTB |= 1 << 5;
	
	// for(;;);
	
	// GTCCR &= ~(1 << 7);
}
void RunningState::spin() {
	uint16_t encoder_bias = ENCODER_DIV_MOTOR_GCD << 3;
	ATOMIC_BLOCK(ATOMIC_FORCEON) {
		Encoder::position = encoder_bias; // we can miss 8 rotations: this is really generous tbh
	}
	
	uint16_t last_encoder = 0;
	uint16_t current_encoder;
	uint16_t last_time = TCNT1;
	uint16_t last_period = 1024; // baseline period: 1024 TCNT1 for an encoder tick: pretty slow.
	uint8_t majority = 0;
	for(;;) {
		// given a velocity and the last time switch (deterministic given the velocity estimate at a given time)
		
		// possible to be either encoder-driven or timer-driven
		// whenever the encoder ticks, we update to the expected phase
		// somehow, this has to be done without drifting either the local
		// encoder states or the A phase angle. Will probably be safest
		// to keep these constant and do relative compensation based only
		// on the encoder count (which is correct up to modulo)
		
		current_encoder = Encoder::position;
		if(current_encoder > encoder_bias + ENCODER_DIV_MOTOR_GCD || current_encoder < encoder_bias) {
			ATOMIC_BLOCK(ATOMIC_FORCEON) {
				current_encoder = (encoder_bias) + (Encoder::position % ENCODER_DIV_MOTOR_GCD);
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
				
				uint16_t next_position = A_phase_angle + ((uint32_t)current_encoder * NUM_HALF_SINE) * MOTOR_PHASES_PER_ENCODER_TICK;
				
				ATOMIC_BLOCK(ATOMIC_FORCEON) {
					SET_PHASE(next_position);
				}
				majority = 0;
				last_encoder = current_encoder;
			}
		}
		else {
			majority--;
			// since the encoder is so fine, with the 1600CPR, there's really no point here since
			// there are almost as many ticks per motor phase as there are increments of HALF_SINE
			
			// ATOMIC_BLOCK(ATOMIC_FORCEON) {
			// 	uint16_t last_T1 = TCNT1;
			// }
			// // yeah this calculation will definitely overflow into int32
			// uint16_t next_position = A_phase_angle + ((last_encoder % ENCODER_DIV_MOTOR_GCD) * (NUM_HALF_SINE << 1)) * MOTOR_PHASES_PER_ENCODER_TICK +
			//                          last_T1 * (NUM_HALF_SINE << 1) * MOTOR_PHASES_PER_ENCODER_TICK / last_period;
			// ATOMIC_BLOCK(ATOMIC_FORCEON) {
			// 	SET_PHASE(next_position);
			// }
		}
	}
	// if(Encoder::position > 18) {
	// 	uint16_t current_t1 = TCNT1;
	// 	uint16_t 
	// 	if(this->valid_speed_buffer_samples < SPEED_BUFFER_SIZE)
	// 		this->valid_speed_buffer_samples++;
		
	// 	this->speed_buffer_idx = (this->speed_buffer_idx + 1) % SPEED_BUFFER_SIZE;
	// 	this->speed_buffer[this->speed_buffer_idx] = TCNT1 / Encoder::position;
	// 	Encoder::position %= 18;
	// }
}
void RunningState::ISR_entry(uint8_t vect) {
	switch(vect) {
		case TIMER2_OVF_vect_num:
			this->t2_ovf();
			return;
	}
}