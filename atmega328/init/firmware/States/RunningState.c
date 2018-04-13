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
	PORTD |= 1 << 4;
		
	if(previous_state.state_type() == INDUCTANCE_ANGLE) {
		// this->FirstEncoderTick_phase = ((const FirstEncoderTickState&)previous).phase;
		this->A_phase_angle = ((const InductanceAngleState&)previous_state).A_phase_angle;
	}
	
	// setup TIMER0 COMPA and COMPB + TIMER1 COMPB to publish to phases,
	// setup TIMER2 to estimate the velocity
	
	EIMSK |= 0b11; // enable INT1 and INT0
	
	GTCCR = 1 << 7 | 0b11; // synchronize TIMER0 and TIMER1
	
	TCCR0A = 0b11;
	TCCR0B = 0b10; // 8x prescaling (47kHz PWM)
	TCNT0 = 0;
	// OCR0A = 250; // temp, to test one phase
	TIMSK0 = 0b101;
	// TIMSK0 = 0b111; // enable all interrupts, including overflow (this will beat the drum)
	
	TCCR1A = 0b00;
	TCCR1B = 0b01 << 3 | 0b010; // normal + 8x prescaling
	OCR1A = 0xFF;
	TCNT1 = 0;
	// TIMSK1 = 0b100; // enable CMPB interrupt
	
	TCCR2A = 0b10;
	TCCR2B = 0b100; // OCR2A sets top + 256x prescaling, 732Hz (for 512-valued sine, motor phase is 1.4Hz to begin with)
	OCR2A = 0xFF; // 0xFF; // start with max delay
	// TIMSK2 = 0b000;
	TIMSK2 = 0b001; // enable TOV interrupts only (will do the work of condensing OCR2A)
	
	// TEMP
	OCR0A = 256 - (HALF_SINE[this->A_phase_angle & 0xFF] >> 2);
	OCR0B = 256 - (HALF_SINE[(this->A_phase_angle + 171) & 0xFF] >> 2);
	OCR1B = 256 - (HALF_SINE[(this->A_phase_angle + 341) & 0xFF] >> 2);
	
	GTCCR &= ~(1 << 7);
}
void RunningState::spin() {
	for(;;);
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
		case TIMER0_OVF_vect_num:
			this->t0_ovf();
			return;
		case TIMER0_COMPA_vect_num:
			this->t0_compa();
			return;
		case TIMER0_COMPB_vect_num:
			this->t0_compb();
			return;
		case TIMER1_COMPB_vect_num:
			this->t1_compb();
			return;
		case TIMER2_OVF_vect_num:
			this->t2_ovf();
			return;
	}
}