#include "../main.h"
#include "../State.h"
#include "RunningState.h"

RunningState::RunningState(void) {
	// register_ISR(TIMER0_OVF_vect_num, STATE_ENUM, this->t0_ovf);
	// register_ISR(TIMER0_COMPA_vect, STATE_ENUM, this->t0_compa);
	// register_ISR(TIMER0_COMPB_vect, STATE_ENUM, this->t0_compb);
	// register_ISR(TIMER1_COMPB_vect, STATE_ENUM, this->t1_compb);
}
void RunningState::setup(const State& previous_state) {
	// setup TIMER0 COMPA and COMPB + TIMER1 COMPB to publish to phases,
	// setup TIMER2 to estimate the velocity
	
}
void RunningState::spin() {
	if(this->valid_speed_buffer_samples < SPEED_BUFFER_SIZE)
		this->valid_speed_buffer_samples++;
	
	this->speed_buffer_idx = (this->speed_buffer_idx + 1) % SPEED_BUFFER_SIZE;
	this->speed_buffer[this->speed_buffer_idx] = TCNT1 / Encoder::position;
	TCNT1 = 0;
	Encoder::position = 0;
}
void RunningState::teardown() {}
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
	}
}
void RunningState::t0_ovf(void) {}
void RunningState::t0_compa(void) {}
void RunningState::t0_compb(void) {}
void RunningState::t1_compb(void) {}