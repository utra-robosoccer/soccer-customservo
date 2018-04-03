#include "../State.h"
#include "WakeupState.h"

WakeupState::WakeupState(void) {}
void WakeupState::setup(const State& previous_state) {
	this->port_init();
	this->timer_init();
	this->ac_init();
}
void WakeupState::spin() {
	PROGRAM_STATE = FIRST_ENCODER_TICK;
}
void WakeupState::ISR_entry(uint8_t vect) {}