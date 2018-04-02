#include "../main.h"
#include "../State.h"
#include "WakeupState.h"

WakeupState::WakeupState(void) {}
void WakeupState::setup(const State& previous_state) {
	port_init();
	timer_init();
	ac_init();
}
void WakeupState::ISR_entry(uint8_t vect) {}