#ifndef _FIRST_ENCODER_TICK_STATE_H
#define _FIRST_ENCODER_TICK_STATE_H 1

#include "../main.h"
#include "../State.h"

class FirstEncoderTickState : public State {
	public:
		static const uint8_t PHASES[3]; // separated by 120deg, one of these ought to work!
	
	private:
		static const program_state_t STATE_ENUM = FIRST_ENCODER_TICK;
		uint8_t phase;
	
	public:
		FirstEncoderTickState(void);
		void setup(const State& previous_state);
		void spin(void);
		inline void teardown(void) {}
		void ISR_entry(uint8_t ISR_num);
	
	private:
		inline void t0_ovf(void) {
			
		}
		inline void t0_compa(void) {
			
		}
};

#endif