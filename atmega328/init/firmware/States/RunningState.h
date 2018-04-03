#ifndef _RUNNING_STATE_H
#define _RUNNING_STATE_H 1

#include "../State.h"

class RunningState : public State {
	static const program_state_t STATE_ENUM = RUNNING;
	static const uint8_t SPEED_BUFFER_SIZE = 10;
	
	uint16_t speed_buffer[SPEED_BUFFER_SIZE];
	uint8_t valid_speed_buffer_samples;
	uint8_t speed_buffer_idx;
		
	public:
		RunningState(void);
		void setup(const State& previous_state);
		void spin(void);
		void teardown(void);
		void ISR_entry(uint8_t ISR_num);
		inline program_state_t state_type(void) const {
			return RUNNING;
		}
		
	private:
		void t0_ovf(void);
		void t0_compa(void);
		void t0_compb(void);
		void t1_compb(void);
};

#endif