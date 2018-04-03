#ifndef _FIRST_ENCODER_TICK_STATE_H
#define _FIRST_ENCODER_TICK_STATE_H 1

#include "EncoderTickState.h" // import/share PHASES and RESTING

class FirstEncoderTickState : public EncoderTickState {
	public:
		// state exports
		uint8_t direction; // 0 for backwards, 1 for forwards
	protected:
		static const program_state_t STATE_ENUM = FIRST_ENCODER_TICK;
	
	public:
		FirstEncoderTickState(void);
		void setup(const State& previous_state);
		void spin(void);
		inline void teardown(void) {}
		void ISR_entry(uint8_t ISR_num);
		inline program_state_t state_type(void) const {
			return FIRST_ENCODER_TICK;
		}
};

#endif