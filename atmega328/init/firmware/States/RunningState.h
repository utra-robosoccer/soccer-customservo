#ifndef _RUNNING_STATE_H
#define _RUNNING_STATE_H 1

#include "../State.h"
#include "../constants.h"

class RunningState : public State {
	static const program_state_t STATE_ENUM = RUNNING;
	static const uint8_t SPEED_BUFFER_SIZE = 10;
	
	uint16_t speed_buffer[SPEED_BUFFER_SIZE];
	uint8_t valid_speed_buffer_samples;
	uint8_t speed_buffer_idx;
	
	// speed predictor
	uint16_t last_captured_phase;
	
	// imports
	uint16_t A_phase_angle;
		
	public:
		RunningState(void);
		void setup(const State& previous_state);
		void spin(void);
		inline void teardown(void) {}
		void ISR_entry(uint8_t ISR_num);
		inline program_state_t state_type(void) const {
			return RUNNING;
		}
		
	private:
		// inline void t0_ovf(void) {
		// 	PORTB &= OFF_MASK;
		// }
		// inline void t0_compa(void) {
		// 	if((this->A_phase_angle >> 8) & 1)
		// 		PORTB |= 0b11;
		// 	else
		// 		PORTB &= 0b111100;
		// }
		// inline void t0_compb(void) {
		// 	if(((this->A_phase_angle + 171) >> 8) & 1)
		// 		PORTB |= 0b1100;
		// 	else
		// 		PORTB &= 0b110011;
		// }
		// inline void t1_compb(void) {
		// 	if(((this->A_phase_angle + 341) >> 8) & 1)
		// 		PORTB |= 0b110000;
		// 	else
		// 		PORTB &= 0b001111;
		// }
		inline void t2_compa(void) {
			// 
		}
		inline void t2_ovf(void) {
		}
};

#endif