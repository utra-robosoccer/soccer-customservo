#ifndef _INDUCTANCE_ANGLE_STATE_H
#define _INDUCTANCE_ANGLE_STATE_H 1

#include "../State.h"

class InductanceAngleState: public State {
	// logic for rectangular pulses found in TIMER1 OVF and COMPA
	static const uint8_t PHASES[3];
	static const uint8_t adc[3];
	static const uint8_t RESTING[3];
	
	static const program_state_t STATE_ENUM = INDUCTANCE_ANGLE;
	
	public:
		typedef struct saturation_timing_buffer_t {
			uint16_t buffer[INDUCTANCE_ANGLE_SAMPLES];
			volatile uint16_t counter;
		} saturation_timing_buffer_t;
		saturation_timing_buffer_t sat_timing_buffers[3];
		
	private:
		volatile uint8_t phase;
		volatile uint8_t triggered;
	
	public:
		InductanceAngleState(void);
		void setup(const State& previous_state);
		void spin(void);
		inline void teardown(void) {}
		void ISR_entry(uint8_t ISR_num);
	
	private:
		void t0_ovf(void);
		void t0_compa(void);
		void t1_capt(void);
};

#endif