#ifndef _INDUCTANCE_ANGLE_STATE_H
#define _INDUCTANCE_ANGLE_STATE_H 1

#include "../State.h"
#include "../pin_util.h"

class InductanceAngleState: public State {
	// logic for rectangular pulses found in TIMER1 OVF and COMPA
	static const commutation_t PHASES[2];
	static const uint8_t adc[2];
	static const commutation_t RESTING[2];
	
	public:
		// state exports
		int8_t A_phase_angle;
		
		// state import-exports
		// int8_t FirstEncoderTick_phase;
		int8_t FirstEncoderTick_direction;
		
	protected:
		static const program_state_t STATE_ENUM = INDUCTANCE_ANGLE;
	private:
		volatile uint8_t phase;
		volatile uint8_t triggered;
		
		typedef struct saturation_timing_buffer_t {
			uint16_t buffer[INDUCTANCE_ANGLE_SAMPLES];
			volatile uint16_t counter = 0;
		} saturation_timing_buffer_t;
		saturation_timing_buffer_t sat_timing_buffers[2];
	
	public:
		InductanceAngleState(void);
		void setup(const State& previous_state);
		void spin(void);
		inline void teardown(void) {}
		void ISR_entry(uint8_t ISR_num);
		inline program_state_t state_type(void) const {
			return INDUCTANCE_ANGLE;
		}
	
	private:
		void t0_ovf(void);
		void t0_compa(void);
		void t1_capt(void);
};

#endif