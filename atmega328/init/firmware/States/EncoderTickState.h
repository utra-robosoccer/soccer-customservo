#ifndef _ENCODER_TICK_STATE_H
#define _ENCODER_TICK_STATE_H 1

#include "../State.h"
#include "../pin_util.h"

class EncoderTickState : public State { // abstract
	public:
		static const commutation_t PHASES[2];
		static const commutation_t RESTING[2];
		uint8_t phase = 0;
	
	public:
		virtual void setup(const State& previous);
		virtual void spin(void);
		virtual void teardown(void);
		virtual void ISR_entry(uint8_t ISR_num);
};

#endif