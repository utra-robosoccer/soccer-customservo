#ifndef _STATE_H
#define _STATE_H 1

#include <avr/io.h>

#define NUM_PROGRAM_STATES 5
#define NUM_VECTORS 26

typedef enum program_state_t
{
	WAKEUP = 0,
	FIRST_ENCODER_TICK = 1,
	INDUCTANCE_ANGLE = 2,
	SECOND_ENCODER_TICK = 3,
	RUNNING = 4
} program_state_t;

extern program_state_t PROGRAM_STATE;

/////////
// ISR //
/////////

typedef void (*ISR_t)(void);

class State {
		
	public:
		virtual void setup(const State& previous);
		virtual void spin(void);
		virtual void teardown(void);
		virtual void ISR_entry(uint8_t ISR_num);
		virtual program_state_t state_type(void) const;
};

#endif