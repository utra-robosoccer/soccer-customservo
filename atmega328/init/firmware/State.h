#ifndef _STATE_H
#define _STATE_H 1

#define NUM_PROGRAM_STATES 4
#define NUM_VECTORS 26

typedef enum program_state_t
{
	WAKEUP = 0,
	FIRST_ENCODER_TICK = 1,
	INDUCTANCE_ANGLE = 2,
	RUNNING = 3
} program_state_t;

extern program_state_t PROGRAM_STATE;

class State {
	protected:
		static const program_state_t STATE_ENUM;
	public:
		virtual void setup(const State& previous);
		virtual void spin(void);
		virtual void teardown(void);
		virtual void ISR_entry(uint8_t ISR_num);
};

#endif