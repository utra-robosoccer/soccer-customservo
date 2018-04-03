/* Name: main.c
 * Author: <insert your name here>
 * Copyright: <insert your copyright message here>
 * License: <insert your license reference here>
 */

#include <util/delay.h>
#include <util/atomic.h>
#include <avr/eeprom.h>

#include "main.h"

State* states[NUM_PROGRAM_STATES];

ISR(TIMER0_OVF_vect) {
	// PORTB = 0b110101;
	hit_registry(TIMER0_OVF_vect_num);
}
ISR(TIMER0_COMPA_vect) {
	hit_registry(TIMER0_COMPA_vect_num);
}
ISR(TIMER0_COMPB_vect) {
	hit_registry(TIMER0_COMPB_vect_num);
}
ISR(TIMER1_CAPT_vect) {
	hit_registry(TIMER1_CAPT_vect_num);
}
ISR(TIMER1_COMPB_vect) {
	hit_registry(TIMER1_COMPB_vect_num);
}

int8_t SPEED = 60;
program_state_t PROGRAM_STATE = WAKEUP;

int main(void)
{
	WakeupState s0 = WakeupState();
	FirstEncoderTickState s1 = FirstEncoderTickState();
	InductanceAngleState s2 = InductanceAngleState();
	// SecondEncoderTickState s3 = SecondEncoderTickState();
	RunningState s3 = RunningState();
	
	states[0] = &s0;
	states[1] = &s1;
	states[2] = &s2;
	states[3] = &s3;
	
	sei();

	program_state_t last_state_enum = PROGRAM_STATE;
	State* current_state;
	for(;;) {
		current_state = states[PROGRAM_STATE];
		(*current_state).setup(*states[last_state_enum]);
		last_state_enum = PROGRAM_STATE;
		(*current_state).spin();
		(*current_state).teardown();
	}
	return 0;   /* never reached */
}
