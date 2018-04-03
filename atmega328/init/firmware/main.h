#ifndef MAIN_H
#define MAIN_H 1

#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdlib.h>

#include "constants.h"
#include "State.h"
#include "States/WakeupState.h"
#include "States/FirstEncoderTickState.h"
#include "States/InductanceAngleState.h"
// #include "States/SecondEncoderTickState.h"
#include "States/RunningState.h"

#define MEMBER_FN(obj, member) ((obj).*(member))

//////////////
// PIN UTIL //
//////////////

static inline void put_commutation(uint8_t c) {
	// ATOMIC_BLOCK(ATOMIC_FORCEON) {
		// PORTD = (PORTD & ~(0b111 << 5)) | ((c & 0b111) << 5); // so long as this is coming from resting, it shouldn't be possible to short (LSb of PORTB should be 0)
		// PORTB = (PORTB & ~(0b111)) | ((c >> 3) & 0b111);
	// }
	PORTB = (PORTB & ~(0b111111)) | (c & 0b111111);
}

// ISR_t* ISR_registry[NUM_VECTORS];

extern State* states[NUM_PROGRAM_STATES];

// static inline void register_ISR(uint8_t vect, program_state_t state, ISR_t fn) {
// 	if(state < NUM_PROGRAM_STATES && vect < NUM_VECTORS) {
// 		if(ISR_registry[vect] == 0) {
// 			ISR_registry[vect] = (ISR_t*)malloc(NUM_PROGRAM_STATES * sizeof(ISR_t));
// 		}
// 		ISR_registry[vect][state] = fn;
// 	}
// }
static inline void hit_registry(uint8_t vect) {
	states[PROGRAM_STATE]->ISR_entry(vect);
}

#endif