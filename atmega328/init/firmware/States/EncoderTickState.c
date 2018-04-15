#include "EncoderTickState.h"

const commutation_t EncoderTickState::PHASES[2] = {
	{ .p = 0b100, .n = 0b010 }, // REVERSE
	{ .p = 0b100, .n = 0b001 } // FORWARD
};
const commutation_t EncoderTickState::RESTING[2] = {
	{ .p = 0b000, .n = 0b010 },
	{ .p = 0b000, .n = 0b001 }
};