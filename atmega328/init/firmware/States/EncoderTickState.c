#include "EncoderTickState.h"

const uint8_t EncoderTickState::PHASES[2] = {
	0b110100, // REVERSE
	0b110001 // FORWARD
};
const uint8_t EncoderTickState::RESTING[2] = {
	0b010100,
	0b010001
};