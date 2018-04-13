#include "EncoderTickState.h"

const uint8_t EncoderTickState::PHASES[2] = {
	0b011100, // REVERSE
	0b001101 // FORWARD
};
const uint8_t EncoderTickState::RESTING[2] = {
	0b010100,
	0b000101
};