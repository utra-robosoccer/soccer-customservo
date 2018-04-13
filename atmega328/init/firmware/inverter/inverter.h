#include <avr/io.h>

volatile uint16_t phase_idx[3] = {
	0, 171, 341
};
const uint8_t RESTING[3] = {
	0b010100,
	0b010001,
	0b000101
};

// static inline void phase_rest(uint8_t idx) {
// 	put_commutation(RESTING[idx][(phase_idx[idx] >> 8) & 1]);
// }
