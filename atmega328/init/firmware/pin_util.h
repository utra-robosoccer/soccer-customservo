#include <avr/io.h>

static inline void put_commutation(uint8_t c) {
	// ATOMIC_BLOCK(ATOMIC_FORCEON) {
		// PORTD = (PORTD & ~(0b111 << 5)) | ((c & 0b111) << 5); // so long as this is coming from resting, it shouldn't be possible to short (LSb of PORTB should be 0)
		// PORTB = (PORTB & ~(0b111)) | ((c >> 3) & 0b111);
	// }
	PORTC = (PORTC & ~(0b111111)) | (c & 0b111111);
}