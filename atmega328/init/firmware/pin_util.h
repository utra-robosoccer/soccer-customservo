#ifndef _PIN_UTIL_H
#define _PIN_UTIL_H 1
#include <avr/io.h>

typedef struct commutation_t {
	uint8_t p, n;
} commutation_t;

#define PUT(p, n) { \
	uint8_t p_ = ~p; \
	PORTD = (PORTD & 0b10011111) | ((p_ & 0b1) << 6) | ((p_ & 0b10) << 4), \
	PORTB = (PORTB & 0b11110111) | ((p_ & 0b100) << 1), \
	PORTC = (PORTC & 0b11000111) | (n << 3); \
}

#endif